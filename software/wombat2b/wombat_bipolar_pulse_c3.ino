/*
  wombat2 - bipolar pulse-induction metal detector
  Seeed Studio XIAO ESP32-C3 (RISC-V @ 160 MHz)

  This file: bipolar pulse + flyback/damping timing, 40 kHz timer ISR, fast GPIO
  via the C3 atomic set/clear registers, live serial tuning of the damping timing.
  ADC sampling is deliberately NOT in the ISR (see loop() and the note at bottom).

  arduino-esp32 core 3.x (a core-2.x timer block is included). MIT (see LICENSE).
*/

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include "soc/soc.h"
#include "soc/gpio_reg.h"
#include "esp_cpu.h"

// ------------------------------------------------------------------ pin map
// XIAO ESP32-C3, as-built board (2026-07-04).
// GPIO6/7 (D4/D5) are the default Wire pins -> SDA/SCL to the ADS1115.
// GPIO2 is a strapping pin (must be HIGH at boot): the low-left FET is likely
// ON from power-up until setup() runs. High-side gates (active-LOW) need
// external pull-ups so they stay OFF while the C3 is in reset.
#define PIN_HIGH_LEFT   4   // A2/D2  high-side LEFT  (CCW)
#define PIN_HIGH_RIGHT  5   // A3/D3  high-side RIGHT (CW)
#define PIN_LOW_LEFT    2   // A0/D0  low-side  LEFT
#define PIN_LOW_RIGHT   3   // A1/D1  low-side  RIGHT
#define PIN_DEBUG       20  // D7     scope this (UART RX, unused - serial is USB-CDC)

// TMUX1108 8:1 sample MUX, VDD = 5 V -> 4 hold caps on channels S1..S4.
// A2 tied LOW (only S1-S4 used; ground the unused S5-S8 inputs). Logic inputs
// are 1.8 V-compatible at any supply, so 3.3 V GPIO drives them directly.
// EN is ACTIVE-HIGH (opposite of the 74HC4051): EN LOW = all channels off.
// GPIO10 floats during boot -> fit an external ~100k pull-DOWN on EN so the
// MUX is disconnected until setup() runs. GPIO9 (A1) has a boot pull-up -
// harmless while EN is low; never add an external pull-down to GPIO9
// (strapping pin, must be HIGH at boot).
#define PIN_MUX_A0      8   // D8   TMUX1108 A0 (address LSB)
#define PIN_MUX_A1      9   // D9   TMUX1108 A1
#define PIN_MUX_EN      10  // D10  TMUX1108 EN (ACTIVE-HIGH enable)

// ------------------------------------------------------------- fast GPIO ops
// Atomic set / clear via the C3's W1TS / W1TC registers.
#define GPIO_SET(p)  REG_WRITE(GPIO_OUT_W1TS_REG, 1UL << (p))
#define GPIO_CLR(p)  REG_WRITE(GPIO_OUT_W1TC_REG, 1UL << (p))

// High-side ACTIVE-LOW, low-side ACTIVE-HIGH.
#define HIGH_LEFT_ON    GPIO_CLR(PIN_HIGH_LEFT)
#define HIGH_LEFT_OFF   GPIO_SET(PIN_HIGH_LEFT)
#define HIGH_RIGHT_ON   GPIO_CLR(PIN_HIGH_RIGHT)
#define HIGH_RIGHT_OFF  GPIO_SET(PIN_HIGH_RIGHT)
#define LOW_LEFT_ON     GPIO_SET(PIN_LOW_LEFT)
#define LOW_LEFT_OFF    GPIO_CLR(PIN_LOW_LEFT)
#define LOW_RIGHT_ON    GPIO_SET(PIN_LOW_RIGHT)
#define LOW_RIGHT_OFF   GPIO_CLR(PIN_LOW_RIGHT)
#define D2_HIGH         GPIO_SET(PIN_DEBUG)
#define D2_LOW          GPIO_CLR(PIN_DEBUG)

// ---- TMUX1108 ops: pure register writes, safe inside the ISR ---------------
// Sample-cap addressing (A0,A1):  cap1=00  cap2=10  cap3=01  cap4=11
// i.e. cap index n (0..3): A0 = bit0, A1 = bit1 -> cap n on channel S(n+1)
// (A2A1A0 = 000..011). With a constant n the compiler folds each MUX_ADDR()
// to two register stores - no branches, no calls.
#define MUX_DISCONNECT  GPIO_CLR(PIN_MUX_EN)   // EN LOW:  all channels off
#define MUX_CONNECT     GPIO_SET(PIN_MUX_EN)   // EN HIGH: selected channel on
#define MUX_ADDR(n) do { \
    if ((n) & 1) { GPIO_SET(PIN_MUX_A0); } else { GPIO_CLR(PIN_MUX_A0); } \
    if ((n) & 2) { GPIO_SET(PIN_MUX_A1); } else { GPIO_CLR(PIN_MUX_A1); } \
  } while (0)

// --------------------------------------------------------------- pulse timing
// Set these two in MICROSECONDS. The ISR runs every 25 us (40 kHz), so both
// are quantized to 25 us steps - keep them multiples of 25.
// Detector pulse width: nominally 100 us, range 50-150 us, set-and-forget.
// NOTE: the coil turns ON ref_sample_width (~10 us) AFTER the start tick (the
// pre-pulse reference gate runs first) but turns OFF exactly on the width
// tick, so true coil ON time = PULSE_WIDTH_US - ~10 us. Constant every pulse,
// so it doesn't matter - just don't be surprised on the scope.
#define PULSE_WIDTH_US   (100)     // pulse ON window (start tick -> off tick)
#define PULSE_PERIOD_US  (250000)  // 250 ms -> 4 Hz slow (bipolar bring-up; set 2000 = 500 Hz for final)

// internal: convert to 25-us ISR ticks (what pulse_counter actually counts)
#define V_PULSE_WIDTH   (PULSE_WIDTH_US  / 25)
#define V_PULSE_PERIOD  (PULSE_PERIOD_US / 25)

// -------------------------------------------------------- NOP-sled busy delay
// Pure NOPs. No CSR reads, no calls. Argument is in CPU cycles (~6.25 ns each
// @ 160 MHz, 1 cycle == 1 NOP).
//
// Granularity is ONE cycle: the bulk is dispensed in lots of 16 NOPs by the
// loop, and the low 4 bits fall through a NOP sled so n -> n+1 adds exactly one
// NOP. The switch dispatch + loop overhead are a small CONSTANT offset (so the
// absolute delay is a bit longer than `cyc`); the *step* size is 1 NOP. Tune by
// scope - you care about moving the edge predictably, not the absolute number.
static inline void IRAM_ATTR delay_cyc(uint32_t cyc)
{
  uint32_t chunks = cyc >> 4;                 // bulk: lots of 16 NOPs
  while (chunks--)
    asm volatile("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
                 "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");

  switch (cyc & 0x0F) {                        // fine remainder: 1 NOP per step
    case 15: asm volatile("nop");
    case 14: asm volatile("nop");
    case 13: asm volatile("nop");
    case 12: asm volatile("nop");
    case 11: asm volatile("nop");
    case 10: asm volatile("nop");
    case  9: asm volatile("nop");
    case  8: asm volatile("nop");
    case  7: asm volatile("nop");
    case  6: asm volatile("nop");
    case  5: asm volatile("nop");
    case  4: asm volatile("nop");
    case  3: asm volatile("nop");
    case  2: asm volatile("nop");
    case  1: asm volatile("nop");
    case  0: ;
  }
}

static uint32_t g_cpu_mhz = 160;              // informational only

// ---- LIVE-TUNABLE damping parameters, in CYCLES (~6.25 ns each) -------------
// nudge over serial: + / - on delay, [ / ] on segment width
volatile uint32_t g_damp_delay_cyc = 48;  // pulse-end -> first damping switch-on (~300ns)
volatile uint32_t g_damp_seg_cyc   = 48;  // width of each damping segment        (~300ns)
#define FLYBACK_CYC (48)                  // flyback ring-up before damping (~300ns)

// ---- BENCH TEST MODE: simple unipolar pulse, NO active damping ---------------
// 1 = fire HIGH_LEFT + LOW_RIGHT on EVERY pulse (single direction, no bipolar
//     alternation). At pulse end the bridge just opens; the coil rings down
//     through the passive damping resistor only. LOW_LEFT is never asserted, so
//     there is NO active damping pulse. g_damp_delay_cyc / g_damp_seg_cyc and
//     the '+ - [ ]' serial keys are inert in this mode.
// 0 = full bipolar engine with mirrored active damping (the intended design).
#define SIMPLE_PULSE_NO_DAMP  1

// Within the tuned-clamp engine (SIMPLE_PULSE_NO_DAMP=1):
//   1 = BIPOLAR - alternate LEFT (HIGH_LEFT+LOW_RIGHT) and RIGHT (HIGH_RIGHT+LOW_LEFT)
//       diagonals every pulse; the discharge sequence mirrors (LOW_RIGHT on LEFT
//       pulses, LOW_LEFT on RIGHT), SAME g_hardoff_cyc / g_hold_ticks for both.
//   0 = unipolar bench mode: always LEFT.
#define BIPOLAR  1

// ---- SAMPLING REGIME: 3 decay windows + 1 pre-pulse reference ---------------
// All values in CPU cycles (~6.25 ns each @ 160 MHz). 1 us = 160 cycles.
//
// Decay windows (caps 1-3), timeline starts when damping finishes (FETs off):
//
//   |<-s1_start->|##s1##|<-s2_space->|##s2##|<-s3_space->|##s3##|
//                 cap1                cap2                cap3
//
// REF sample (cap4): gated in the pulse-START tick, immediately BEFORE the
// bridge fires, when the frontend output has been settled for ages. It is the
// DC reference subtracted from the other three. Fixed width = fixed offset to
// every pulse start, so pulse cadence and ON time are untouched.
//
// During each window the MUX connects the frontend output to that cap;
// otherwise the MUX is disconnected (caps hold). Runs on EVERY pulse, both
// directions; loop() reads the caps via ADS1115 every ~100+ pulses.
//
// BUDGET: the post-pulse sequence (damping + windows 1-3) runs inside one
// 25 us timer tick with interrupts masked. With the values below it totals
// ~30 us: the next 25 us alarm gets latched and serviced ~6 us late. That is
// harmless - it's a plain mid-period counting tick (no GPIO edges) and the
// timer autoreloads in hardware, so pulse start/end cadence is unaffected.
// HARD LIMIT: stay under 50 us (two ticks) or a count is LOST and the period
// stretches. The ref window has its own budget: < 25 us in the start tick.
#define US_TO_CYC(us)  ((uint32_t)((us) * 160))

volatile uint32_t sample1_start    = US_TO_CYC(8);   // damping-end -> window 1 opens
volatile uint32_t sample1_width    = US_TO_CYC(3);
volatile uint32_t sample2_space    = US_TO_CYC(0);   // back-to-back with window 1
volatile uint32_t sample2_width    = US_TO_CYC(3);
volatile uint32_t sample3_space    = US_TO_CYC(10);
volatile uint32_t sample3_width    = US_TO_CYC(6);
volatile uint32_t ref_sample_width = US_TO_CYC(10);  // cap4: settled-DC reference, pre-pulse

// SIMPLE_PULSE_NO_DAMP only: after the diagonal switches off (hard-off), wait
// this long with the bridge fully open (flyback rings), THEN switch LOW_RIGHT
// back on to pin the coil to ground. Live-tunable over serial with + / -.
volatile uint32_t g_hardoff_cyc = US_TO_CYC(2);      // ~2 us dead-off before the LOW_RIGHT discharge pulse (start point)
#define HARDOFF_MAX_CYC  US_TO_CYC(12)               // cap: hardoff+6us dual-low+~31us sampling ~=49us < 50us budget
// Value is a RAW cycle count, calibrated on the scope: delay_cyc() loop/dispatch
// overhead makes the measured ON-time longer than US_TO_CYC() nominal
// (US_TO_CYC(5)=800cyc measured ~8 us; 300cyc -> ~3 us). 600 cyc -> ~6 us measured.
#define DUAL_LOW_CYC     (600)                        // both low-side ON at the trough, ~6 us measured, then both OFF
// Pulse-end dials: '+'/'-' = dead-off timing (WHEN the low side turns on, fine,
// g_hardoff_cyc). '['/']' = HOLD LENGTH (how long the low side stays on grounding
// the coil end for measurement), in 25 us ticks - COARSE, tick-scheduled (not a
// busy-wait), so it can be long (~100 us) without blowing the ISR budget.
volatile uint32_t g_hold_ticks      = 4;   // low-side hold after hard-off (~100 us). live dial [ / ]
#define HOLD_TICKS_MAX   (20)              // cap ~500 us; must end well before the next pulse
volatile uint32_t g_lowside_off_tick = 0;  // ISR: pulse_counter value at which to drop the low side

// --- Non-volatile save of the tuned hard-off (clamp START) time --------------
// ESP32-C3 has no real EEPROM; Preferences uses the NVS flash partition, which
// SURVIVES a normal sketch re-upload (separate partition). Serial 's' saves the
// current g_hardoff_cyc; setup() restores it on boot. The hold length
// (g_hold_ticks) is intentionally NOT saved - it stays at its compiled default.
Preferences g_prefs;
#define NVS_NAMESPACE    "wombat"
#define NVS_KEY_HARDOFF  "hardoff"

// ------------------------------------------------ ADS1115 (16-bit I2C ADC)
// Reads the 4 held cap voltages. SLOW and that's fine: it runs from loop(),
// never the ISR. Assumed wiring: cap1..cap4 -> AIN0..AIN3 (edit ADS_CH if not).
// Single-shot, FSR +/-4.096 V (1 LSB = 125 uV), 860 SPS -> ~1.2 ms/conversion,
// ~5 ms for all four channels.
#define ADS1115_ADDR    0x48
#define ADS_READ_EVERY  100     // pulses between automatic read bursts ("100 or more")

// config word: OS=1(start) | MUX=100+ch(AINx vs GND) | PGA=001(4.096V) |
//              MODE=1(single-shot) | DR=111(860SPS) | comparator disabled
#define ADS_CFG(ch)  (0xC3E3 | ((uint16_t)(ch) << 12))

int16_t g_cap_raw[4];           // last successful read, raw counts

static bool ads1115_read_all()
{
  for (uint8_t ch = 0; ch < 4; ch++)
  {
    Wire.beginTransmission(ADS1115_ADDR);          // start conversion on AINch
    Wire.write(0x01);                              // config register
    Wire.write(ADS_CFG(ch) >> 8);
    Wire.write(ADS_CFG(ch) & 0xFF);
    if (Wire.endTransmission() != 0) return false;

    uint32_t t0 = millis();                        // poll OS bit: 1 = done
    for (;;)
    {
      Wire.beginTransmission(ADS1115_ADDR);
      Wire.write(0x01);
      if (Wire.endTransmission() != 0) return false;
      if (Wire.requestFrom((uint8_t)ADS1115_ADDR, (uint8_t)2) != 2) return false;
      uint16_t cfg = ((uint16_t)Wire.read() << 8) | Wire.read();
      if (cfg & 0x8000) break;                     // conversion complete
      if (millis() - t0 > 10) return false;        // should take ~1.2 ms
    }

    Wire.beginTransmission(ADS1115_ADDR);          // read conversion register
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) return false;
    if (Wire.requestFrom((uint8_t)ADS1115_ADDR, (uint8_t)2) != 2) return false;
    g_cap_raw[ch] = (int16_t)(((uint16_t)Wire.read() << 8) | Wire.read());
  }
  return true;
}

// ----------------------------------------------------------------- ISR state
hw_timer_t *pulseTimer = NULL;
portMUX_TYPE pulseMux  = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t pulse_counter = 0;
volatile bool     sampleReady   = false;   // set after the (current) phase completes
volatile bool     phase_left    = true;    // alternates every pulse: LEFT/CCW <-> RIGHT/CW

// loop() holds this true while it reads the ADS1115 over I2C. The ISR then
// leaves the MUX disconnected (EN stays high, caps hold rock-steady for the
// ADC) but the pulse + damping still run EXACTLY as always - coil current,
// supply-cap loading and discharge stay continuous and consistent. A few
// pulses simply go uncaptured; nobody minds.
volatile bool     doing_sampling = false;
volatile uint32_t pulse_events   = 0;      // total pulses fired (wraps; use deltas)

// =================================================================== the ISR
void IRAM_ATTR timer_pulse_Interrupt()
{
  pulse_counter++;

  // ---- end the long low-side hold, ~g_hold_ticks*25us after the pulse-end tick.
  // Tick-scheduled (not a busy-wait) so the hold can be ~100 us. One GPIO write.
  if (g_lowside_off_tick && pulse_counter == g_lowside_off_tick)
  {
    LOW_LEFT_OFF;
    LOW_RIGHT_OFF;
    g_lowside_off_tick = 0;
  }

  // ---------------------------------------------------------- END OF PULSE
  if (pulse_counter == V_PULSE_WIDTH)
  {
    portENTER_CRITICAL_ISR(&pulseMux);
    D2_HIGH;                              // scope marker: pulse end

#if SIMPLE_PULSE_NO_DAMP
    // Pulse-end sequence (bipolar, MIRRORED per direction):
    //   1. DEAD-OFF: whole bridge open, flyback rings.
    //   2. wait g_hardoff_cyc (fine, + / -, tuned+saved) - lands on the voltage peak.
    //   3. low side ON, grounding the coil end for measurement. LEFT diagonal uses
    //      LOW_RIGHT, RIGHT diagonal uses LOW_LEFT. Stays ON for ~g_hold_ticks*25us
    //      (~100 us), dropped by the tick-scheduled check at the top of the ISR
    //      (NOT a busy-wait, so it can be long). Sampling runs during the hold.
    LOW_LEFT_OFF;
    LOW_RIGHT_OFF;
    HIGH_LEFT_OFF;  HIGH_RIGHT_OFF;      // dead-off: whole bridge open, flyback rings
    delay_cyc(g_hardoff_cyc);            // rings; tune to land on the voltage peak
    if (phase_left) LOW_RIGHT_ON; else LOW_LEFT_ON;    // low side ON: ground the coil end
    g_lowside_off_tick = pulse_counter + g_hold_ticks; // schedule the long hold's end
#else
    if (phase_left)
    {
      // turn the whole bridge off
      LOW_RIGHT_OFF;
      LOW_LEFT_OFF;
      HIGH_LEFT_OFF;

      delay_cyc(FLYBACK_CYC);              // let the flyback ring up

      // ---- DAMPING: turn on the opposite low-side at current-max/voltage-min
      delay_cyc(g_damp_delay_cyc);        // <-- TUNE THIS (live, over serial)
      LOW_LEFT_ON;

      delay_cyc(g_damp_seg_cyc);
      LOW_RIGHT_ON;
      LOW_LEFT_OFF;

      delay_cyc(g_damp_seg_cyc);
      LOW_RIGHT_OFF;
    }
    else  // RIGHT / CW phase - exact mirror of the LEFT branch
    {
      LOW_RIGHT_OFF;
      LOW_LEFT_OFF;
      HIGH_RIGHT_OFF;

      delay_cyc(FLYBACK_CYC);
      delay_cyc(g_damp_delay_cyc);
      LOW_RIGHT_ON;                      // mirror of the LEFT branch

      delay_cyc(g_damp_seg_cyc);
      LOW_LEFT_ON;
      LOW_RIGHT_OFF;

      delay_cyc(g_damp_seg_cyc);
      LOW_LEFT_OFF;
    }

    LOW_LEFT_OFF;
    LOW_RIGHT_OFF;
#endif

    // ------------- SAMPLING: gate the decay onto the 4 hold caps -----------
    // Address is always changed while the MUX is disconnected, so a cap can
    // never see a glitch from a neighbouring channel during the transition.
    // Skipped entirely while loop() is reading the ADS1115 (doing_sampling):
    // EN stays high, the caps hold, the pulse itself was identical as ever.
    if (!doing_sampling)
    {
      delay_cyc(sample1_start);
      MUX_ADDR(0);  MUX_CONNECT;         // sample1: A0=0 A1=0 (S1)
      delay_cyc(sample1_width);
      MUX_DISCONNECT;

      delay_cyc(sample2_space);
      MUX_ADDR(1);  MUX_CONNECT;         // sample2: A0=1 A1=0 (S2)
      delay_cyc(sample2_width);
      MUX_DISCONNECT;

      delay_cyc(sample3_space);
      MUX_ADDR(2);  MUX_CONNECT;         // sample3: A0=0 A1=1 (S3)
      delay_cyc(sample3_width);
      MUX_DISCONNECT;                    // caps hold; cap4 (ref) gates pre-pulse
    }

    D2_LOW;                              // marker spans damping (+sampling if it ran)

    pulse_events++;

    sampleReady = true;                  // window has closed; consumer may act
    portEXIT_CRITICAL_ISR(&pulseMux);

    // >>> NO ADC HERE. analogRead() is NOT IRAM-safe. See notes at bottom. <<<
  }

  // ----------------------------------------------------- START NEXT PULSE
  if (pulse_counter == V_PULSE_PERIOD)
  {
    pulse_counter = 0;

#if SIMPLE_PULSE_NO_DAMP && !BIPOLAR
    phase_left = true;                   // unipolar bench mode: always LEFT
#else
    phase_left = !phase_left;            // alternate L/R diagonal every pulse
#endif

    portENTER_CRITICAL_ISR(&pulseMux);   // ref window + pulse start must not jitter

    LOW_RIGHT_OFF;
    LOW_LEFT_OFF;

    // ---- REF SAMPLE (cap4, A0=1 A1=1, S4): grab the settled frontend DC level
    // immediately before firing the pulse. The delay ALWAYS runs - even when
    // loop() is mid-ADS1115-read and the gating is skipped - so the pulse
    // start is a fixed offset from the tick and the ON time never changes.
    MUX_ADDR(3);
    if (!doing_sampling) MUX_CONNECT;
    delay_cyc(ref_sample_width);
    MUX_DISCONNECT;                      // harmless if it never connected

    if (phase_left)
    {
      HIGH_LEFT_ON;   // start LEFT (CCW) pulse
      LOW_RIGHT_ON;
    }
    else
    {
      HIGH_RIGHT_ON;  // start RIGHT (CW) pulse
      LOW_LEFT_ON;
    }

    portEXIT_CRITICAL_ISR(&pulseMux);
  }
}

// =============================================================== timer setup
bool setup_pulseTimer()
{
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  // ---- arduino-esp32 core 3.x ----
  pulseTimer = timerBegin(1000000);                        // 1 MHz -> 1 us ticks
  if (pulseTimer == NULL) return false;
  timerAttachInterrupt(pulseTimer, &timer_pulse_Interrupt);
  timerAlarm(pulseTimer, 25, true, 0);                     // 25 us alarm = 40 kHz
  return true;
#else
  // ---- arduino-esp32 core 2.x (you are here: 2.0.16) ----
  // prescaler 80 on the 80 MHz APB clock -> 1 us ticks
  pulseTimer = timerBegin(0, 80, true);
  if (pulseTimer == NULL) return false;
  timerAttachInterrupt(pulseTimer, &timer_pulse_Interrupt, true);  // edge-triggered
  timerAlarmWrite(pulseTimer, 25, true);                  // 25 us = 40 kHz, autoreload
  timerAlarmEnable(pulseTimer);
  return true;
#endif
}

// ===================================================================== setup
void setup()
{
  Serial.begin(115200);
  g_cpu_mhz = getCpuFrequencyMhz();      // should report 160

  // Restore the tuned hard-off (clamp start) time from NVS, if it was saved.
  // Kept open (RW) so the 's' key can write back in loop(). Clamp on load.
  g_prefs.begin(NVS_NAMESPACE, false);
  uint32_t saved_hardoff = g_prefs.getUInt(NVS_KEY_HARDOFF, g_hardoff_cyc);
  if (saved_hardoff <= HARDOFF_MAX_CYC) g_hardoff_cyc = saved_hardoff;
  Serial.printf("hardoff restored = %u cyc\n", g_hardoff_cyc);

  // Drive OUTPUT LATCHES to the SAFE (all-off) state BEFORE enabling outputs,
  // so we never glitch a leg into shoot-through at startup.
  HIGH_LEFT_OFF;  HIGH_RIGHT_OFF;  LOW_LEFT_OFF;  LOW_RIGHT_OFF;  D2_LOW;
  MUX_DISCONNECT;  MUX_ADDR(0);

  pinMode(PIN_HIGH_LEFT,  OUTPUT);
  pinMode(PIN_HIGH_RIGHT, OUTPUT);
  pinMode(PIN_LOW_LEFT,   OUTPUT);
  pinMode(PIN_LOW_RIGHT,  OUTPUT);
  pinMode(PIN_DEBUG,      OUTPUT);
  pinMode(PIN_MUX_A0,     OUTPUT);
  pinMode(PIN_MUX_A1,     OUTPUT);
  pinMode(PIN_MUX_EN,     OUTPUT);

  // re-assert safe levels (pinMode may have reset the latch)
  HIGH_LEFT_OFF;  HIGH_RIGHT_OFF;  LOW_LEFT_OFF;  LOW_RIGHT_OFF;  D2_LOW;
  MUX_DISCONNECT;  MUX_ADDR(0);

  // ADS1115 on the XIAO's default Wire pins (SDA=GPIO6, SCL=GPIO7).
  Wire.begin();
  Wire.setClock(400000);

  if (!setup_pulseTimer())
    Serial.println(F("timer init FAILED"));
  else
    Serial.println(F("pulse engine running. Scope D7 (debug) + coil."));

  Serial.println(F("Tune:  '+'/'-' hard-off (clamp start),  '['/']' hold length (x25us),  's' save hard-off,  'r' read caps"));
}

// ====================================================================== loop
void loop()
{
  // Live tuning of the damping timing - the whole point of the C3 switch.
  // + / -  step the damping delay by ONE cycle (one NOP, ~6.25 ns).
  // ] / [  step the segment width by one cycle.  Hold the key to sweep fast.
  static uint32_t last_read_pulse = 0;
  static bool     force_read      = false;

  while (Serial.available())
  {
    char c = Serial.read();
#if SIMPLE_PULSE_NO_DAMP
    // In simple mode: '+'/'-' tune the dead-off window (WHEN LOW_RIGHT fires);
    // '['/']' tune the LOW_RIGHT pulse width (HOW LONG it is on).
    // Both are busy-waits inside the interrupt-masked pulse-end section, so both
    // MUST stay small: hardoff + pulse + ~31 us sampling has to fit the ~50 us
    // (two-tick) ISR budget or counts are lost and the cadence stalls. Both capped.
    if      (c == '+' || c == '=') g_hardoff_cyc = (g_hardoff_cyc + 1 <= HARDOFF_MAX_CYC) ? g_hardoff_cyc + 1 : HARDOFF_MAX_CYC;
    else if (c == '-') g_hardoff_cyc = (g_hardoff_cyc >= 1) ? g_hardoff_cyc - 1 : 0;
    else if (c == ']') g_hold_ticks = (g_hold_ticks + 1 <= HOLD_TICKS_MAX) ? g_hold_ticks + 1 : HOLD_TICKS_MAX;
    else if (c == '[') g_hold_ticks = (g_hold_ticks >= 2) ? g_hold_ticks - 1 : 1;   // min 1 tick
    else if (c == 's') { g_prefs.putUInt(NVS_KEY_HARDOFF, g_hardoff_cyc);   // save to NVS
                         Serial.printf("saved hardoff = %u cyc\n", g_hardoff_cyc); continue; }
    else if (c == 'r') { force_read = true; continue; }
    else continue;
#else
    if      (c == '+') g_damp_delay_cyc += 1;
    else if (c == '-') g_damp_delay_cyc  = (g_damp_delay_cyc >= 1) ? g_damp_delay_cyc - 1 : 0;
    else if (c == ']') g_damp_seg_cyc   += 1;
    else if (c == '[') g_damp_seg_cyc    = (g_damp_seg_cyc >= 1) ? g_damp_seg_cyc - 1 : 0;
    else if (c == 'r') { force_read = true; continue; }
    else continue;
    Serial.printf("damp_delay = %u cyc (~%u ns)   damp_seg = %u cyc (~%u ns)\n",
                  g_damp_delay_cyc, (g_damp_delay_cyc * 1000) / g_cpu_mhz,
                  g_damp_seg_cyc,   (g_damp_seg_cyc   * 1000) / g_cpu_mhz);
#endif
  }

  // ------- ADS1115 read burst, every ADS_READ_EVERY pulses (or on 'r') -------
  // doing_sampling parks the MUX: the ISR keeps pulsing + damping unchanged
  // (consistent coil / supply-cap loading) but skips cap refresh, so the held
  // voltages sit still for the ~5 ms the four I2C conversions take.
  uint32_t pe = pulse_events;
  if (force_read || (pe - last_read_pulse >= ADS_READ_EVERY))
  {
    force_read      = false;
    last_read_pulse = pe;

    doing_sampling = true;
    bool ok = ads1115_read_all();        // refresh g_cap_raw[] (used by detection math)
    doing_sampling = false;              // cap refresh resumes on the next pulse

    // Serial Plotter format: "label:value" pairs, tab-separated, numbers only.
    // c1..c3 = raw decay-window ADC codes; ref = raw pre-pulse baseline (cap4).
    // d1..d3 = decay AMPLITUDE = ref - decay (positive: the decay swings BELOW the
    // baseline through the inverting amp). Raw ADS LSB, 125 uV each. cap1..4 -> AIN0..AIN3.
    if (ok)
      Serial.printf("c1_c3:%d\tc2_c3:%d\n",
                    g_cap_raw[0] - g_cap_raw[2],
                    g_cap_raw[1] - g_cap_raw[2]);
  }

  if (sampleReady) sampleReady = false;  // (kept for future per-pulse work)
}

/* ---------------------------------------------------------------------------
   Why ADC sampling isn't in the ISR: analogRead() / IDF oneshot are ~tens of us
   and not IRAM-safe, and C3 ADC2 is broken (errata). Instead the TMUX1108 gates
   the decay onto 4 hold caps at fixed post-pulse windows (done in the ISR - pure
   register writes), and loop() reads the held voltages over I2C via the ADS1115.
   Slow reads are fine: the caps capture the fast decay, the ADC just reads DC.
--------------------------------------------------------------------------- */
