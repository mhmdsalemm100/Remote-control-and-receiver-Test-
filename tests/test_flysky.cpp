// ================================================================
//  HOST SIMULATION TEST - FLYSKY FS-i6X + RECEIVER HEALTH TEST
// ================================================================
//
//  The REAL sketch (flysky_health_test.ino) is compiled on the PC
//  together with the Arduino mock. Instead of real hardware:
//
//   * RcSystem   simulates the transmitter + receiver: stick positions,
//                end points, PWM noise, 50 Hz frames, link loss and
//                three failsafe modes (no pulses / hold / preset).
//
//   * Operator   is a "virtual operator". Every time the sketch waits
//                for ENTER it reads the text on the screen (the Serial
//                Monitor output) and does what a person would do: move
//                the requested stick, switch the transmitter off, ...
//
//  Each scenario runs in its own child process (fresh sketch state).
//
//  Build and run:        make -C tests        (or: make -C tests flysky)
//  Show full transcripts: tests/build/test_flysky -v
//  Only print results:    tests/build/test_flysky --report-only
// ================================================================

#include "Arduino.h"

#include <random>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>


MockSerial Serial;

#ifndef FLYSKY_SKETCH
#define FLYSKY_SKETCH "../02_FlySky_Receiver_Health_Test/flysky_health_test/flysky_health_test.ino"
#endif

#include FLYSKY_SKETCH


// ----------------------------------------------------------------
// Simulated transmitter + receiver
// ----------------------------------------------------------------

enum FailsafeMode
{
  FS_NO_PULSES,     // receiver stops the PWM output
  FS_HOLD,          // receiver keeps the last received values
  FS_PRESET         // receiver outputs programmed failsafe values
};

const int ROLL = 0, PITCH = 1, THROTTLE = 2, YAW = 3;

struct RcSystem
{
  // ---- configuration ----
  double low[4]    = { 1000, 1000, 1000, 1000 };   // PWM with stick at -1
  double center[4] = { 1500, 1500, 1500, 1500 };   // PWM with stick at  0
  double high[4]   = { 2000, 2000, 2000, 2000 };   // PWM with stick at +1

  double noiseSD = 3.0;             // PWM jitter (us)
  unsigned long frameUs = 20000;    // 50 Hz output

  FailsafeMode failsafe = FS_PRESET;
  double preset[4] = { 1500, 1500, 1000, 1500 };

  // ---- state ----
  bool txOn = true;
  double linkLostAtS = -1;          // >= 0: RF link lost from this time on
  bool channelDead[4] = { false, false, false, false };

  double stick[4] = { 0, 0, -1, 0 };
  bool rollMoving = false;          // operator moves ROLL left <-> right

  double lastOutput[4] = { 1500, 1500, 1000, 1500 };

  std::mt19937 rng { 12345 };


  static double nowS()
  {
    return sim::nowMicros / 1e6;
  }

  bool linkUp() const
  {
    if (!txOn)
      return false;

    return !(linkLostAtS >= 0 && nowS() >= linkLostAtS);
  }

  void neutral()
  {
    rollMoving = false;
    stick[ROLL] = 0;
    stick[PITCH] = 0;
    stick[THROTTLE] = -1;
    stick[YAW] = 0;
  }

  double commandedPWM(int ch) const
  {
    double position = stick[ch];

    // One full LEFT -> RIGHT -> LEFT movement per second
    if (ch == ROLL && rollMoving)
    {
      double phase = fmod(nowS(), 1.0);
      position = phase < 0.5 ? -1 + 4 * phase : 3 - 4 * phase;
    }

    if (position < 0)
      return center[ch] + position * (center[ch] - low[ch]);

    return center[ch] + position * (high[ch] - center[ch]);
  }

  unsigned long pulseIn(uint8_t pin, unsigned long timeout)
  {
    int ch = (int)pin - 2;

    if (ch < 0 || ch > 3 || channelDead[ch])
    {
      sim::nowMicros += timeout;
      return 0;
    }

    double value = 0;

    if (linkUp())
    {
      value = commandedPWM(ch);
      lastOutput[ch] = value;
    }
    else
    {
      switch (failsafe)
      {
        case FS_NO_PULSES:
          sim::nowMicros += timeout;
          return 0;

        case FS_HOLD:
          value = lastOutput[ch];
          break;

        case FS_PRESET:
          value = preset[ch];
          break;
      }
    }

    if (noiseSD > 0)
    {
      std::normal_distribution<double> noise(0.0, noiseSD);
      value += noise(rng);
    }

    // pulseIn() usually has to wait about one frame for the next pulse
    sim::nowMicros += frameUs;

    return (unsigned long)lround(value);
  }
};


// ----------------------------------------------------------------
// Virtual operator
// ----------------------------------------------------------------

struct StopSimulation
{
};

struct Operator
{
  RcSystem *rc = nullptr;

  std::string unitPrefix = "UNIT-";
  std::string unitIdOverride;       // typed instead of UNIT-0n if set
  int unitsToTest = 1;
  bool txComesBack = true;          // transmitter reconnects in TEST 10

  // Called after the normal reaction, to inject faults:
  // fault(screen text, rc, number of the unit being tested)
  std::function<void(const std::string &, RcSystem &, int)> fault;

  size_t screenStart = 0;
  int unitsStarted = 0;
  int inputs = 0;


  void act()
  {
    if (++inputs > 200)
      throw StopSimulation();       // safety net: sketch waits forever

    std::string screen = Serial.tx.substr(screenStart);
    screenStart = Serial.tx.size();

    auto has = [&](const char *text)
    {
      return screen.find(text) != std::string::npos;
    };

    std::string typed = "\n";

    if (has("unit ID"))
    {
      if (unitsStarted == unitsToTest)
        throw StopSimulation();

      unitsStarted++;

      typed = (unitIdOverride.empty() ? unitPrefix + "0" + std::to_string(unitsStarted)
                                      : unitIdOverride) + "\n";

      rc->txOn = true;
      rc->neutral();
    }
    else if (has("TEST 3A")) { rc->neutral(); rc->stick[ROLL] = -1; }
    else if (has("TEST 3B")) { rc->neutral(); rc->stick[ROLL] = +1; }
    else if (has("TEST 4A")) { rc->neutral(); rc->stick[PITCH] = -1; }
    else if (has("TEST 4B")) { rc->neutral(); rc->stick[PITCH] = +1; }
    else if (has("TEST 5A")) { rc->neutral(); rc->stick[THROTTLE] = -1; }
    else if (has("TEST 5B")) { rc->neutral(); rc->stick[THROTTLE] = +1; }
    else if (has("TEST 6A")) { rc->neutral(); rc->stick[YAW] = -1; }
    else if (has("TEST 6B")) { rc->neutral(); rc->stick[YAW] = +1; }
    else if (has("Turn the transmitter OFF"))
    {
      rc->rollMoving = false;
      rc->txOn = false;
    }
    else if (has("THROTTLE at about HALF"))
    {
      rc->neutral();
      rc->stick[THROTTLE] = 0;
    }
    else if (has("transmitter back ON"))
    {
      rc->neutral();
      rc->txOn = txComesBack;
      rc->rollMoving = has("moving the ROLL stick");
    }
    else if (has("TEST 8"))
    {
      rc->neutral();
      rc->rollMoving = true;
    }
    else if (has("TEST 1 - INITIAL"))
    {
      rc->txOn = true;
      rc->neutral();
    }
    else if (has("TEST 2 -") || has("TEST 7 -"))
    {
      rc->neutral();
    }

    if (fault)
      fault(screen, *rc, unitsStarted);

    for (char c : typed)
      Serial.rx.push_back(c);
  }
};


// ----------------------------------------------------------------
// Running a scenario and reading the report
// ----------------------------------------------------------------

static void runSketch(RcSystem &rc, Operator &op)
{
  op.rc = &rc;

  sim::pulseSource = [&](uint8_t pin, unsigned long timeout)
  {
    return rc.pulseIn(pin, timeout);
  };

  // The sketch polls Serial.available() with delay(10) while it waits
  // for the operator: that is the moment the operator reacts.
  sim::onDelay = [&](unsigned long ms)
  {
    if (ms == 10 && Serial.rx.empty())
      op.act();
  };

  try
  {
    setup();

    for (int i = 0; i < 10; i++)
    {
      uint64_t before = sim::nowMicros;
      loop();

      if (sim::nowMicros == before)   // loop() does nothing any more
        break;
    }
  }
  catch (StopSimulation &)
  {
  }
}


static bool verbose = false;
static bool reportOnly = false;
static int checksFailed = 0;

#define CHECK(condition)                                                    \
  do                                                                        \
  {                                                                         \
    if (!reportOnly && !(condition))                                        \
    {                                                                       \
      printf("      CHECK FAILED (line %d): %s\n", __LINE__, #condition);  \
      checksFailed++;                                                       \
    }                                                                       \
  } while (0)


static bool contains(const std::string &text, const std::string &needle)
{
  return text.find(needle) != std::string::npos;
}

static int countOf(const std::string &text, const std::string &needle)
{
  int count = 0;

  for (size_t pos = text.find(needle); pos != std::string::npos;
       pos = text.find(needle, pos + needle.size()))
  {
    count++;
  }

  return count;
}

// Integer written after the LAST occurrence of 'label' (-1 if missing)
static int numberAfter(const std::string &text, const std::string &label)
{
  size_t pos = text.rfind(label);

  if (pos == std::string::npos)
    return -1;

  return atoi(text.c_str() + pos + label.size());
}

static std::string lineAfter(const std::string &text, const std::string &label)
{
  size_t pos = text.rfind(label);

  if (pos == std::string::npos)
    return "";

  pos += label.size();
  return text.substr(pos, text.find("\r\n", pos) - pos);
}

static int totalScore(const std::string &out)
{
  return numberAfter(out, "FINAL DIAGNOSTIC HEALTH SCORE = ");
}

static std::string condition(const std::string &out)
{
  size_t pos = out.rfind("SYSTEM CONDITION:");

  if (pos == std::string::npos)
    return "";

  return lineAfter(out.substr(pos), "*** ");
}

static void summary(const std::string &out)
{
  printf("      score %d/100 | %s | range %d/20 | recovery: %s | failsafe: %s\n",
         totalScore(out),
         condition(out).c_str(),
         numberAfter(out, "Distance response : "),
         contains(out, "RECOVERY: PASS") ? "PASS" : "FAIL",
         lineAfter(out, "Throttle on signal loss : ").c_str());
}

static void showTranscript(const std::string &out)
{
  if (!verbose)
    return;

  size_t start = 0;

  while (start < out.size())
  {
    size_t end = out.find("\r\n", start);
    if (end == std::string::npos)
      end = out.size();

    printf("      | %s\n", out.substr(start, end - start).c_str());
    start = end + 2;
  }
}

static void finish(const std::string &out)
{
  summary(out);
  showTranscript(out);
}


// ----------------------------------------------------------------
// SCENARIOS
// ----------------------------------------------------------------

// 1. A healthy system with a correctly programmed failsafe.
static void scenario_healthy()
{
  RcSystem rc;
  Operator op;
  runSketch(rc, op);

  const std::string &out = Serial.tx;

  CHECK(totalScore(out) == 100);
  CHECK(condition(out) == "EXCELLENT ***");
  CHECK(contains(out, "Unit ID: UNIT-01"));
  CHECK(contains(out, "RESULT: PASS"));
  CHECK(numberAfter(out, "Neutral-position score = ") == 15);
  CHECK(numberAfter(out, "Signal stability  : ") == 15);
  CHECK(contains(out, "RANGE RESPONSE: EXCELLENT"));
  CHECK(contains(out, "Live-response windows  : 10/10"));
  CHECK(contains(out, "SAFE: throttle goes to MINIMUM"));
  CHECK(contains(out, "RECOVERY: PASS"));
  CHECK(contains(out, "CSV_RESULT,UNIT-01,100,EXCELLENT,10,15,10,10,10,10,15,20,"));
  CHECK(countOf(out, "WARNING") == 1);   // only the "no propellers" reminder
  CHECK(!contains(out, "nan"));

  // Measured end points close to 1000 / 2000 us
  CHECK(fabs(atof(lineAfter(out, "ROLL LEFT        : ").c_str()) - 1000) < 5);
  CHECK(fabs(atof(lineAfter(out, "ROLL RIGHT       : ").c_str()) - 2000) < 5);

  finish(out);
}


// 2. Failsafe = HOLD: throttle stays at half when the TX is switched off.
static void scenario_failsafe_hold()
{
  RcSystem rc;
  rc.failsafe = FS_HOLD;

  Operator op;
  runSketch(rc, op);

  const std::string &out = Serial.tx;

  CHECK(contains(out, "UNCHANGED ("));
  CHECK(contains(out, "*** WARNING: throttle does NOT go to minimum"));
  CHECK(contains(out, "Throttle on signal loss : THROTTLE NOT LOW (WARNING)"));
  CHECK(contains(out, "Throttle does not go to minimum on signal loss."));
  CHECK(totalScore(out) == 100);                   // failsafe is not scored
  CHECK(contains(out, "RECOVERY: PASS"));

  finish(out);
}


// 3. Failsafe = no pulses.
static void scenario_failsafe_no_pulses()
{
  RcSystem rc;
  rc.failsafe = FS_NO_PULSES;

  Operator op;
  runSketch(rc, op);

  const std::string &out = Serial.tx;

  CHECK(contains(out, "PWM OUTPUT DISAPPEARED."));
  CHECK(contains(out, "NO PULSES (output stopped)"));
  CHECK(contains(out, "Throttle on signal loss : PULSES STOP"));
  CHECK(contains(out, "RECOVERY: PASS"));
  CHECK(totalScore(out) == 100);

  finish(out);
}


// 4. Example from the documentation: pitch only 1205 - 1770 us.
static void scenario_reduced_pitch_travel()
{
  RcSystem rc;
  rc.low[PITCH] = 1205;
  rc.high[PITCH] = 1770;
  rc.center[PITCH] = 1487;

  Operator op;
  runSketch(rc, op);

  const std::string &out = Serial.tx;

  CHECK(numberAfter(out, "Pitch travel      : ") == 2);   // 565 us -> 2 points
  CHECK(totalScore(out) == 92);
  CHECK(fabs(atof(lineAfter(out, "PITCH TRAVEL     : ").c_str()) - 565) < 5);

  finish(out);
}


// 5. Loose wire: CH2 has no signal during "PITCH DOWN" only.
//    A missing measurement must not give travel points.
static void scenario_channel_lost_during_capture()
{
  RcSystem rc;

  Operator op;
  op.fault = [](const std::string &screen, RcSystem &r, int)
  {
    r.channelDead[PITCH] = screen.find("TEST 4A") != std::string::npos;
  };

  runSketch(rc, op);

  const std::string &out = Serial.tx;

  CHECK(contains(out, "RESULT: NO VALID SIGNAL on this channel."));
  CHECK(contains(out, "PITCH TRAVEL = NOT MEASURED"));
  CHECK(numberAfter(out, "Pitch travel      : ") == 0);
  CHECK(contains(out, "PITCH DOWN       : NO SIGNAL"));
  CHECK(totalScore(out) == 90);
  CHECK(condition(out) == "MARGINAL / INSPECTION REQUIRED ***");   // a category scored 0
  CHECK(contains(out, "At least one test category scored 0 points."));

  finish(out);
}


// 6. RF link lost 8 s into the range measurement; receiver holds the
//    last values, so PWM pulses are still present.
static void scenario_link_lost_during_range_test()
{
  RcSystem rc;
  rc.failsafe = FS_HOLD;

  Operator op;
  op.fault = [](const std::string &screen, RcSystem &r, int)
  {
    if (screen.find("TEST 8") != std::string::npos)
      r.linkLostAtS = RcSystem::nowS() + 0.3 + 15 + 8;   // ENTER delay + countdown + 8 s

    if (screen.find("THROTTLE at about HALF") != std::string::npos)
      r.linkLostAtS = -1;                                 // operator walks back
  };

  runSketch(rc, op);

  const std::string &out = Serial.tx;

  CHECK(contains(out, "NO LIVE MOVEMENT"));
  CHECK(contains(out, "RANGE RESPONSE: FAIL / INVESTIGATE"));
  CHECK(contains(out, "NOTE: PWM pulses were present"));
  CHECK(numberAfter(out, "Distance response : ") == 0);
  CHECK(totalScore(out) == 80);
  CHECK(condition(out) == "MARGINAL / INSPECTION REQUIRED ***");   // range scored 0

  finish(out);
}


// 7. Transmitter does NOT come back in TEST 10; the receiver keeps
//    sending its preset failsafe values (PWM present, no live control).
static void scenario_no_recovery()
{
  RcSystem rc;
  rc.failsafe = FS_PRESET;

  Operator op;
  op.txComesBack = false;

  runSketch(rc, op);

  const std::string &out = Serial.tx;

  CHECK(contains(out, "RECOVERY: FAIL / CHECK LINK"));
  CHECK(contains(out, "PWM is present but ROLL does not follow the stick"));
  CHECK(contains(out, "Radio link did not recover correctly."));
  CHECK(totalScore(out) == 100);
  CHECK(condition(out) == "MARGINAL / INSPECTION REQUIRED ***");

  finish(out);
}


// 8. CH4 (yaw) wire disconnected for the whole test.
static void scenario_dead_yaw_channel()
{
  RcSystem rc;
  rc.channelDead[YAW] = true;

  Operator op;
  runSketch(rc, op);

  const std::string &out = Serial.tx;

  CHECK(contains(out, "NO SIGNAL on YAW / CH4 (Arduino pin 5)."));
  CHECK(contains(out, "RESULT: FAIL / UNSTABLE SIGNAL"));
  CHECK(contains(out, "Stability: NOT MEASURABLE (signal missing)"));
  CHECK(numberAfter(out, "Yaw travel        : ") == 0);
  CHECK(numberAfter(out, "Signal stability  : ") == 0);
  CHECK(numberAfter(out, "Neutral-position score = ") == 10);
  CHECK(totalScore(out) == 40);
  CHECK(condition(out) == "FAIL / DO NOT USE FOR FLIGHT ***");

  finish(out);
}


// 9. Noisy receiver output (jitter SD about 12 us).
static void scenario_noisy_output()
{
  RcSystem rc;
  rc.noiseSD = 12;

  Operator op;
  runSketch(rc, op);

  const std::string &out = Serial.tx;

  CHECK(contains(out, "Stability: GOOD"));
  CHECK(numberAfter(out, "Signal stability  : ") == 12);
  CHECK(totalScore(out) == 97);

  finish(out);
}


// 10. Throttle channel reversed in the transmitter; failsafe throttle
//     programmed to the (reversed) minimum = 2000 us.
static void scenario_reversed_throttle()
{
  RcSystem rc;
  rc.low[THROTTLE] = 2000;
  rc.high[THROTTLE] = 1000;
  rc.preset[THROTTLE] = 2000;

  Operator op;
  runSketch(rc, op);

  const std::string &out = Serial.tx;

  CHECK(numberAfter(out, "Throttle travel   : ") == 10);
  CHECK(contains(out, "SAFE: throttle goes to MINIMUM"));
  CHECK(totalScore(out) == 100);

  finish(out);
}


// 11. Two units tested one after the other without RESET: all results
//     must be cleared between the units.
static void scenario_two_units()
{
  RcSystem rc;

  Operator op;
  op.unitsToTest = 2;
  op.fault = [](const std::string &screen, RcSystem &r, int unit)
  {
    // The second unit has a short pitch travel and a dead CH4
    if (unit == 2 && screen.find("unit ID") != std::string::npos)
    {
      r.low[PITCH] = 1205;
      r.high[PITCH] = 1770;
      r.center[PITCH] = 1487;
    }
  };

  runSketch(rc, op);

  const std::string &out = Serial.tx;

  CHECK(countOf(out, "FINAL RC SYSTEM HEALTH REPORT") == 2);
  CHECK(contains(out, "CSV_RESULT,UNIT-01,100,EXCELLENT,"));
  CHECK(contains(out, "CSV_RESULT,UNIT-02,92,EXCELLENT,"));
  CHECK(countOf(out, "unit ID") == 3);    // asks again for a third unit

  finish(out);
}


// 12. Unit ID typed with a comma and "Both NL & CR" line ending.
static void scenario_unit_id_input()
{
  RcSystem rc;

  Operator op;
  op.unitIdOverride = "TX,77\r";

  runSketch(rc, op);

  const std::string &out = Serial.tx;

  CHECK(contains(out, "Unit ID: TX;77\r\n"));
  CHECK(contains(out, "CSV_RESULT,TX;77,100,"));

  finish(out);
}


// ----------------------------------------------------------------
// Runner (one child process per scenario)
// ----------------------------------------------------------------

struct Scenario
{
  const char *name;
  void (*function)();
};

int main(int argc, char **argv)
{
  for (int i = 1; i < argc; i++)
  {
    std::string arg = argv[i];

    if (arg == "-v")
      verbose = true;

    if (arg == "--report-only")
      reportOnly = true;
  }

  const Scenario scenarios[] =
  {
    { "healthy system, failsafe = throttle low",       scenario_healthy },
    { "failsafe = HOLD (throttle stays up)",            scenario_failsafe_hold },
    { "failsafe = no pulses",                           scenario_failsafe_no_pulses },
    { "reduced pitch travel 1205-1770 us",              scenario_reduced_pitch_travel },
    { "CH2 lost during 'PITCH DOWN' only",              scenario_channel_lost_during_capture },
    { "RF link lost in the middle of the range test",   scenario_link_lost_during_range_test },
    { "transmitter does not reconnect (preset FS)",     scenario_no_recovery },
    { "CH4 wire disconnected for the whole test",       scenario_dead_yaw_channel },
    { "noisy receiver output (SD 12 us)",               scenario_noisy_output },
    { "reversed throttle channel",                      scenario_reversed_throttle },
    { "two units tested without RESET",                 scenario_two_units },
    { "unit ID with comma and CR",                      scenario_unit_id_input },
  };

  const int count = (int)(sizeof(scenarios) / sizeof(scenarios[0]));
  int failed = 0;

  printf("FlySky health test - host simulation scenarios\n");

  for (const Scenario &s : scenarios)
  {
    printf("  - %s\n", s.name);
    fflush(stdout);

    pid_t pid = fork();

    if (pid == 0)
    {
      s.function();
      fflush(stdout);
      _exit(checksFailed == 0 ? 0 : 1);
    }

    int status = 0;
    waitpid(pid, &status, 0);

    bool ok = WIFEXITED(status) && WEXITSTATUS(status) == 0;

    if (!reportOnly)
      printf("    [%s]\n", ok ? "PASS" : "FAIL");

    if (!ok)
      failed++;
  }

  if (!reportOnly)
    printf("%d of %d scenarios passed\n", count - failed, count);

  return failed == 0 ? 0 : 1;
}
