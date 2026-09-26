/*
 ================================================================
     FLYSKY FS-i6X + RECEIVER AUTOMATED HEALTH TEST
                  Arduino Mega 2560
 ================================================================

 WIRING
   Receiver CH1 signal -> Arduino Mega pin 2   (ROLL / aileron)
   Receiver CH2 signal -> Arduino Mega pin 3   (PITCH / elevator)
   Receiver CH3 signal -> Arduino Mega pin 4   (THROTTLE)
   Receiver CH4 signal -> Arduino Mega pin 5   (YAW / rudder)
   Receiver GND        -> Arduino GND          (common ground REQUIRED)
   Receiver +5V        -> Arduino 5V           (only if the receiver is
                                                NOT already powered by a
                                                BEC / flight controller)

 SERIAL MONITOR
   115200 baud, line ending = "Newline"

 IMPORTANT
   REMOVE ALL PROPELLERS before testing.
   The test switches the transmitter OFF on purpose to observe the
   failsafe behavior of the receiver.

 WHAT IS MEASURED
   The PWM pulses produced by the receiver (typically 1000-2000 us).
   This is a FUNCTIONAL diagnostic of the complete chain:
     sticks -> transmitter -> 2.4 GHz RF -> receiver -> PWM outputs
   It cannot measure RF power, RSSI, signal-to-noise ratio or packet
   error rate. The score is a diagnostic score created by this program;
   it is NOT an official FlySky certification.

 TEST SEQUENCE
   TEST 1   Initial receiver connection
   TEST 2   Neutral (center) positions
   TEST 3   Roll travel      (left / right)
   TEST 4   Pitch travel     (down / up)
   TEST 5   Throttle travel  (minimum / maximum)
   TEST 6   Yaw travel       (left / right)
   TEST 7   Stability / jitter
   TEST 8   Distance / live-link test
   TEST 9   Failsafe (transmitter OFF)   - reported, not scored
   TEST 10  Link recovery (transmitter ON again)
   FINAL    Health report + diagnostic score (0-100)
 ================================================================
*/

#include <math.h>


// ================================================================
// CONFIGURATION
// ================================================================

const byte NUM_CHANNELS = 4;

const byte channelPins[NUM_CHANNELS] =
{
  2, 3, 4, 5
};

const char* const channelNames[NUM_CHANNELS] =
{
  "ROLL / CH1",
  "PITCH / CH2",
  "THROTTLE / CH3",
  "YAW / CH4"
};

// Position of every control in the arrays above
const byte CH_ROLL     = 0;
const byte CH_PITCH    = 1;
const byte CH_THROTTLE = 2;
const byte CH_YAW      = 3;


// Reasonable RC PWM limits: a pulse outside this window is not a
// valid RC command (or there is no pulse at all).
const unsigned int PWM_VALID_MIN = 900;    // us
const unsigned int PWM_VALID_MAX = 2100;   // us


// Expected window for a centered stick
const unsigned int CENTER_MIN = 1400;      // us
const unsigned int CENTER_MAX = 1600;      // us


// pulseIn() timeout.
// A 50 Hz receiver outputs one pulse every 20 ms. In the worst case
// pulseIn() has to wait for the current pulse to end, then for the next
// pulse to start, and then measure it: about 22 ms. 30 ms gives margin.
const unsigned long PWM_TIMEOUT = 30000;   // us


// Measurement durations
const unsigned long CONNECTION_TEST_MS = 3000;
const unsigned long NEUTRAL_TEST_MS    = 3000;
const unsigned long POSITION_TEST_MS   = 2000;
const unsigned long STABILITY_TEST_MS  = 8000;
const unsigned long FAILSAFE_REF_MS    = 2000;
const unsigned long FAILSAFE_TEST_MS   = 5000;
const unsigned long RECOVERY_TEST_MS   = 4000;


// Distance / live-link test.
// The 20 s measurement is divided into 2 s windows. A window counts as
// LIVE only if the ROLL output moved by at least LIVE_WINDOW_MIN_SPAN.
// This detects a link that is lost in the MIDDLE of the test even if
// the receiver keeps sending (failsafe / hold) PWM pulses.
const byte          RANGE_COUNTDOWN_S    = 15;
const unsigned long RANGE_TEST_MS        = 20000;
const unsigned long RANGE_WINDOW_MS      = 2000;
const unsigned int  LIVE_WINDOW_MIN_SPAN = 300;    // us


// Diagnostic limits of this test rig (NOT FlySky factory limits)
const float        MIN_CHANNEL_AVAILABILITY = 90.0;  // %
const float        MAX_HOLD_SD              = 20.0;  // us, stick held still
const unsigned int SAME_VALUE_TOLERANCE     = 30;    // us, "did not change"
const float        THROTTLE_LOW_FRACTION    = 0.05;  // <= 5 % of travel
const unsigned int RECOVERY_MIN_SPAN        = 400;   // us of ROLL movement


// ================================================================
// STATISTICS STRUCTURE
// ================================================================

struct ChannelStats
{
  unsigned long validSamples;
  unsigned long invalidSamples;

  unsigned int minPWM;
  unsigned int maxPWM;

  float average;              // running mean (Welford)
  float m2;                   // running sum of squared deviations (Welford)

  float standardDeviation;
};


// ================================================================
// FAILSAFE RESULT (throttle channel)
// ================================================================

const byte FAILSAFE_NOT_TESTED      = 0;
const byte FAILSAFE_NO_PULSES       = 1;  // throttle PWM output stopped
const byte FAILSAFE_THROTTLE_LOW    = 2;  // throttle went to minimum
const byte FAILSAFE_THROTTLE_NOT_LOW = 3; // DANGEROUS: throttle stays up
const byte FAILSAFE_UNKNOWN         = 4;  // could not be evaluated


// ================================================================
// TEST RESULTS
// ================================================================

char unitId[21] = "";

float neutralPWM[NUM_CHANNELS];

// Stick end-point values in us (NAN = no valid signal was measured)
float rollLeft;
float rollRight;

float pitchDown;
float pitchUp;

float throttleMin;
float throttleMax;

float yawLeft;
float yawRight;


float stabilitySD[NUM_CHANNELS];
float averageJitterSD = 0;
bool  stabilityValid = false;

float initialSignalQuality = 0;

float rangeSignalQuality = 0;

unsigned int rangeMinimum = 0;
unsigned int rangeMaximum = 0;

byte rangeLiveWindows = 0;
byte rangeTotalWindows = 0;

bool initialConnectionPassed = false;
bool recoveryPassed = false;

byte  failsafeResult = FAILSAFE_NOT_TESTED;
float failsafeThrottlePWM = NAN;


// Scores

int connectionScore = 0;
int neutralScore = 0;

int rollScore = 0;
int pitchScore = 0;
int throttleScore = 0;
int yawScore = 0;

int stabilityScore = 0;
int rangeScore = 0;


// ================================================================
// RESET ALL RESULTS (a new unit is tested)
// ================================================================

void resetResults()
{
  unitId[0] = '\0';

  for (byte i = 0; i < NUM_CHANNELS; i++)
  {
    neutralPWM[i] = NAN;
    stabilitySD[i] = 0;
  }

  rollLeft = rollRight = NAN;
  pitchDown = pitchUp = NAN;
  throttleMin = throttleMax = NAN;
  yawLeft = yawRight = NAN;

  averageJitterSD = 0;
  stabilityValid = false;

  initialSignalQuality = 0;
  rangeSignalQuality = 0;
  rangeMinimum = 0;
  rangeMaximum = 0;
  rangeLiveWindows = 0;
  rangeTotalWindows = 0;

  initialConnectionPassed = false;
  recoveryPassed = false;

  failsafeResult = FAILSAFE_NOT_TESTED;
  failsafeThrottlePWM = NAN;

  connectionScore = 0;
  neutralScore = 0;
  rollScore = 0;
  pitchScore = 0;
  throttleScore = 0;
  yawScore = 0;
  stabilityScore = 0;
  rangeScore = 0;
}


// ================================================================
// READ ONE CHANNEL
// ================================================================

// Returns the length of one HIGH pulse in microseconds,
// or 0 if no pulse arrived before PWM_TIMEOUT.
unsigned long readPWM(byte pin)
{
  return pulseIn(pin, HIGH, PWM_TIMEOUT);
}


// ================================================================
// CHECK PWM VALIDITY
// ================================================================

bool validPWM(unsigned long value)
{
  return value >= PWM_VALID_MIN &&
         value <= PWM_VALID_MAX;
}


// ================================================================
// SERIAL MONITOR HELPERS
// ================================================================

void flushSerialInput()
{
  while (Serial.available())
  {
    Serial.read();
  }
}


void waitForEnter()
{
  flushSerialInput();

  Serial.println();
  Serial.println(F(">>> Press ENTER when ready."));

  while (!Serial.available())
  {
    delay(10);
  }

  flushSerialInput();

  delay(300);
}


// Reads one line typed by the operator (without the line ending).
void readOperatorLine(char* buffer, size_t bufferSize)
{
  flushSerialInput();

  while (!Serial.available())
  {
    delay(10);
  }

  size_t length = Serial.readBytesUntil('\n', buffer, bufferSize - 1);
  buffer[length] = '\0';

  // Remove a trailing carriage return / spaces
  while (length > 0 &&
         (buffer[length - 1] == '\r' || buffer[length - 1] == ' '))
  {
    buffer[--length] = '\0';
  }

  // Commas would break the CSV result line
  for (size_t i = 0; i < length; i++)
  {
    if (buffer[i] == ',')
      buffer[i] = ';';
  }

  delay(50);
  flushSerialInput();
}


void printHeader(const __FlashStringHelper* title)
{
  Serial.println();
  Serial.println(F("================================================"));
  Serial.println(title);
  Serial.println(F("================================================"));
}


// Prints text and fills with spaces up to 'width' characters
void printPadded(const char* text, byte width)
{
  Serial.print(text);

  for (size_t n = strlen(text); n < width; n++)
    Serial.print(' ');
}


// Prints "1502.3 us" or "NO SIGNAL"
void printMicros(float value)
{
  if (isnan(value))
  {
    Serial.print(F("NO SIGNAL"));
  }
  else
  {
    Serial.print(value, 1);
    Serial.print(F(" us"));
  }
}


// |a - b|, or NAN when one of the values was not measured
float travelBetween(float a, float b)
{
  if (isnan(a) || isnan(b))
    return NAN;

  return fabs(a - b);
}


// ================================================================
// STATISTICS
// ================================================================

void resetStats(ChannelStats &s)
{
  s.validSamples = 0;
  s.invalidSamples = 0;

  s.minPWM = 65535;
  s.maxPWM = 0;

  s.average = 0;
  s.m2 = 0;

  s.standardDeviation = 0;
}


// Adds one reading to the statistics of a channel.
//
// The mean and the standard deviation are updated with Welford's
// online algorithm. On the Arduino Mega "double" is only a 32-bit
// float (about 7 significant digits). The textbook formula
//     variance = sum(x^2)/n - mean^2
// subtracts two almost equal numbers of about 2 250 000 and can lose
// 1-3 us of accuracy on long measurements. Welford's method only adds
// small deviations from the running mean and stays accurate.
void addSample(ChannelStats &s, unsigned long pwm)
{
  if (!validPWM(pwm))
  {
    s.invalidSamples++;
    return;
  }

  s.validSamples++;

  if (pwm < s.minPWM)
    s.minPWM = pwm;

  if (pwm > s.maxPWM)
    s.maxPWM = pwm;

  float x = (float)pwm;
  float delta = x - s.average;

  s.average += delta / s.validSamples;
  s.m2 += delta * (x - s.average);
}


void finishStats(ChannelStats &s)
{
  if (s.validSamples > 0)
    s.standardDeviation = sqrt(s.m2 / s.validSamples);
  else
    s.standardDeviation = 0;
}


// Percentage of readings of ONE channel that were valid pulses
float channelAvailability(const ChannelStats &s)
{
  unsigned long total = s.validSamples + s.invalidSamples;

  if (total == 0)
    return 0;

  return 100.0 * (float)s.validSamples / (float)total;
}


// Average of a channel, or NAN if it had no valid pulse
float averageOrNan(const ChannelStats &s)
{
  if (s.validSamples == 0)
    return NAN;

  return s.average;
}


// ================================================================
// COLLECT DATA
// ================================================================

// Reads every channel once. The readings are added to the statistics
// and copied into readings[] (0 = no valid pulse).
void readAllChannels(ChannelStats stats[], unsigned int readings[])
{
  for (byte i = 0; i < NUM_CHANNELS; i++)
  {
    unsigned long pwm = readPWM(channelPins[i]);

    addSample(stats[i], pwm);

    readings[i] = validPWM(pwm) ? (unsigned int)pwm : 0;
  }
}


void collectData(unsigned long duration,
                 ChannelStats stats[])
{
  unsigned int readings[NUM_CHANNELS];

  for (byte i = 0; i < NUM_CHANNELS; i++)
  {
    resetStats(stats[i]);
  }


  unsigned long startTime = millis();

  while (millis() - startTime < duration)
  {
    readAllChannels(stats, readings);
  }


  for (byte i = 0; i < NUM_CHANNELS; i++)
  {
    finishStats(stats[i]);
  }
}


// ================================================================
// SIGNAL QUALITY (all channels together)
// ================================================================

float calculateSignalQuality(ChannelStats stats[])
{
  unsigned long valid = 0;
  unsigned long total = 0;


  for (byte i = 0; i < NUM_CHANNELS; i++)
  {
    valid += stats[i].validSamples;

    total += stats[i].validSamples +
             stats[i].invalidSamples;
  }


  if (total == 0)
    return 0;


  return 100.0 *
         ((float)valid / (float)total);
}


// ================================================================
// PRINT STATISTICS
// ================================================================

void printStats(ChannelStats stats[])
{
  Serial.println();

  for (byte i = 0; i < NUM_CHANNELS; i++)
  {
    printPadded(channelNames[i], 15);

    Serial.print(F("| AVG = "));

    if (stats[i].validSamples > 0)
    {
      Serial.print(stats[i].average, 1);
      Serial.print(F(" us"));
    }
    else
    {
      Serial.print(F("NO SIGNAL"));
    }


    Serial.print(F(" | MIN = "));

    if (stats[i].validSamples > 0)
      Serial.print(stats[i].minPWM);
    else
      Serial.print('-');


    Serial.print(F(" | MAX = "));

    if (stats[i].validSamples > 0)
      Serial.print(stats[i].maxPWM);
    else
      Serial.print('-');


    Serial.print(F(" | SD = "));
    Serial.print(stats[i].standardDeviation, 2);
    Serial.print(F(" us"));


    Serial.print(F(" | Valid = "));
    Serial.print(stats[i].validSamples);


    Serial.print(F(" | Lost = "));
    Serial.println(stats[i].invalidSamples);
  }
}


// ================================================================
// CAPTURE ONE STICK POSITION
// ================================================================

// Returns the average PWM of 'channel', or NAN if it had no signal.
float capturePosition(const __FlashStringHelper* title,
                      const __FlashStringHelper* instruction,
                      byte channel)
{
  printHeader(title);

  Serial.println(instruction);

  waitForEnter();


  Serial.println();
  Serial.println(F("Measuring... keep holding the stick."));

  ChannelStats stats[NUM_CHANNELS];

  collectData(POSITION_TEST_MS, stats);

  printStats(stats);


  const ChannelStats &s = stats[channel];

  Serial.println();

  if (s.validSamples == 0)
  {
    Serial.println(F("RESULT: NO VALID SIGNAL on this channel."));
    Serial.println(F("        The position was NOT recorded."));

    return NAN;
  }


  Serial.print(F("Recorded value = "));
  Serial.print(s.average, 1);
  Serial.println(F(" us"));


  if (channelAvailability(s) < MIN_CHANNEL_AVAILABILITY)
  {
    Serial.print(F("WARNING: pulses were missing in "));
    Serial.print(100.0 - channelAvailability(s), 1);
    Serial.println(F("% of the readings."));
  }

  if (s.standardDeviation > MAX_HOLD_SD)
  {
    Serial.println(F("WARNING: the value moved during the measurement."));
    Serial.println(F("         Hold the stick steady at the end point."));
  }


  return s.average;
}


// ================================================================
// TRAVEL SCORE
// ================================================================

int calculateTravelScore(float a,
                         float b)
{
  // A missing measurement can never give points
  if (isnan(a) || isnan(b))
    return 0;


  float travel = fabs(a - b);


  if (travel >= 900)
    return 10;

  if (travel >= 800)
    return 9;

  if (travel >= 700)
    return 7;

  if (travel >= 600)
    return 5;

  if (travel >= 400)
    return 2;


  return 0;
}


void reportTravel(const __FlashStringHelper* name,
                  float a,
                  float b,
                  int score)
{
  Serial.println();
  Serial.println(F("------------------------------------------------"));
  Serial.print(name);
  Serial.print(F(" TRAVEL = "));

  float travel = travelBetween(a, b);

  if (isnan(travel))
  {
    Serial.println(F("NOT MEASURED (no signal)"));
  }
  else
  {
    Serial.print(travel, 1);
    Serial.println(F(" us"));

    if (travel < 400)
    {
      Serial.println(F("WARNING: very small travel. Was the correct stick"));
      Serial.println(F("         moved to BOTH end points?"));
    }
  }

  Serial.print(F("Score = "));
  Serial.print(score);
  Serial.println(F(" / 10"));
  Serial.println(F("------------------------------------------------"));
}


// ================================================================
// TEST 0 - UNIT IDENTIFICATION
// ================================================================

void askUnitId()
{
  printHeader(F("NEW TEST"));

  Serial.println();
  Serial.println(F("Type a unit ID / serial number (e.g. TX-001)"));
  Serial.println(F("and press ENTER, or just press ENTER to skip."));
  Serial.println();
  Serial.println(F(">>> Pressing ENTER starts the test."));

  readOperatorLine(unitId, sizeof(unitId));

  Serial.print(F("Unit ID: "));

  if (unitId[0] != '\0')
    Serial.println(unitId);
  else
    Serial.println(F("(not entered)"));
}


// ================================================================
// TEST 1 - INITIAL CONNECTION
// ================================================================

void initialConnectionTest()
{
  printHeader(F("TEST 1 - INITIAL RECEIVER CONNECTION"));

  Serial.println();
  Serial.println(F("Turn ON the transmitter."));
  Serial.println(F("Make sure the receiver is powered and bound"));
  Serial.println(F("(receiver LED normally lights steadily when linked)."));

  waitForEnter();


  ChannelStats stats[NUM_CHANNELS];

  Serial.println();
  Serial.println(F("Measuring receiver outputs..."));

  collectData(CONNECTION_TEST_MS, stats);

  printStats(stats);


  initialSignalQuality =
    calculateSignalQuality(stats);


  // Name every channel that produced no valid pulse at all
  for (byte i = 0; i < NUM_CHANNELS; i++)
  {
    if (stats[i].validSamples == 0)
    {
      Serial.println();
      Serial.print(F("NO SIGNAL on "));
      Serial.print(channelNames[i]);
      Serial.print(F(" (Arduino pin "));
      Serial.print(channelPins[i]);
      Serial.println(F(")."));
      Serial.println(F("Check the signal wire, the common GND and that the"));
      Serial.println(F("receiver output mode is PWM (not PPM)."));
    }
  }


  Serial.println();

  Serial.print(F("PWM availability = "));

  Serial.print(initialSignalQuality, 1);

  Serial.println(F("%"));


  if (initialSignalQuality >= 98.0)
  {
    initialConnectionPassed = true;
    connectionScore = 10;

    Serial.println(F("RESULT: PASS"));
  }

  else if (initialSignalQuality >= 90.0)
  {
    initialConnectionPassed = true;
    connectionScore = 7;

    Serial.println(F("RESULT: PASS WITH WARNING"));
  }

  else
  {
    initialConnectionPassed = false;
    connectionScore = 0;

    Serial.println(F("RESULT: FAIL / UNSTABLE SIGNAL"));
    Serial.println(F("The remaining tests will run, but fix the"));
    Serial.println(F("connection first for meaningful results."));
  }
}


// ================================================================
// TEST 2 - NEUTRAL POSITIONS
// ================================================================

bool isCentered(float value)
{
  return !isnan(value) &&
         value >= CENTER_MIN &&
         value <= CENTER_MAX;
}


void neutralTest()
{
  printHeader(F("TEST 2 - NORMAL / NEUTRAL POSITION"));

  Serial.println();

  Serial.println(F("Set:"));

  Serial.println(F("ROLL     = CENTER"));
  Serial.println(F("PITCH    = CENTER"));
  Serial.println(F("YAW      = CENTER"));
  Serial.println(F("THROTTLE = MINIMUM"));

  waitForEnter();


  ChannelStats stats[NUM_CHANNELS];

  Serial.println();
  Serial.println(F("Measuring neutral positions..."));

  collectData(NEUTRAL_TEST_MS, stats);

  printStats(stats);


  for (byte i = 0; i < NUM_CHANNELS; i++)
  {
    neutralPWM[i] = averageOrNan(stats[i]);
  }


  // Roll, pitch and yaw must be centered: 5 points each
  const byte centeredChannels[3] = { CH_ROLL, CH_PITCH, CH_YAW };

  neutralScore = 0;

  Serial.println();

  for (byte k = 0; k < 3; k++)
  {
    byte ch = centeredChannels[k];

    printPadded(channelNames[ch], 15);
    Serial.print(F(" center = "));
    printMicros(neutralPWM[ch]);

    if (isCentered(neutralPWM[ch]))
    {
      neutralScore += 5;
      Serial.println(F("  -> OK"));
    }
    else
    {
      Serial.print(F("  -> OUTSIDE "));
      Serial.print(CENTER_MIN);
      Serial.print('-');
      Serial.print(CENTER_MAX);
      Serial.println(F(" us (check trims / sub-trims)"));
    }
  }


  // Throttle is shown for information only (not scored here)
  printPadded(channelNames[CH_THROTTLE], 15);
  Serial.print(F(" minimum = "));
  printMicros(neutralPWM[CH_THROTTLE]);
  Serial.println(F("  (information)"));


  Serial.println();

  Serial.print(F("Neutral-position score = "));

  Serial.print(neutralScore);

  Serial.println(F(" / 15"));
}


// ================================================================
// TESTS 3 TO 6 - END POINTS / TRAVEL
// ================================================================

void endpointTests()
{
  // Stick hints are for a Mode 2 transmitter (FS-i6X default):
  // right stick = roll + pitch, left stick = throttle + yaw.

  rollLeft =
    capturePosition(
      F("TEST 3A - ROLL LEFT"),
      F("Move ROLL fully LEFT and HOLD it.\n(Mode 2: RIGHT stick to the LEFT)"),
      CH_ROLL
    );


  rollRight =
    capturePosition(
      F("TEST 3B - ROLL RIGHT"),
      F("Move ROLL fully RIGHT and HOLD it.\n(Mode 2: RIGHT stick to the RIGHT)"),
      CH_ROLL
    );


  rollScore =
    calculateTravelScore(rollLeft, rollRight);

  reportTravel(F("ROLL"), rollLeft, rollRight, rollScore);


  pitchDown =
    capturePosition(
      F("TEST 4A - PITCH DOWN"),
      F("Move PITCH fully DOWN and HOLD it.\n(Mode 2: RIGHT stick DOWN, toward you)"),
      CH_PITCH
    );


  pitchUp =
    capturePosition(
      F("TEST 4B - PITCH UP"),
      F("Move PITCH fully UP and HOLD it.\n(Mode 2: RIGHT stick UP, away from you)"),
      CH_PITCH
    );


  pitchScore =
    calculateTravelScore(pitchDown, pitchUp);

  reportTravel(F("PITCH"), pitchDown, pitchUp, pitchScore);


  throttleMin =
    capturePosition(
      F("TEST 5A - THROTTLE MINIMUM"),
      F("Move THROTTLE completely DOWN / MINIMUM.\n(Mode 2: LEFT stick fully DOWN)"),
      CH_THROTTLE
    );


  throttleMax =
    capturePosition(
      F("TEST 5B - THROTTLE MAXIMUM"),
      F("Move THROTTLE completely UP / MAXIMUM.\n(Mode 2: LEFT stick fully UP)"),
      CH_THROTTLE
    );


  throttleScore =
    calculateTravelScore(throttleMin, throttleMax);

  reportTravel(F("THROTTLE"), throttleMin, throttleMax, throttleScore);


  yawLeft =
    capturePosition(
      F("TEST 6A - YAW LEFT"),
      F("Move YAW fully LEFT and HOLD it.\n(Mode 2: LEFT stick to the LEFT)\nKeep THROTTLE at MINIMUM."),
      CH_YAW
    );


  yawRight =
    capturePosition(
      F("TEST 6B - YAW RIGHT"),
      F("Move YAW fully RIGHT and HOLD it.\n(Mode 2: LEFT stick to the RIGHT)\nKeep THROTTLE at MINIMUM."),
      CH_YAW
    );


  yawScore =
    calculateTravelScore(yawLeft, yawRight);

  reportTravel(F("YAW"), yawLeft, yawRight, yawScore);
}


// ================================================================
// TEST 7 - STABILITY / JITTER
// ================================================================

void stabilityTest()
{
  printHeader(F("TEST 7 - PWM STABILITY / JITTER"));

  Serial.println();

  Serial.println(F("Set:"));
  Serial.println(F("ROLL     = CENTER"));
  Serial.println(F("PITCH    = CENTER"));
  Serial.println(F("YAW      = CENTER"));
  Serial.println(F("THROTTLE = MINIMUM"));

  Serial.println();

  Serial.println(F("DO NOT TOUCH the controls during this test."));

  waitForEnter();


  Serial.println();
  Serial.println(F("Collecting stationary signal for 8 seconds..."));


  ChannelStats stats[NUM_CHANNELS];

  collectData(STABILITY_TEST_MS, stats);

  printStats(stats);


  // Every channel must deliver enough pulses, otherwise a silent
  // channel (SD = 0) would look perfectly "stable".
  stabilityValid = true;

  float totalSD = 0;

  for (byte i = 0; i < NUM_CHANNELS; i++)
  {
    stabilitySD[i] =
      stats[i].standardDeviation;

    totalSD += stabilitySD[i];

    if (channelAvailability(stats[i]) < MIN_CHANNEL_AVAILABILITY)
    {
      stabilityValid = false;

      Serial.println();
      Serial.print(F("NOT ENOUGH VALID PULSES on "));
      Serial.println(channelNames[i]);
    }
  }


  averageJitterSD =
    totalSD / NUM_CHANNELS;


  Serial.println();

  Serial.print(F("Average PWM jitter SD = "));

  Serial.print(averageJitterSD, 2);

  Serial.println(F(" us"));


  /*
      These limits are diagnostic limits for this test rig,
      not FlySky factory specifications.
  */

  if (!stabilityValid)
  {
    stabilityScore = 0;

    Serial.println(F("Stability: NOT MEASURABLE (signal missing)"));
  }

  else if (averageJitterSD <= 8)
  {
    stabilityScore = 15;

    Serial.println(F("Stability: EXCELLENT"));
  }

  else if (averageJitterSD <= 16)
  {
    stabilityScore = 12;

    Serial.println(F("Stability: GOOD"));
  }

  else if (averageJitterSD <= 30)
  {
    stabilityScore = 7;

    Serial.println(F("Stability: ACCEPTABLE / CHECK TRIMS"));
  }

  else
  {
    stabilityScore = 0;

    Serial.println(F("Stability: POOR / INVESTIGATE"));
  }
}


// ================================================================
// TEST 8 - DISTANCE / LIVE-LINK TEST
// ================================================================

void rangeTest()
{
  printHeader(F("TEST 8 - DISTANCE / LIVE LINK TEST"));

  Serial.println();

  Serial.println(F("This is a FUNCTIONAL range-response test."));

  Serial.println();

  Serial.print(F("After pressing ENTER you have "));
  Serial.print(RANGE_COUNTDOWN_S);
  Serial.println(F(" seconds to carry the"));
  Serial.println(F("transmitter to the desired test distance."));

  Serial.println();

  Serial.print(F("The measurement then runs for "));
  Serial.print(RANGE_TEST_MS / 1000);
  Serial.println(F(" seconds."));
  Serial.println();
  Serial.println(F("As soon as you are in position, CONTINUOUSLY move"));
  Serial.println(F("the ROLL stick FULL LEFT <-> FULL RIGHT"));
  Serial.println(F("(about one full movement per second)."));
  Serial.println();
  Serial.print(F("Keep moving it until at least "));
  Serial.print(RANGE_COUNTDOWN_S + RANGE_TEST_MS / 1000 + 5);
  Serial.println(F(" seconds after"));
  Serial.println(F("pressing ENTER."));

  waitForEnter();


  Serial.println();

  for (int i = RANGE_COUNTDOWN_S; i > 0; i--)
  {
    Serial.print(F("Test begins in "));

    Serial.print(i);

    Serial.println(F(" s"));

    delay(1000);
  }


  Serial.println();
  Serial.println(F("*** START MOVING ROLL LEFT/RIGHT NOW ***"));
  Serial.println();


  ChannelStats stats[NUM_CHANNELS];
  unsigned int readings[NUM_CHANNELS];

  for (byte i = 0; i < NUM_CHANNELS; i++)
  {
    resetStats(stats[i]);
  }


  rangeTotalWindows = RANGE_TEST_MS / RANGE_WINDOW_MS;
  rangeLiveWindows = 0;


  for (byte w = 0; w < rangeTotalWindows; w++)
  {
    unsigned int windowMin = 65535;
    unsigned int windowMax = 0;

    unsigned long windowStart = millis();

    while (millis() - windowStart < RANGE_WINDOW_MS)
    {
      readAllChannels(stats, readings);

      unsigned int roll = readings[CH_ROLL];

      if (roll > 0)
      {
        if (roll < windowMin)
          windowMin = roll;

        if (roll > windowMax)
          windowMax = roll;
      }
    }


    unsigned int windowSpan = 0;

    if (windowMax > windowMin)
      windowSpan = windowMax - windowMin;


    bool live = windowSpan >= LIVE_WINDOW_MIN_SPAN;

    if (live)
      rangeLiveWindows++;


    Serial.print(F("Window "));
    Serial.print(w + 1);
    Serial.print('/');
    Serial.print(rangeTotalWindows);
    Serial.print(F(" : ROLL span = "));
    Serial.print(windowSpan);
    Serial.print(F(" us -> "));

    if (live)
      Serial.println(F("LIVE"));
    else
      Serial.println(F("NO LIVE MOVEMENT"));
  }


  for (byte i = 0; i < NUM_CHANNELS; i++)
  {
    finishStats(stats[i]);
  }


  Serial.println();
  Serial.println(F("*** RANGE TEST COMPLETE ***"));

  printStats(stats);


  rangeSignalQuality =
    calculateSignalQuality(stats);


  if (stats[CH_ROLL].validSamples > 0)
  {
    rangeMinimum = stats[CH_ROLL].minPWM;
    rangeMaximum = stats[CH_ROLL].maxPWM;
  }


  unsigned int rangeTravel = 0;

  if (rangeMaximum > rangeMinimum)
  {
    rangeTravel =
      rangeMaximum - rangeMinimum;
  }


  float livePercent =
    100.0 * rangeLiveWindows / rangeTotalWindows;


  Serial.println();

  Serial.print(F("PWM availability at distance = "));

  Serial.print(rangeSignalQuality, 1);

  Serial.println(F("%"));


  Serial.print(F("Observed ROLL movement span   = "));

  Serial.print(rangeTravel);

  Serial.println(F(" us"));


  Serial.print(F("Live-response windows         = "));

  Serial.print(rangeLiveWindows);

  Serial.print('/');

  Serial.print(rangeTotalWindows);

  Serial.print(F(" ("));

  Serial.print(livePercent, 0);

  Serial.println(F("%)"));


  if (rangeSignalQuality >= 98 &&
      rangeTravel >= 800 &&
      livePercent >= 90)
  {
    rangeScore = 20;

    Serial.println(F("RANGE RESPONSE: EXCELLENT"));
  }

  else if (rangeSignalQuality >= 95 &&
           rangeTravel >= 650 &&
           livePercent >= 80)
  {
    rangeScore = 16;

    Serial.println(F("RANGE RESPONSE: GOOD"));
  }

  else if (rangeSignalQuality >= 90 &&
           rangeTravel >= 400 &&
           livePercent >= 60)
  {
    rangeScore = 10;

    Serial.println(F("RANGE RESPONSE: MARGINAL"));
  }

  else
  {
    rangeScore = 0;

    Serial.println(F("RANGE RESPONSE: FAIL / INVESTIGATE"));
  }


  if (rangeSignalQuality >= 90 && livePercent < 60)
  {
    Serial.println();
    Serial.println(F("NOTE: PWM pulses were present but the ROLL output"));
    Serial.println(F("did not follow the stick in many windows. The"));
    Serial.println(F("receiver was probably sending FAILSAFE / HOLD values"));
    Serial.println(F("because the live RF link was lost."));
  }
}


// ================================================================
// TEST 9 - FAILSAFE (TRANSMITTER OFF)
// ================================================================

void failsafeTest()
{
  printHeader(F("TEST 9 - FAILSAFE / TRANSMITTER OFF TEST"));

  Serial.println();

  Serial.println(F("WARNING:"));
  Serial.println(F("Make sure NO PROPELLERS are installed."));

  Serial.println();

  Serial.println(F("STEP 1 - Put the THROTTLE at about HALF and keep it"));
  Serial.println(F("there. Keep the other sticks centered."));
  Serial.println(F("(With throttle at half we can see whether the receiver"));
  Serial.println(F("HOLDS the last value or moves to a FAILSAFE value.)"));

  waitForEnter();


  Serial.println();
  Serial.println(F("Measuring reference values (transmitter ON)..."));

  ChannelStats before[NUM_CHANNELS];

  collectData(FAILSAFE_REF_MS, before);

  printStats(before);


  Serial.println();
  Serial.println(F("STEP 2 - Turn the transmitter OFF now."));
  Serial.println(F("Do not touch the receiver."));

  waitForEnter();


  Serial.println();
  Serial.println(F("Observing receiver for 5 seconds..."));


  ChannelStats after[NUM_CHANNELS];

  collectData(FAILSAFE_TEST_MS, after);

  printStats(after);


  float quality =
    calculateSignalQuality(after);


  Serial.println();

  Serial.print(F("PWM still present after TX OFF = "));

  Serial.print(quality, 1);

  Serial.println(F("%"));


  // Behavior of every channel
  Serial.println();
  Serial.println(F("Observed behavior per channel:"));

  for (byte i = 0; i < NUM_CHANNELS; i++)
  {
    printPadded(channelNames[i], 15);
    Serial.print(F(": "));

    if (channelAvailability(after[i]) < 10)
    {
      Serial.println(F("NO PULSES (output stopped)"));
    }
    else if (before[i].validSamples > 0 &&
             fabs(after[i].average - before[i].average) <= SAME_VALUE_TOLERANCE)
    {
      // Hold-last-value, or a failsafe value equal to the last value
      Serial.print(F("UNCHANGED ("));
      Serial.print(after[i].average, 0);
      Serial.println(F(" us)"));
    }
    else
    {
      Serial.print(F("CHANGED TO "));
      Serial.print(after[i].average, 0);
      Serial.println(F(" us (failsafe value)"));
    }
  }


  if (quality < 10)
  {
    Serial.println();
    Serial.println(F("Observed failsafe behavior:"));
    Serial.println(F("PWM OUTPUT DISAPPEARED."));
  }

  else
  {
    Serial.println();
    Serial.println(F("Observed failsafe behavior:"));
    Serial.println(F("RECEIVER CONTINUED PWM OUTPUT."));
    Serial.println();
    Serial.println(F("This may indicate:"));
    Serial.println(F("- programmed failsafe values"));
    Serial.println(F("- hold-last-position behavior"));
    Serial.println(F("- another configured failsafe mode"));
  }


  // ------------------------------------------------------------
  // Throttle safety check
  // ------------------------------------------------------------

  Serial.println();
  Serial.print(F("THROTTLE on signal loss: "));

  const ChannelStats &thr = after[CH_THROTTLE];

  if (channelAvailability(thr) < 10)
  {
    failsafeResult = FAILSAFE_NO_PULSES;

    Serial.println(F("PULSES STOP"));
    Serial.println(F("A flight controller normally detects this as signal"));
    Serial.println(F("loss - verify that its RC failsafe is enabled."));
  }

  else if (isnan(throttleMin) || isnan(throttleMax) ||
           fabs(throttleMax - throttleMin) < 100)
  {
    failsafeResult = FAILSAFE_UNKNOWN;
    failsafeThrottlePWM = thr.average;

    Serial.println(F("CANNOT BE EVALUATED"));
    Serial.println(F("(throttle end points were not measured correctly)"));
  }

  else
  {
    failsafeThrottlePWM = thr.average;

    // 0.0 = throttle minimum, 1.0 = throttle maximum.
    // Works even if the throttle channel is reversed.
    float position =
      (thr.average - throttleMin) / (throttleMax - throttleMin);

    Serial.print(thr.average, 0);
    Serial.print(F(" us = "));
    Serial.print(lround(position * 100.0));
    Serial.println(F("% throttle"));

    if (position <= THROTTLE_LOW_FRACTION)
    {
      failsafeResult = FAILSAFE_THROTTLE_LOW;

      Serial.println(F("SAFE: throttle goes to MINIMUM when the link is lost."));
    }
    else
    {
      failsafeResult = FAILSAFE_THROTTLE_NOT_LOW;

      Serial.println(F("*** WARNING: throttle does NOT go to minimum when ***"));
      Serial.println(F("*** the link is lost. Motors could keep running.  ***"));
      Serial.println(F("Set the failsafe in the transmitter (throttle low)"));
      Serial.println(F("before using this system in an aircraft."));
    }


    // With the throttle still low before TX OFF, HOLD and a low
    // failsafe value look identical.
    float beforePosition =
      (before[CH_THROTTLE].average - throttleMin) / (throttleMax - throttleMin);

    if (before[CH_THROTTLE].validSamples == 0 ||
        beforePosition < 0.25)
    {
      Serial.println(F("NOTE: throttle was not raised in STEP 1, so HOLD and"));
      Serial.println(F("a low failsafe value cannot be told apart."));
    }
  }


  Serial.println();
  Serial.println(F("This test reports the observed behavior."));
  Serial.println(F("It is not included in the health score."));
}


// ================================================================
// TEST 10 - LINK RECOVERY
// ================================================================

void recoveryTest()
{
  printHeader(F("TEST 10 - LINK RECOVERY"));

  Serial.println();

  Serial.println(F("1. Move the THROTTLE back to MINIMUM"));
  Serial.println(F("   (the FS-i6X warns if throttle is high at power-on)."));
  Serial.println(F("2. Turn the transmitter back ON."));
  Serial.println(F("3. Wait for the receiver to reconnect."));
  Serial.println(F("4. Start moving the ROLL stick LEFT <-> RIGHT"));
  Serial.println(F("   continuously, press ENTER, and KEEP MOVING it"));
  Serial.println(F("   until the result is shown."));
  Serial.println();
  Serial.println(F("(Moving the stick proves that LIVE commands are back;"));
  Serial.println(F("failsafe values alone would also produce PWM pulses.)"));

  waitForEnter();


  Serial.println();
  Serial.println(F("Measuring... keep moving ROLL."));

  ChannelStats stats[NUM_CHANNELS];

  collectData(RECOVERY_TEST_MS, stats);


  float quality =
    calculateSignalQuality(stats);


  printStats(stats);


  unsigned int rollSpan = 0;

  if (stats[CH_ROLL].validSamples > 0 &&
      stats[CH_ROLL].maxPWM > stats[CH_ROLL].minPWM)
  {
    rollSpan = stats[CH_ROLL].maxPWM - stats[CH_ROLL].minPWM;
  }


  Serial.println();

  Serial.print(F("Recovered PWM availability = "));

  Serial.print(quality, 1);

  Serial.println(F("%"));


  Serial.print(F("ROLL movement span         = "));

  Serial.print(rollSpan);

  Serial.println(F(" us"));


  if (quality >= 95 && rollSpan >= RECOVERY_MIN_SPAN)
  {
    recoveryPassed = true;

    Serial.println(F("RECOVERY: PASS"));
  }

  else
  {
    recoveryPassed = false;

    Serial.println(F("RECOVERY: FAIL / CHECK LINK"));

    if (quality >= 95)
    {
      Serial.println(F("PWM is present but ROLL does not follow the stick:"));
      Serial.println(F("the receiver may still be in failsafe."));
    }
  }
}


// ================================================================
// FINAL REPORT
// ================================================================

// true if every scored category earned at least some points.
// A category with 0 points is a failed function (for example a stick
// without travel), so the unit cannot be rated EXCELLENT or GOOD.
bool allCategoriesPassed()
{
  return connectionScore > 0 &&
         neutralScore > 0 &&
         rollScore > 0 &&
         pitchScore > 0 &&
         throttleScore > 0 &&
         yawScore > 0 &&
         stabilityScore > 0 &&
         rangeScore > 0;
}


// 3 = EXCELLENT, 2 = GOOD, 1 = MARGINAL, 0 = FAIL
byte conditionLevel(int totalScore)
{
  bool noFailures = recoveryPassed && allCategoriesPassed();

  if (totalScore >= 90 && noFailures)
    return 3;

  if (totalScore >= 80 && noFailures)
    return 2;

  if (totalScore >= 65)
    return 1;

  return 0;
}


const __FlashStringHelper* systemCondition(int totalScore)
{
  switch (conditionLevel(totalScore))
  {
    case 3:  return F("EXCELLENT");
    case 2:  return F("GOOD");
    case 1:  return F("MARGINAL / INSPECTION REQUIRED");
    default: return F("FAIL / DO NOT USE FOR FLIGHT");
  }
}


const __FlashStringHelper* failsafeText()
{
  switch (failsafeResult)
  {
    case FAILSAFE_NO_PULSES:        return F("PULSES STOP");
    case FAILSAFE_THROTTLE_LOW:     return F("THROTTLE LOW (SAFE)");
    case FAILSAFE_THROTTLE_NOT_LOW: return F("THROTTLE NOT LOW (WARNING)");
    case FAILSAFE_UNKNOWN:          return F("NOT EVALUATED");
    default:                        return F("NOT TESTED");
  }
}


void printEndpointLine(const __FlashStringHelper* label, float value)
{
  Serial.print(label);
  printMicros(value);
  Serial.println();
}


// Prints a value for the CSV line ("" when it was not measured)
void printCsvValue(float value)
{
  Serial.print(',');

  if (!isnan(value))
    Serial.print(value, 1);
}


void finalReport()
{
  int totalScore =

      connectionScore
    + neutralScore
    + rollScore
    + pitchScore
    + throttleScore
    + yawScore
    + stabilityScore
    + rangeScore;


  // Maximum:
  //
  // Connection  = 10
  // Neutral     = 15
  // Roll        = 10
  // Pitch       = 10
  // Throttle    = 10
  // Yaw         = 10
  // Stability   = 15
  // Range       = 20
  //
  // TOTAL       = 100


  Serial.println();
  Serial.println();
  Serial.println(F("################################################"));
  Serial.println(F("#                                              #"));
  Serial.println(F("#         FINAL RC SYSTEM HEALTH REPORT        #"));
  Serial.println(F("#                                              #"));
  Serial.println(F("################################################"));

  Serial.println();

  Serial.print(F("Unit ID                  : "));

  if (unitId[0] != '\0')
    Serial.println(unitId);
  else
    Serial.println(F("(not entered)"));


  Serial.print(F("Initial PWM availability : "));

  Serial.print(initialSignalQuality, 1);

  Serial.print(F("%  ("));

  if (initialConnectionPassed)
    Serial.println(F("PASS)"));
  else
    Serial.println(F("FAIL)"));


  Serial.println();
  Serial.println(F("----------- MEASURED ENDPOINTS ----------------"));


  printEndpointLine(F("ROLL LEFT        : "), rollLeft);
  printEndpointLine(F("ROLL RIGHT       : "), rollRight);
  printEndpointLine(F("ROLL TRAVEL      : "), travelBetween(rollLeft, rollRight));

  Serial.println();

  printEndpointLine(F("PITCH DOWN       : "), pitchDown);
  printEndpointLine(F("PITCH UP         : "), pitchUp);
  printEndpointLine(F("PITCH TRAVEL     : "), travelBetween(pitchDown, pitchUp));

  Serial.println();

  printEndpointLine(F("THROTTLE MIN     : "), throttleMin);
  printEndpointLine(F("THROTTLE MAX     : "), throttleMax);
  printEndpointLine(F("THROTTLE TRAVEL  : "), travelBetween(throttleMin, throttleMax));

  Serial.println();

  printEndpointLine(F("YAW LEFT         : "), yawLeft);
  printEndpointLine(F("YAW RIGHT        : "), yawRight);
  printEndpointLine(F("YAW TRAVEL       : "), travelBetween(yawLeft, yawRight));


  Serial.println();
  Serial.println(F("--------------- STABILITY --------------------"));


  for (byte i = 0; i < NUM_CHANNELS; i++)
  {
    printPadded(channelNames[i], 15);

    Serial.print(F(" jitter SD = "));

    Serial.print(stabilitySD[i], 2);

    Serial.println(F(" us"));
  }

  if (!stabilityValid)
    Serial.println(F("(signal missing - stability not measurable)"));


  Serial.println();
  Serial.println(F("---------------- RANGE -----------------------"));


  Serial.print(F("Range PWM availability : "));

  Serial.print(rangeSignalQuality, 1);

  Serial.println(F("%"));


  Serial.print(F("Range ROLL span        : "));

  if (rangeMaximum > rangeMinimum)
    Serial.print(rangeMaximum - rangeMinimum);
  else
    Serial.print(0);

  Serial.println(F(" us"));


  Serial.print(F("Live-response windows  : "));
  Serial.print(rangeLiveWindows);
  Serial.print('/');
  Serial.println(rangeTotalWindows);


  Serial.println();
  Serial.println(F("------------ FAILSAFE / RECOVERY -------------"));


  Serial.print(F("Throttle on signal loss : "));
  Serial.println(failsafeText());

  Serial.print(F("Link recovery           : "));

  if (recoveryPassed)
    Serial.println(F("PASS"));
  else
    Serial.println(F("FAIL"));


  Serial.println();
  Serial.println(F("---------------- SCORES ----------------------"));


  Serial.print(F("Connection        : "));
  Serial.print(connectionScore);
  Serial.println(F(" / 10"));


  Serial.print(F("Neutral controls  : "));
  Serial.print(neutralScore);
  Serial.println(F(" / 15"));


  Serial.print(F("Roll travel       : "));
  Serial.print(rollScore);
  Serial.println(F(" / 10"));


  Serial.print(F("Pitch travel      : "));
  Serial.print(pitchScore);
  Serial.println(F(" / 10"));


  Serial.print(F("Throttle travel   : "));
  Serial.print(throttleScore);
  Serial.println(F(" / 10"));


  Serial.print(F("Yaw travel        : "));
  Serial.print(yawScore);
  Serial.println(F(" / 10"));


  Serial.print(F("Signal stability  : "));
  Serial.print(stabilityScore);
  Serial.println(F(" / 15"));


  Serial.print(F("Distance response : "));
  Serial.print(rangeScore);
  Serial.println(F(" / 20"));


  Serial.println();
  Serial.println(F("=============================================="));

  Serial.print(F("FINAL DIAGNOSTIC HEALTH SCORE = "));

  Serial.print(totalScore);

  Serial.println(F(" / 100"));

  Serial.println(F("=============================================="));


  Serial.println();


  if (!recoveryPassed)
  {
    Serial.println(F("WARNING:"));
    Serial.println(F("Radio link did not recover correctly."));
    Serial.println(F("Investigate before operating an aircraft."));
    Serial.println();
  }


  if (failsafeResult == FAILSAFE_THROTTLE_NOT_LOW)
  {
    Serial.println(F("WARNING:"));
    Serial.println(F("Throttle does not go to minimum on signal loss."));
    Serial.println(F("Configure the failsafe before flying."));
    Serial.println();
  }


  if (!allCategoriesPassed())
  {
    Serial.println(F("WARNING:"));
    Serial.println(F("At least one test category scored 0 points."));
    Serial.println(F("See SCORES above."));
    Serial.println();
  }


  Serial.println(F("SYSTEM CONDITION:"));
  Serial.print(F("*** "));
  Serial.print(systemCondition(totalScore));
  Serial.println(F(" ***"));

  Serial.println();


  byte level = conditionLevel(totalScore);

  if (level == 3)
  {
    Serial.println(F("All major functional tests passed."));
  }

  else if (level == 2)
  {
    Serial.println(F("System is functioning well,"));
    Serial.println(F("but review any lower-scoring category."));
  }

  else if (level == 1)
  {
    Serial.println(F("One or more measurements require attention."));
  }

  else
  {
    Serial.println(F("Investigate the transmitter, receiver,"));
    Serial.println(F("power system, wiring, antennas,"));
    Serial.println(F("binding and configuration."));
  }


  Serial.println();
  Serial.println(F("IMPORTANT:"));
  Serial.println(F("This is a functional diagnostic score."));
  Serial.println(F("It is NOT an official FlySky factory"));
  Serial.println(F("certification or RF compliance measurement."));


  // ------------------------------------------------------------
  // One machine-readable line: copy it into a spreadsheet to
  // compare many transmitter / receiver units.
  // ------------------------------------------------------------

  Serial.println();
  Serial.println(F("CSV_HEADER,unit_id,score,condition,connection,neutral,roll,pitch,throttle,yaw,stability,range,"
                   "initial_availability,roll_travel,pitch_travel,throttle_travel,yaw_travel,avg_jitter_sd,"
                   "range_availability,range_span,live_windows,failsafe_throttle,recovery"));

  Serial.print(F("CSV_RESULT,"));
  Serial.print(unitId);
  Serial.print(',');
  Serial.print(totalScore);
  Serial.print(',');
  Serial.print(systemCondition(totalScore));
  Serial.print(',');
  Serial.print(connectionScore);
  Serial.print(',');
  Serial.print(neutralScore);
  Serial.print(',');
  Serial.print(rollScore);
  Serial.print(',');
  Serial.print(pitchScore);
  Serial.print(',');
  Serial.print(throttleScore);
  Serial.print(',');
  Serial.print(yawScore);
  Serial.print(',');
  Serial.print(stabilityScore);
  Serial.print(',');
  Serial.print(rangeScore);
  printCsvValue(initialSignalQuality);
  printCsvValue(travelBetween(rollLeft, rollRight));
  printCsvValue(travelBetween(pitchDown, pitchUp));
  printCsvValue(travelBetween(throttleMin, throttleMax));
  printCsvValue(travelBetween(yawLeft, yawRight));
  printCsvValue(stabilityValid ? averageJitterSD : NAN);
  printCsvValue(rangeSignalQuality);
  Serial.print(',');
  Serial.print(rangeMaximum > rangeMinimum ? rangeMaximum - rangeMinimum : 0);
  Serial.print(',');
  Serial.print(rangeLiveWindows);
  Serial.print('/');
  Serial.print(rangeTotalWindows);
  Serial.print(',');
  Serial.print(failsafeText());
  Serial.print(',');
  Serial.println(recoveryPassed ? F("PASS") : F("FAIL"));


  Serial.println();
  Serial.println(F("################################################"));
  Serial.println(F("#                TEST COMPLETE                 #"));
  Serial.println(F("################################################"));
}


// ================================================================
// COMPLETE TEST SEQUENCE
// ================================================================

void runCompleteTest()
{
  resetResults();

  askUnitId();

  initialConnectionTest();

  neutralTest();

  endpointTests();

  stabilityTest();

  rangeTest();

  failsafeTest();

  recoveryTest();

  finalReport();
}


// ================================================================
// SETUP
// ================================================================

void setup()
{
  Serial.begin(115200);


  for (byte i = 0; i < NUM_CHANNELS; i++)
  {
    pinMode(channelPins[i], INPUT);
  }


  delay(1500);


  Serial.println();
  Serial.println(F("################################################"));
  Serial.println(F("#                                              #"));
  Serial.println(F("#     FLYSKY RC MANUFACTURING HEALTH TEST      #"));
  Serial.println(F("#                                              #"));
  Serial.println(F("################################################"));

  Serial.println();

  Serial.println(F("Arduino Mega 2560"));

  Serial.println();

  Serial.println(F("CH1 (ROLL)     -> Pin 2"));
  Serial.println(F("CH2 (PITCH)    -> Pin 3"));
  Serial.println(F("CH3 (THROTTLE) -> Pin 4"));
  Serial.println(F("CH4 (YAW)      -> Pin 5"));
  Serial.println(F("Receiver GND   -> GND"));

  Serial.println();

  Serial.println(F("IMPORTANT:"));
  Serial.println(F("REMOVE ALL PROPELLERS BEFORE TESTING."));

  Serial.println();

  Serial.println(F("Serial Monitor must use 115200 baud and NEWLINE."));
}


// ================================================================
// LOOP
// ================================================================

void loop()
{
  // One complete test per unit. When the report is finished the
  // program asks for the next unit, so many units can be tested
  // without pressing RESET.

  runCompleteTest();
}
