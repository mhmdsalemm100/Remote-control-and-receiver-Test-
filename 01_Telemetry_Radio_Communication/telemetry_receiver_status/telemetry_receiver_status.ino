/*
  =====================================================
     TELEMETRY RECEIVER WITH CONNECTION STATUS
  =====================================================

  Board   : Arduino Mega 2560  #2  (RECEIVER)
  Project : Telemetry radio text-communication test

  Same job as telemetry_receiver.ino, plus a simple link status:

    STATUS: CONNECTED / DATA RECEIVED
        printed every time a message arrives.

    STATUS: NO RECENT TELEMETRY DATA
        printed ONCE when no message has arrived for
        CONNECTION_TIMEOUT milliseconds.

    STATUS: DATA RESUMED
        printed when messages arrive again after a timeout.

  NOTE
    Telemetry radios do not give the Arduino a "connected" signal.
    "Connected" here means "a message arrived recently". The sender only
    transmits when you type something, so the timeout message also
    appears if nobody types for CONNECTION_TIMEOUT milliseconds.

  WIRING  (the UART lines must CROSS: TX -> RX, RX <- TX)
    Telemetry TX   -> Arduino RX1  (pin 19)
    Telemetry RX   <- Arduino TX1  (pin 18)
    Telemetry GND  -> Arduino GND
    Telemetry 5V   -> Arduino 5V   (only if your radio is a 5 V module)

  Serial Monitor : 115200 baud
*/


// ---------------------------------------------------
// CONFIGURATION
// ---------------------------------------------------

#define TELEMETRY_BAUD 57600
#define USB_BAUD 115200

// Time without data after which "NO RECENT TELEMETRY DATA" is printed
const unsigned long CONNECTION_TIMEOUT = 5000;


// ---------------------------------------------------
// VARIABLES
// ---------------------------------------------------

String receivedMessage = "";

unsigned long lastMessageTime = 0;
unsigned long messageCount = 0;

// true after the first message has been received
bool dataReceivedOnce = false;

// ONE flag, shared by the "timeout" and the "data received" code.
// It makes sure the timeout message is printed only once per outage
// and is printed again for every new outage.
bool timeoutReported = false;


// ---------------------------------------------------
// SETUP
// ---------------------------------------------------

void setup()
{
  Serial.begin(USB_BAUD);

  Serial1.begin(TELEMETRY_BAUD);

  delay(1000);

  Serial.println();
  Serial.println(F("============================================"));
  Serial.println(F("   TELEMETRY RECEIVER STARTED"));
  Serial.println(F("============================================"));
  Serial.println();
  Serial.print(F("Telemetry baud     : "));
  Serial.println(TELEMETRY_BAUD);
  Serial.print(F("No-data timeout    : "));
  Serial.print(CONNECTION_TIMEOUT);
  Serial.println(F(" ms"));
  Serial.println();
  Serial.println(F("Waiting for telemetry messages..."));
}


// ---------------------------------------------------
// LOOP
// ---------------------------------------------------

void loop()
{
  // ------------------------------------
  // Receive message
  // ------------------------------------

  if (Serial1.available())
  {
    receivedMessage = Serial1.readStringUntil('\n');

    receivedMessage.trim();

    if (receivedMessage.length() > 0)
    {
      // Data is back after a reported timeout
      if (timeoutReported)
      {
        Serial.println();
        Serial.print(F("STATUS: DATA RESUMED after "));
        Serial.print((millis() - lastMessageTime) / 1000.0, 1);
        Serial.println(F(" s without data"));
      }

      lastMessageTime = millis();
      dataReceivedOnce = true;
      timeoutReported = false;
      messageCount++;

      Serial.println();
      Serial.println(F("============================================"));
      Serial.println(F("STATUS: CONNECTED / DATA RECEIVED"));
      Serial.print(F("MESSAGE: "));
      Serial.println(receivedMessage);
      Serial.print(F("Messages received: "));
      Serial.println(messageCount);
      Serial.println(F("============================================"));

      // Send acknowledgement (never acknowledge an ACK)
      if (!receivedMessage.startsWith("ACK:"))
      {
        Serial1.print(F("ACK: "));
        Serial1.println(receivedMessage);
      }
    }
  }


  // ------------------------------------
  // Connection timeout
  // ------------------------------------

  if (dataReceivedOnce &&
      !timeoutReported &&
      (millis() - lastMessageTime > CONNECTION_TIMEOUT))
  {
    Serial.println();
    Serial.println(F("STATUS: NO RECENT TELEMETRY DATA"));
    Serial.print(F("        (no message for more than "));
    Serial.print(CONNECTION_TIMEOUT / 1000.0, 1);
    Serial.println(F(" s)"));

    timeoutReported = true;
  }
}
