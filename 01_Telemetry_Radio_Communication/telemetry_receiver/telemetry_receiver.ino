/*
  ===================================================
     ARDUINO MEGA + TELEMETRY TEXT RECEIVER
  ===================================================

  Board   : Arduino Mega 2560  #2  (RECEIVER)
  Project : Telemetry radio text-communication test

  WIRING  (the UART lines must CROSS: TX -> RX, RX <- TX)
    Telemetry TX   -> Arduino RX1  (pin 19)
    Telemetry RX   <- Arduino TX1  (pin 18)
    Telemetry GND  -> Arduino GND
    Telemetry 5V   -> Arduino 5V   (only if your radio is a 5 V module)

  SERIAL PORTS
    Serial   (USB)           -> computer / Serial Monitor, 115200 baud
    Serial1  (pins 18 / 19)  -> telemetry radio, TELEMETRY_BAUD

  HOW IT WORKS
    1. Wait for text arriving from telemetry radio #2 (Serial1).
    2. Read the complete line (up to the newline character).
    3. Display the message in the Serial Monitor.
    4. Send "ACK: <message>" back to Arduino #1, which proves that
       communication works in BOTH directions.
*/


// ---------------------------------------------------
// CONFIGURATION
// ---------------------------------------------------

// Must match the serial speed configured inside the telemetry radio.
// Use the SAME value as in the sender sketch.
#define TELEMETRY_BAUD 57600

// Speed of the USB link between the Arduino and the computer.
#define USB_BAUD 115200


// ---------------------------------------------------
// VARIABLES
// ---------------------------------------------------

String receivedMessage = "";

unsigned long messageCount = 0;


// ---------------------------------------------------
// SETUP
// ---------------------------------------------------

void setup()
{
  // Communication with the computer
  Serial.begin(USB_BAUD);

  // Communication with the telemetry radio
  Serial1.begin(TELEMETRY_BAUD);

  delay(1000);

  Serial.println();
  Serial.println(F("================================================"));
  Serial.println(F("        TELEMETRY RECEIVER READY"));
  Serial.println(F("================================================"));
  Serial.println();
  Serial.print(F("USB baud       : "));
  Serial.println(USB_BAUD);
  Serial.print(F("Telemetry baud : "));
  Serial.println(TELEMETRY_BAUD);
  Serial.println();
  Serial.println(F("Waiting for telemetry messages..."));
  Serial.println();
}


// ---------------------------------------------------
// LOOP
// ---------------------------------------------------

void loop()
{
  // Check for incoming telemetry data
  if (Serial1.available())
  {
    // Read the complete line sent by the sender
    receivedMessage = Serial1.readStringUntil('\n');

    // Remove the '\r' character and spaces at the ends
    receivedMessage.trim();

    if (receivedMessage.length() > 0)
    {
      messageCount++;

      Serial.println();
      Serial.println(F("------------------------------------------------"));
      Serial.println(F("TELEMETRY CONNECTED / DATA RECEIVED"));
      Serial.print(F("MESSAGE: "));
      Serial.println(receivedMessage);
      Serial.print(F("Messages received: "));
      Serial.println(messageCount);
      Serial.println(F("------------------------------------------------"));

      // Send the acknowledgement back to the sender.
      // An incoming "ACK: ..." is never acknowledged again; this avoids
      // an endless "ACK: ACK: ACK: ..." loop if both boards were
      // accidentally programmed with a receiver sketch.
      if (!receivedMessage.startsWith("ACK:"))
      {
        Serial1.print(F("ACK: "));
        Serial1.println(receivedMessage);
      }
    }
  }
}
