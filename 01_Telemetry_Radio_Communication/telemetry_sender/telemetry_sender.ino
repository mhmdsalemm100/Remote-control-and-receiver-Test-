/*
  ===================================================
     ARDUINO MEGA + TELEMETRY TEXT SENDER
  ===================================================

  Board   : Arduino Mega 2560  #1  (SENDER)
  Project : Telemetry radio text-communication test

  WIRING  (the UART lines must CROSS: TX -> RX, RX <- TX)
    Telemetry TX   -> Arduino RX1  (pin 19)
    Telemetry RX   <- Arduino TX1  (pin 18)
    Telemetry GND  -> Arduino GND
    Telemetry 5V   -> Arduino 5V   (only if your radio is a 5 V module)

  SERIAL PORTS
    Serial   (USB)           -> computer / Serial Monitor, 115200 baud
    Serial1  (pins 18 / 19)  -> telemetry radio, TELEMETRY_BAUD

  SERIAL MONITOR SETTINGS
    Baud rate   : 115200
    Line ending : Newline

  HOW IT WORKS
    1. Type any word or sentence in the Serial Monitor and press ENTER.
    2. The text is sent to telemetry radio #1 through Serial1.
    3. Arduino Mega #2 (receiver) answers with "ACK: <your text>".
    4. This sketch prints the reply, checks that it matches the text
       that was sent, and shows the round-trip time.
    5. If no reply arrives within ACK_TIMEOUT_MS a warning is printed,
       which means the RETURN path (receiver -> sender) has a problem.
*/


// ---------------------------------------------------
// CONFIGURATION
// ---------------------------------------------------

// Must match the serial speed configured inside the telemetry radio
// (many drone telemetry radios use 57600). Use the SAME value on both
// Arduino boards.
#define TELEMETRY_BAUD 57600

// Speed of the USB link between the Arduino and the computer.
#define USB_BAUD 115200

// How long to wait for the "ACK: ..." reply before printing a warning.
const unsigned long ACK_TIMEOUT_MS = 3000;


// ---------------------------------------------------
// VARIABLES
// ---------------------------------------------------

String message = "";           // text typed by the user
String lastSentMessage = "";   // last text sent, waiting for its ACK

unsigned long sendTime = 0;    // millis() when the last text was sent
bool waitingForAck = false;    // true while an ACK is expected

unsigned long messagesSent = 0;
unsigned long acksReceived = 0;


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
  Serial.println(F("        TELEMETRY SENDER READY"));
  Serial.println(F("================================================"));
  Serial.println();
  Serial.print(F("USB baud       : "));
  Serial.println(USB_BAUD);
  Serial.print(F("Telemetry baud : "));
  Serial.println(TELEMETRY_BAUD);
  Serial.println();
  Serial.println(F("Type any word or sentence."));
  Serial.println(F("Then press ENTER."));
  Serial.println();
}


// ---------------------------------------------------
// LOOP
// ---------------------------------------------------

void loop()
{
  // -----------------------------------------------
  // 1) Check if the user typed something
  // -----------------------------------------------

  if (Serial.available())
  {
    // Read the text until the ENTER / newline character
    message = Serial.readStringUntil('\n');

    // Remove spaces and the '\r' character at the ends
    message.trim();

    // Only send if the message is not empty
    if (message.length() > 0)
    {
      // Send through telemetry (println adds "\r\n" = end of message)
      Serial1.println(message);

      lastSentMessage = message;
      sendTime = millis();
      waitingForAck = true;
      messagesSent++;

      // Show locally
      Serial.print(F("[SENT] "));
      Serial.println(message);
    }
  }


  // -----------------------------------------------
  // 2) Check if the receiver sent anything back
  // -----------------------------------------------

  if (Serial1.available())
  {
    String reply = Serial1.readStringUntil('\n');

    reply.trim();

    if (reply.length() > 0)
    {
      Serial.print(F("[RECEIVER REPLY] "));
      Serial.println(reply);

      // The receiver answers with "ACK: " + the text it received.
      String expectedAck = String("ACK: ") + lastSentMessage;

      if (waitingForAck && reply == expectedAck)
      {
        waitingForAck = false;
        acksReceived++;

        Serial.print(F("[LINK OK] ACK matches the sent text. Round-trip time = "));
        Serial.print(millis() - sendTime);
        Serial.print(F(" ms  ("));
        Serial.print(acksReceived);
        Serial.print(F(" of "));
        Serial.print(messagesSent);
        Serial.println(F(" messages acknowledged)"));
      }
      else if (reply.startsWith("ACK:"))
      {
        Serial.println(F("[WARNING] This ACK does not match the last text sent."));
        Serial.println(F("          Possible data corruption (check baud rate),"));
        Serial.println(F("          or several messages were sent very quickly."));
      }
    }
  }


  // -----------------------------------------------
  // 3) No ACK received in time?
  // -----------------------------------------------

  if (waitingForAck && (millis() - sendTime > ACK_TIMEOUT_MS))
  {
    waitingForAck = false;

    Serial.print(F("[NO ACK] No reply received within "));
    Serial.print(ACK_TIMEOUT_MS);
    Serial.println(F(" ms."));
    Serial.println(F("         If Arduino #2 displayed the message, the forward path works"));
    Serial.println(F("         but the RETURN path does not. Check:"));
    Serial.println(F("         - Arduino #2 TX1 (pin 18) -> Telemetry #2 RX"));
    Serial.println(F("         - Telemetry #1 TX -> Arduino #1 RX1 (pin 19)"));
  }
}
