// ================================================================
//  Minimal PC (host) replacement of the Arduino API
// ================================================================
//
//  Only the functions used by the sketches of this repository are
//  provided. Time is SIMULATED: it only moves forward when the sketch
//  calls delay(), pulseIn() or a Serial read that has to wait. This
//  makes every test run in milliseconds and completely repeatable.
//
//  The mock is used by test_telemetry.cpp and test_flysky.cpp.
// ================================================================

#pragma once

#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
#include <math.h>
#include <string>


typedef uint8_t byte;

#define HIGH 0x1
#define LOW  0x0

#define INPUT        0x0
#define OUTPUT       0x1
#define INPUT_PULLUP 0x2

#define DEC 10


// ----------------------------------------------------------------
// F() macro: on the AVR it keeps text in flash memory. On the PC the
// text is simply used as a normal C string.
// ----------------------------------------------------------------

class __FlashStringHelper;

#define F(text) (reinterpret_cast<const __FlashStringHelper *>(text))


// ----------------------------------------------------------------
// Simulated time and hardware hooks
// ----------------------------------------------------------------

namespace sim
{
  inline uint64_t nowMicros = 0;

  // Called by pulseIn(). Must return the pulse length in us (0 = none)
  // and advance sim::nowMicros by the time the measurement takes.
  inline std::function<unsigned long(uint8_t pin, unsigned long timeout)> pulseSource;

  // Called after every delay(). The FlySky test uses it to let the
  // "virtual operator" type when the sketch is waiting for ENTER.
  inline std::function<void(unsigned long ms)> onDelay;

  inline void advanceMs(unsigned long ms)
  {
    nowMicros += (uint64_t)ms * 1000ULL;
  }
}


inline unsigned long millis()
{
  return (unsigned long)(sim::nowMicros / 1000ULL);
}

inline unsigned long micros()
{
  return (unsigned long)sim::nowMicros;
}

inline void delay(unsigned long ms)
{
  sim::advanceMs(ms);

  if (sim::onDelay)
    sim::onDelay(ms);
}

inline void pinMode(uint8_t, uint8_t)
{
}

inline unsigned long pulseIn(uint8_t pin, uint8_t, unsigned long timeout)
{
  if (sim::pulseSource)
    return sim::pulseSource(pin, timeout);

  sim::nowMicros += timeout;
  return 0;
}


// ----------------------------------------------------------------
// String (subset of the Arduino String class)
// ----------------------------------------------------------------

class String
{
public:
  String(const char *text = "") : s(text ? text : "") {}
  String(const std::string &text) : s(text) {}

  unsigned int length() const { return (unsigned int)s.size(); }
  const char *c_str() const { return s.c_str(); }

  void trim()
  {
    size_t begin = 0;
    while (begin < s.size() && isspace((unsigned char)s[begin]))
      begin++;

    size_t end = s.size();
    while (end > begin && isspace((unsigned char)s[end - 1]))
      end--;

    s = s.substr(begin, end - begin);
  }

  bool startsWith(const String &prefix) const
  {
    return s.size() >= prefix.s.size() &&
           s.compare(0, prefix.s.size(), prefix.s) == 0;
  }

  bool equals(const String &other) const { return s == other.s; }
  bool operator==(const String &other) const { return s == other.s; }
  bool operator!=(const String &other) const { return s != other.s; }

  friend String operator+(const String &a, const String &b)
  {
    return String(a.s + b.s);
  }

private:
  std::string s;
};


// ----------------------------------------------------------------
// Serial port: 'rx' holds bytes waiting to be read by the sketch,
// 'tx' collects everything the sketch printed / sent.
// ----------------------------------------------------------------

class MockSerial
{
public:
  std::deque<char> rx;
  std::string tx;

  long baud = 0;
  unsigned long timeoutMs = 1000;   // Arduino default Stream timeout


  void begin(long baudRate) { baud = baudRate; }
  void setTimeout(unsigned long ms) { timeoutMs = ms; }

  int available() { return (int)rx.size(); }

  int peek() { return rx.empty() ? -1 : (unsigned char)rx.front(); }

  int read()
  {
    if (rx.empty())
      return -1;

    char c = rx.front();
    rx.pop_front();
    return (unsigned char)c;
  }

  // Like Stream::timedRead(): if no byte is waiting, the real Arduino
  // waits 'timeoutMs' and gives up.
  int timedRead()
  {
    if (rx.empty())
    {
      sim::advanceMs(timeoutMs);
      return -1;
    }

    return read();
  }

  String readStringUntil(char terminator)
  {
    std::string out;

    int c = timedRead();

    while (c >= 0 && (char)c != terminator)
    {
      out += (char)c;
      c = timedRead();
    }

    return String(out);
  }

  size_t readBytesUntil(char terminator, char *buffer, size_t length)
  {
    size_t index = 0;

    while (index < length)
    {
      int c = timedRead();

      if (c < 0 || (char)c == terminator)
        break;

      buffer[index++] = (char)c;
    }

    return index;
  }


  // ---- output ----

  size_t write(char c)
  {
    tx += c;
    return 1;
  }

  size_t print(const char *text)
  {
    tx += text;
    return strlen(text);
  }

  size_t print(const __FlashStringHelper *text)
  {
    return print(reinterpret_cast<const char *>(text));
  }

  size_t print(const String &text) { return print(text.c_str()); }
  size_t print(char c) { return write(c); }

  size_t print(unsigned char value, int base = DEC) { return printNumber((unsigned long)value, base); }
  size_t print(int value, int base = DEC) { return printSigned(value, base); }
  size_t print(unsigned int value, int base = DEC) { return printNumber(value, base); }
  size_t print(long value, int base = DEC) { return printSigned(value, base); }
  size_t print(unsigned long value, int base = DEC) { return printNumber(value, base); }

  size_t print(double value, int digits = 2)
  {
    char buffer[64];

    if (std::isnan(value))
      snprintf(buffer, sizeof(buffer), "nan");
    else if (std::isinf(value))
      snprintf(buffer, sizeof(buffer), "inf");
    else
      snprintf(buffer, sizeof(buffer), "%.*f", digits, value);

    return print(buffer);
  }

  size_t println()
  {
    tx += "\r\n";
    return 2;
  }

  template <typename T>
  size_t println(T value)
  {
    size_t n = print(value);
    return n + println();
  }

  template <typename T>
  size_t println(T value, int format)
  {
    size_t n = print(value, format);
    return n + println();
  }

private:
  size_t printNumber(unsigned long value, int base)
  {
    char buffer[40];

    if (base == 16)
      snprintf(buffer, sizeof(buffer), "%lX", value);
    else
      snprintf(buffer, sizeof(buffer), "%lu", value);

    return print(buffer);
  }

  size_t printSigned(long value, int base)
  {
    if (base != DEC)
      return printNumber((unsigned long)value, base);

    char buffer[40];
    snprintf(buffer, sizeof(buffer), "%ld", value);
    return print(buffer);
  }
};
