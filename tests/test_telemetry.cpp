// ================================================================
//  HOST SIMULATION TEST - TELEMETRY TEXT COMMUNICATION
// ================================================================
//
//  The REAL sketch files are compiled on the PC together with the
//  Arduino mock (arduino_mock/Arduino.h). Two "virtual Arduino Mega
//  boards" are connected through a simulated telemetry-radio link:
//
//     sender  Serial1  ==== radio link (latency, faults) ====  Serial1  receiver
//
//  Every test runs in its own child process, so each one starts with
//  freshly initialised sketch variables (like pressing RESET).
//
//  Build and run:  make -C tests        (or: make -C tests telemetry)
//  Show the Serial Monitor transcripts:  tests/build/test_telemetry -v
// ================================================================

#include "Arduino.h"

#include <cstdlib>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>


#ifndef SENDER_SKETCH
#define SENDER_SKETCH "../01_Telemetry_Radio_Communication/telemetry_sender/telemetry_sender.ino"
#endif

#ifndef RECEIVER_SKETCH
#define RECEIVER_SKETCH "../01_Telemetry_Radio_Communication/telemetry_receiver/telemetry_receiver.ino"
#endif

#ifndef STATUS_SKETCH
#define STATUS_SKETCH "../01_Telemetry_Radio_Communication/telemetry_receiver_status/telemetry_receiver_status.ino"
#endif


// ----------------------------------------------------------------
// Each sketch is compiled inside its own namespace, with its own
// Serial (USB) and Serial1 (telemetry) ports.
// ----------------------------------------------------------------

namespace sender
{
  MockSerial Serial;
  MockSerial Serial1;
#include SENDER_SKETCH
}
#undef TELEMETRY_BAUD
#undef USB_BAUD

namespace receiver
{
  MockSerial Serial;
  MockSerial Serial1;
#include RECEIVER_SKETCH
}
#undef TELEMETRY_BAUD
#undef USB_BAUD

namespace status_rx
{
  MockSerial Serial;
  MockSerial Serial1;
#include STATUS_SKETCH
}
#undef TELEMETRY_BAUD
#undef USB_BAUD


// ----------------------------------------------------------------
// Virtual board and radio link
// ----------------------------------------------------------------

struct Board
{
  const char *name;
  MockSerial *usb;      // Serial  -> Serial Monitor
  MockSerial *radio;    // Serial1 -> telemetry radio
  void (*setup)();
  void (*loop)();
};

Board senderBoard   = { "SENDER (Arduino #1)",   &sender::Serial,    &sender::Serial1,    sender::setup,    sender::loop };
Board receiverBoard = { "RECEIVER (Arduino #2)", &receiver::Serial,  &receiver::Serial1,  receiver::setup,  receiver::loop };
Board statusBoard   = { "RECEIVER+STATUS (#2)",  &status_rx::Serial, &status_rx::Serial1, status_rx::setup, status_rx::loop };


struct RadioLink
{
  Board *a = nullptr;             // sender side
  Board *b = nullptr;             // receiver side

  bool forwardOk = true;          // a -> b
  bool returnOk = true;           // b -> a
  bool corruptReturn = false;     // change one character on the way back

  unsigned long latencyMs = 40;   // one-way latency of the radio link

  std::deque<std::pair<uint64_t, char>> aToB;
  std::deque<std::pair<uint64_t, char>> bToA;


  void start()
  {
    a->setup();
    b->setup();
  }

  void transfer(MockSerial *from, bool ok, bool corrupt, std::deque<std::pair<uint64_t, char>> &queue)
  {
    uint64_t deliverAt = sim::nowMicros + (uint64_t)latencyMs * 1000ULL;

    for (size_t i = 0; i < from->tx.size(); i++)
    {
      char c = from->tx[i];

      // Corrupt the first character after "ACK: " (index 5)
      if (corrupt && i == 5 && c != '\r' && c != '\n')
        c = 'X';

      if (ok)
        queue.push_back({ deliverAt, c });
    }

    from->tx.clear();
  }

  void deliver(std::deque<std::pair<uint64_t, char>> &queue, MockSerial *to)
  {
    while (!queue.empty() && queue.front().first <= sim::nowMicros)
    {
      to->rx.push_back(queue.front().second);
      queue.pop_front();
    }
  }

  void runFor(unsigned long ms)
  {
    uint64_t end = sim::nowMicros + (uint64_t)ms * 1000ULL;

    while (sim::nowMicros < end)
    {
      a->loop();
      b->loop();

      transfer(a->radio, forwardOk, false, aToB);
      transfer(b->radio, returnOk, corruptReturn, bToA);

      deliver(aToB, b->radio);
      deliver(bToA, a->radio);

      sim::advanceMs(1);
    }
  }
};


// ----------------------------------------------------------------
// Small test framework
// ----------------------------------------------------------------

static bool verbose = false;
static int checksFailed = 0;

static void type(Board &board, const std::string &text)
{
  for (char c : text)
    board.usb->rx.push_back(c);
}

static size_t countOf(const std::string &haystack, const std::string &needle)
{
  size_t count = 0;

  for (size_t pos = haystack.find(needle); pos != std::string::npos;
       pos = haystack.find(needle, pos + needle.size()))
  {
    count++;
  }

  return count;
}

static bool contains(const std::string &haystack, const std::string &needle)
{
  return haystack.find(needle) != std::string::npos;
}

#define CHECK(condition)                                                    \
  do                                                                        \
  {                                                                         \
    if (!(condition))                                                       \
    {                                                                       \
      printf("      CHECK FAILED (line %d): %s\n", __LINE__, #condition);  \
      checksFailed++;                                                       \
    }                                                                       \
  } while (0)

static void showTranscript(const Board &board)
{
  if (!verbose)
    return;

  printf("\n      ---- %s Serial Monitor ----\n", board.name);

  std::string text = board.usb->tx;
  size_t start = 0;

  while (start < text.size())
  {
    size_t end = text.find("\r\n", start);
    if (end == std::string::npos)
      end = text.size();

    printf("      | %s\n", text.substr(start, end - start).c_str());
    start = end + 2;
  }
}


// ----------------------------------------------------------------
// TESTS
// ----------------------------------------------------------------

// The basic experiment of the README: "Hello drone" goes out, ACK comes back.
static void test_basic_round_trip()
{
  RadioLink link;
  link.a = &senderBoard;
  link.b = &receiverBoard;
  link.start();

  type(senderBoard, "Hello drone\n");
  link.runFor(500);

  const std::string &s = senderBoard.usb->tx;
  const std::string &r = receiverBoard.usb->tx;

  CHECK(contains(s, "TELEMETRY SENDER READY"));
  CHECK(contains(r, "TELEMETRY RECEIVER READY"));
  CHECK(contains(s, "[SENT] Hello drone\r\n"));
  CHECK(contains(r, "TELEMETRY CONNECTED / DATA RECEIVED"));
  CHECK(contains(r, "MESSAGE: Hello drone\r\n"));
  CHECK(contains(s, "[RECEIVER REPLY] ACK: Hello drone\r\n"));
  CHECK(contains(s, "[LINK OK]"));
  CHECK(!contains(s, "[NO ACK]"));

  // Round trip = 2 x 40 ms latency (+ a few ms of loop time)
  size_t pos = s.find("Round-trip time = ");
  CHECK(pos != std::string::npos);

  if (pos != std::string::npos)
  {
    int rtt = atoi(s.c_str() + pos + strlen("Round-trip time = "));
    CHECK(rtt >= 80 && rtt <= 90);
  }

  showTranscript(senderBoard);
  showTranscript(receiverBoard);
}


// All example messages from the project description.
static void test_several_messages()
{
  RadioLink link;
  link.a = &senderBoard;
  link.b = &receiverBoard;
  link.start();

  const char *messages[] =
  {
    "Hello", "Drone test", "This is telemetry test number 1",
    "Roll = 5 degrees", "Battery = 12.4 V", "ROLL=5.4,PITCH=-2.1,YAW=43.8"
  };

  for (const char *m : messages)
  {
    type(senderBoard, std::string(m) + "\n");
    link.runFor(1000);

    CHECK(contains(receiverBoard.usb->tx, std::string("MESSAGE: ") + m + "\r\n"));
    CHECK(contains(senderBoard.usb->tx, std::string("[RECEIVER REPLY] ACK: ") + m + "\r\n"));
  }

  CHECK(countOf(senderBoard.usb->tx, "[LINK OK]") == 6);
  CHECK(contains(senderBoard.usb->tx, "(6 of 6 messages acknowledged)"));
  CHECK(contains(receiverBoard.usb->tx, "Messages received: 6"));

  showTranscript(senderBoard);
}


// Serial Monitor set to "Both NL & CR": the '\r' must not reach the message.
static void test_crlf_line_ending()
{
  RadioLink link;
  link.a = &senderBoard;
  link.b = &receiverBoard;
  link.start();

  type(senderBoard, "Test 123\r\n");
  link.runFor(500);

  CHECK(contains(senderBoard.usb->tx, "[SENT] Test 123\r\n"));
  CHECK(contains(receiverBoard.usb->tx, "MESSAGE: Test 123\r\n"));
  CHECK(contains(senderBoard.usb->tx, "[LINK OK]"));
}


// Serial Monitor set to "No line ending": still works after the 1 s timeout.
static void test_no_line_ending()
{
  RadioLink link;
  link.a = &senderBoard;
  link.b = &receiverBoard;
  link.start();

  type(senderBoard, "Hello");
  link.runFor(2000);

  CHECK(contains(senderBoard.usb->tx, "[SENT] Hello\r\n"));
  CHECK(contains(receiverBoard.usb->tx, "MESSAGE: Hello\r\n"));
}


// Empty lines (only ENTER or spaces) are not transmitted.
static void test_empty_line_not_sent()
{
  RadioLink link;
  link.a = &senderBoard;
  link.b = &receiverBoard;
  link.start();

  type(senderBoard, "\n");
  link.runFor(300);
  type(senderBoard, "    \n");
  link.runFor(4000);

  CHECK(!contains(senderBoard.usb->tx, "[SENT]"));
  CHECK(!contains(senderBoard.usb->tx, "[NO ACK]"));
  CHECK(!contains(receiverBoard.usb->tx, "MESSAGE:"));
}


// Receiver -> sender direction broken: receiver shows the message,
// sender reports [NO ACK] after 3 s.
static void test_return_path_broken()
{
  RadioLink link;
  link.a = &senderBoard;
  link.b = &receiverBoard;
  link.returnOk = false;
  link.start();

  type(senderBoard, "Hello\n");
  link.runFor(2000);

  CHECK(contains(receiverBoard.usb->tx, "MESSAGE: Hello\r\n"));
  CHECK(!contains(senderBoard.usb->tx, "[NO ACK]"));   // not yet (3 s timeout)

  link.runFor(2000);

  CHECK(contains(senderBoard.usb->tx, "[NO ACK]"));
  CHECK(contains(senderBoard.usb->tx, "RETURN path"));
  CHECK(!contains(senderBoard.usb->tx, "[LINK OK]"));

  showTranscript(senderBoard);
}


// Sender -> receiver direction broken: nothing arrives at all.
static void test_forward_path_broken()
{
  RadioLink link;
  link.a = &senderBoard;
  link.b = &receiverBoard;
  link.forwardOk = false;
  link.start();

  type(senderBoard, "Hello\n");
  link.runFor(4000);

  CHECK(contains(senderBoard.usb->tx, "[SENT] Hello"));
  CHECK(!contains(receiverBoard.usb->tx, "MESSAGE:"));
  CHECK(contains(senderBoard.usb->tx, "[NO ACK]"));
}


// A corrupted character on the way back is detected.
static void test_corrupted_ack_detected()
{
  RadioLink link;
  link.a = &senderBoard;
  link.b = &receiverBoard;
  link.corruptReturn = true;
  link.start();

  type(senderBoard, "Hello\n");
  link.runFor(500);

  CHECK(contains(senderBoard.usb->tx, "[RECEIVER REPLY] ACK: Xello"));
  CHECK(contains(senderBoard.usb->tx, "[WARNING] This ACK does not match"));
  CHECK(!contains(senderBoard.usb->tx, "[LINK OK]"));

  showTranscript(senderBoard);
}


// The receiver never answers an "ACK: ..." line (no endless ACK loop).
static void test_receiver_ignores_ack()
{
  receiver::setup();

  for (char c : std::string("ACK: hello\r\n"))
    receiver::Serial1.rx.push_back(c);

  for (int i = 0; i < 100; i++)
  {
    receiver::loop();
    sim::advanceMs(1);
  }

  CHECK(contains(receiver::Serial.tx, "MESSAGE: ACK: hello"));
  CHECK(receiver::Serial1.tx.empty());
}


// Status receiver: timeout message once per outage, repeated for every
// new outage, and "DATA RESUMED" when messages come back.
static void test_status_timeout_repeats()
{
  RadioLink link;
  link.a = &senderBoard;
  link.b = &statusBoard;
  link.start();

  const std::string &r = statusBoard.usb->tx;

  // No message yet -> no timeout message
  link.runFor(8000);
  CHECK(countOf(r, "NO RECENT TELEMETRY DATA") == 0);

  // First message, then 6 s silence -> timeout reported once
  type(senderBoard, "Hello\n");
  link.runFor(6500);
  CHECK(contains(r, "STATUS: CONNECTED / DATA RECEIVED"));
  CHECK(countOf(r, "NO RECENT TELEMETRY DATA") == 1);

  // Much longer silence -> still reported only once
  link.runFor(15000);
  CHECK(countOf(r, "NO RECENT TELEMETRY DATA") == 1);

  // Data comes back
  type(senderBoard, "Second message\n");
  link.runFor(500);
  CHECK(contains(r, "DATA RESUMED"));
  CHECK(contains(r, "MESSAGE: Second message"));

  // Second outage -> reported again
  link.runFor(6000);
  CHECK(countOf(r, "NO RECENT TELEMETRY DATA") == 2);

  // The sender still receives ACKs from the status receiver
  CHECK(countOf(senderBoard.usb->tx, "[LINK OK]") == 2);

  showTranscript(statusBoard);
}


// Status receiver: regular messages (every 2 s) -> never a timeout.
static void test_status_no_false_timeout()
{
  RadioLink link;
  link.a = &senderBoard;
  link.b = &statusBoard;
  link.start();

  for (int i = 0; i < 10; i++)
  {
    type(senderBoard, "PING " + std::to_string(i) + "\n");
    link.runFor(2000);
  }

  CHECK(countOf(statusBoard.usb->tx, "STATUS: CONNECTED / DATA RECEIVED") == 10);
  CHECK(countOf(statusBoard.usb->tx, "NO RECENT TELEMETRY DATA") == 0);
}


// ----------------------------------------------------------------
// Test runner (one child process per test)
// ----------------------------------------------------------------

struct TestCase
{
  const char *name;
  void (*function)();
};

int main(int argc, char **argv)
{
  for (int i = 1; i < argc; i++)
  {
    if (std::string(argv[i]) == "-v")
      verbose = true;
  }

  const TestCase tests[] =
  {
    { "basic round trip (Hello drone -> ACK)",      test_basic_round_trip },
    { "all example messages",                      test_several_messages },
    { "Serial Monitor 'Both NL & CR'",             test_crlf_line_ending },
    { "Serial Monitor 'No line ending'",           test_no_line_ending },
    { "empty lines are not sent",                  test_empty_line_not_sent },
    { "return path broken -> [NO ACK]",            test_return_path_broken },
    { "forward path broken -> [NO ACK]",           test_forward_path_broken },
    { "corrupted ACK is detected",                 test_corrupted_ack_detected },
    { "receiver never ACKs an ACK",                test_receiver_ignores_ack },
    { "status: timeout once per outage + resume",  test_status_timeout_repeats },
    { "status: no false timeout with traffic",     test_status_no_false_timeout },
  };

  int failed = 0;

  printf("Telemetry communication - host simulation tests\n");

  for (const TestCase &t : tests)
  {
    fflush(stdout);

    pid_t pid = fork();

    if (pid == 0)
    {
      t.function();
      fflush(stdout);
      _exit(checksFailed == 0 ? 0 : 1);
    }

    int status = 0;
    waitpid(pid, &status, 0);

    bool ok = WIFEXITED(status) && WEXITSTATUS(status) == 0;

    printf("  [%s] %s\n", ok ? "PASS" : "FAIL", t.name);

    if (!ok)
      failed++;
  }

  printf("%d of %d tests passed\n", (int)(sizeof(tests) / sizeof(tests[0])) - failed,
         (int)(sizeof(tests) / sizeof(tests[0])));

  return failed == 0 ? 0 : 1;
}
