# Host simulation tests

These tests compile the **real** Arduino sketches on a PC and run them against simulated hardware, so the
program logic can be checked without any Arduino, radio or transmitter.

| File | What it does |
|---|---|
| [`arduino_mock/Arduino.h`](arduino_mock/Arduino.h) | small replacement of the Arduino API (`Serial`, `String`, `millis`, `delay`, `pulseIn`, `F()`). Time is simulated, so a 90-second test runs in milliseconds. |
| [`test_telemetry.cpp`](test_telemetry.cpp) | two virtual Arduino Megas (sender + receiver) connected by a simulated radio link with latency, broken directions and corrupted characters — 11 tests |
| [`test_flysky.cpp`](test_flysky.cpp) | simulated FS-i6X + receiver (end points, noise, 50 Hz frames, link loss, failsafe: no pulses / hold / preset) and a *virtual operator* that reads every prompt and moves the requested stick — 12 scenarios |

## Run

Requires `g++` (C++17) and `make` on Linux or macOS:

```bash
make -C tests            # build and run everything
make -C tests telemetry  # only the telemetry tests
make -C tests flysky     # only the FlySky scenarios

tests/build/test_telemetry -v        # also print the Serial Monitor transcripts
tests/build/test_flysky -v           # full transcripts of every scenario
tests/build/test_flysky --report-only
```

Example output:

```text
Telemetry communication - host simulation tests
  [PASS] basic round trip (Hello drone -> ACK)
  ...
11 of 11 tests passed
FlySky health test - host simulation scenarios
  - healthy system, failsafe = throttle low
      score 100/100 | EXCELLENT *** | range 20/20 | recovery: PASS | failsafe: THROTTLE LOW (SAFE)
    [PASS]
  ...
12 of 12 scenarios passed
```

The simulation checks the program logic. It does not replace a test on real hardware.
