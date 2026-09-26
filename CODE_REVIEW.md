# Code Review — What Was Checked, Found and Fixed

Both programs were reviewed line by line, compiled for the Arduino Mega 2560 with all compiler warnings
enabled, and executed in a PC simulation ([tests/](tests)) together with the **first versions**, so every
finding below was confirmed by a test and not only by reading.

Severity: **Bug** = wrong result or wrong behavior · **Robustness** = can fail in real use ·
**Improvement** = better diagnostics, memory use or usability.

---

## 1. Telemetry text communication

**Result:** the basic sender and receiver logic of the first version was **correct**: `Hello drone` is
sent, displayed and acknowledged in the simulation. One real bug was in the status receiver.

| # | Severity | Finding | Fix | Verified by |
|---|---|---|---|---|
| T1 | **Bug** | `telemetry_receiver_status`: two different `static bool alreadyPrinted` variables (one in the `if` block, one in the `else` block). A `static` variable belongs to its block, so the `else` branch reset a variable nobody reads. `NO RECENT TELEMETRY DATA` was printed **only for the first outage**, never again. The compiler warns: `variable 'alreadyPrinted' set but not used`. | one shared flag `timeoutReported`, reset when a message arrives; new `STATUS: DATA RESUMED` message | test *status: timeout once per outage + resume* — first version FAILS, final version PASSES |
| T2 | Improvement | the sender printed the reply but did not check it | the sender verifies `reply == "ACK: " + sent text`, prints `[LINK OK]` with the **round-trip time**, `[WARNING]` for a corrupted ACK and `[NO ACK]` after 3 s with hints for the return path | tests *return path broken*, *forward path broken*, *corrupted ACK* |
| T3 | Robustness | if both boards ran a receiver sketch, every `ACK:` would be acknowledged again (`ACK: ACK: ACK: …`) | the receiver never acknowledges a line starting with `ACK:` | test *receiver never ACKs an ACK* |
| T4 | Improvement | text constants were copied into RAM | `F("…")` macro keeps them in flash (e.g. status receiver 522 B → 387 B RAM) | compiler size report |
| T5 | Cleanup | `lastMessageTime` was set but never used in the basic receiver | replaced by a message counter (`Messages received: n`) | — |

Behavior checked and found correct (no change needed): TX/RX crossing, `Serial1` pins 18/19, the two
baud rates, `readStringUntil('\n')` + `trim()` (also works with "Both NL & CR"; with "No line ending"
the text is sent after the 1 s timeout), empty lines are not sent.

---

## 2. FlySky FS-i6X + receiver health test

**Result:** the test sequence and the scoring scheme are good and were kept (10 + 15 + 4 × 10 + 15 + 20 = 100
points, limits 90 / 80 / 65). Several measurements could, however, give **full points for a failed function**.

| # | Severity | Finding | Effect (confirmed in simulation) | Fix |
|---|---|---|---|---|
| F1 | **Bug** | `capturePosition()` returned `stats[channel].average`, which is **0** when the channel had no valid pulse; the travel was then computed against 0 µs | CH2 lost only during *PITCH DOWN*: travel = \|1999 − 0\| → **pitch 10/10, total 100/100 EXCELLENT** | a missing measurement is `NAN` ("not measured"), travel score 0, message *NO VALID SIGNAL* |
| F2 | **Bug** | stability test: a channel without pulses has SD = 0 and counted as perfectly stable | CH4 wire disconnected → **stability 15/15** | every channel needs ≥ 90 % valid pulses, otherwise 0 points and *NOT MEASURABLE* |
| F3 | **Bug** | range test: one `max − min` span over the whole 20 s | RF link lost after 8 s with hold/failsafe PWM → **range 20/20, EXCELLENT** | 20 s split into ten 2-s windows; a window is LIVE only with ≥ 300 µs roll movement; the score also requires ≥ 90 / 80 / 60 % live windows |
| F4 | **Bug** | recovery test passed whenever PWM pulses were present | receiver outputs failsafe values, transmitter never switched back on → **RECOVERY PASS** | operator moves roll during the measurement; ≥ 95 % availability **and** ≥ 400 µs roll span required |
| F5 | **Bug** | "EXCELLENT — All major functional tests passed" could be printed while a category scored 0 | e.g. 90/100 with pitch 0/10 → EXCELLENT | EXCELLENT / GOOD require that no category scored 0 (condition limited to MARGINAL, same rule as a failed recovery); numeric score unchanged |
| F6 | Improvement (safety) | failsafe test started with the throttle at minimum, so *hold last value* and *throttle-low failsafe* looked identical; no verdict for the throttle | a receiver that keeps the throttle up after link loss was not reported | STEP 1 raises the throttle to half; the behavior of each channel is reported (NO PULSES / UNCHANGED / CHANGED TO …); throttle verdict **SAFE** or a prominent **WARNING**; still not part of the score |
| F7 | Robustness | about 4.5 KB of text strings were copied into RAM: 4 537 B = **55 %** of the Mega's 8 KB | little room for the stack | `F("…")` macro: **399 B (4.9 %)** |
| F8 | Improvement (precision) | standard deviation computed as `sumSquares/n − mean²` with the Mega's 32-bit `double` | error up to about 1–3 µs on long measurements (small compared with the 8 / 16 / 30 µs limits) | Welford's online algorithm — exact to < 0.01 µs |
| F9 | Robustness | `pulseIn` timeout 25 ms while the worst case at 50 Hz is ≈ 22 ms | only 3 ms margin | 30 ms |
| F10 | Improvement | the test ran once and needed RESET; no unit identification; hard to compare units | slow for many units | unit ID prompt, automatic next test with all results reset, `CSV_RESULT` line per unit |
| F11 | Improvement | diagnostics | — | channel without signal named with its pin, center verdict per channel, travel shown after each pair, warnings for missing pulses / unsteady stick, Mode 2 stick hints, live-window log during the range test |
| F12 | Cleanup | unused `GOOD_TRAVEL` constant; `initialConnectionPassed` set but never shown | — | constant removed; connection PASS/FAIL shown in the report |

### Simulation results — first version vs. final version

| Scenario | First version | Final version |
|---|---|---|
| healthy system | 100 EXCELLENT | 100 EXCELLENT |
| failsafe = hold (throttle stays at half) | 100 EXCELLENT, no warning | 100 EXCELLENT + throttle **WARNING** |
| reduced pitch travel 1205–1770 µs | 92 (pitch 2/10) | 92 (pitch 2/10) — unchanged |
| CH2 lost during *PITCH DOWN* | **100 EXCELLENT** ✗ | 90, pitch 0/10, MARGINAL ✓ |
| RF link lost in the middle of the range test | **100 EXCELLENT** ✗ | 80, range 0/20, MARGINAL ✓ |
| transmitter does not reconnect | **RECOVERY PASS, EXCELLENT** ✗ | RECOVERY FAIL, MARGINAL ✓ |
| CH4 wire disconnected | 55 FAIL, stability **15/15** ✗ | 40 FAIL, stability 0 ✓ |

---

## 3. Verification summary

| Check | Result |
|---|---|
| `arduino-cli compile -b arduino:avr:mega --warnings all` (4 sketches) | no warnings, RAM ≤ 4.9 % |
| `make -C tests` — telemetry simulation | 11 / 11 tests pass |
| `make -C tests` — FlySky simulation | 12 / 12 scenarios pass |
| GitHub Actions ([ci.yml](.github/workflows/ci.yml)) | compiles all sketches and runs the tests on every push |

The simulation cannot replace a hardware test: radio latency, real PWM frame timing and `pulseIn`
measurement noise are modeled, not measured. Run the procedures in the project READMEs on the real
hardware.
