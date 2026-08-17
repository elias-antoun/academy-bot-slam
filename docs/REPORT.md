# AcadBot Courier: a delivery robot that tells the truth

Final project for the Robotics Academy SLAM track. A service books a delivery between two
named places; an action runs it, streaming progress, retrying honestly, and stopping cleanly
when asked. The driving is Nav2's — this is the mission layer above it.

[`BACKGROUND.md`](BACKGROUND.md) builds the theory from first principles.
[`COMPONENTS.md`](COMPONENTS.md) is the piece-by-piece reference. This document is what was
built, what it measured, and what broke.

---

## Table of contents

| | |
|---|---|
| [0. The result in one page](#0-the-result-in-one-page) | headline numbers |
| [1. The task](#1-the-task) | the nine requirements, and what is out of scope |
| [2. The design](#2-the-design) | service vs action, packages, the state machine |
| [3. The procedure, chronologically](#3-the-procedure-chronologically) | eight phases, in the order they happened |
| [4. Results](#4-results) | every measurement |
| [5. Problems faced](#5-problems-faced) | six autopsies, including the one that cost a day |
| [6. Ablations and negative results](#6-ablations-and-negative-results) | four things that did not work |
| [7. Analysis](#7-analysis) | reading the results |
| [8. Engineering](#8-engineering) | build, tests, commits |
| [9. What I would do next](#9-what-i-would-do-next) | |
| [10. Presentation appendix](#10-presentation-appendix) | slides, demo script, questions |
| [11. Bonus: the same mission as a behaviour tree](#11-bonus-the-same-mission-as-a-behaviour-tree) | what it deleted, what it cost, three bugs it introduced |

---

## 0. The result in one page

All nine requirements implemented and verified **on the real stack** — Gazebo, `map_server`,
AMCL, the full Nav2 server set, and the courier.

| # | Requirement | Evidence |
|---|---|---|
| 1 | Service, immediate reply, identifier | `accepted=True, job_id='job_0001'` returned without delay |
| 2 | Reject bad jobs with a reason | 5 distinct refusals, each naming what could have been said instead |
| 3 | Action for execution | both legs driven under one `ExecuteDelivery` goal |
| 4 | Feedback a human can follow | leg, target, distance, attempt, recoveries, state at 2 Hz |
| 5 | Drive with Nav2 | `navigate_to_pose` client; no planner or controller written |
| 6 | Cancel means stop | `CANCELED` in **17 ms** driving, **4.5 ms** mid-retry; robot stopped |
| 7 | Honest failure | retry *saved* one delivery; another reported `failed_leg: dropoff` |
| 8 | Everything in YAML | 4 locations + 5 limits; no coordinate in any `.cpp` |
| 9 | One command | Nav2 `active` **20 s** after launch, no RViz click |

The headline delivery:

```
success: true
message: delivered from 'charging_dock' to 'storage'
failed_leg: ''
attempts_used: 3
Goal finished with status: SUCCEEDED       # ~50 s, 0 recoveries
```

**Plus the bonus:** the same mission implemented a second time as a **BehaviorTree.CPP** tree
(§11). `mission:=bt` runs it over the identical service and action; the state machine is
byte-for-byte untouched. Both engines produce identical results on identical inputs, and the
tree costs about **0.4 s** more per two-leg delivery — which turns out to be the tick rate, not
the tree.

**Two numbers worth the whole report.** The one route that does not complete —
`reception → lab_bench` — fails because DWB weights path-following **1600×** more than
obstacle proximity (`PathAlign.scale: 32.0` vs `BaseObstacle.scale: 0.02`) and drives its own
footprint into a wall. And the hardest single problem, seeding AMCL without a mouse, took
**four** attempts at the verification alone — the first three each *passed while the system
was broken*.

Scale: 2 packages, 3 nodes, 2 interchangeable mission engines, ~2,260 lines of C++ and 74 of
tree XML, 40 commits.

---

## 1. The task

### 1.1 What a courier is

Someone asks for a delivery — *"take this from reception to lab_bench"*. The robot accepts the
job, drives to the pickup, then to the dropoff, reports what it is doing while it moves, and
tells the truth when something goes wrong. If the requester changes their mind halfway, they
cancel and the robot stops cleanly.

### 1.2 The nine requirements

Paraphrased from the brief: accept jobs through a **service** with an immediate reply and an
identifier; **reject** bad jobs with a human-readable reason; run jobs through an **action**;
publish **feedback** at least once a second naming the leg, the destination and the distance
left; drive with **Nav2**; make **cancel** actually cancel; **retry** a configurable number of
times then fail *naming the leg*; put everything configurable in **YAML**; and bring the whole
thing up with **one command**.

The one line that shapes every design decision:

> A robot that reports success for a delivery it did not make is the one unforgivable bug in
> this project.

### 1.3 What is explicitly out of scope

> Under the hood your code is a `navigate_to_pose` action client. **You do not write a planner
> or a controller.**

`acadbot_navigation/config/nav2_params.yaml` is untouched. This matters twice in what follows:
it is why the courier is a thin mission layer, and it is why the one failing route was
*documented* rather than fixed by retuning DWB (§5.4).

---

## 2. The design

### 2.1 Service versus action — the central decision

The brief tests one idea above all: do you know which ROS 2 primitive fits which job?

**Booking is a transaction.** It either succeeds or it does not, it takes no measurable time,
and there is nothing to report while it happens. That is a **service**.

**Executing is a journey.** Minutes long, continuously interesting, and it must be
interruptible. That is an **action** — goal, feedback, result, plus accept/reject and cancel.

Splitting them this way also answers "what does the identifier do": the service mints a
`job_id`, and the action goal carries nothing else.

```
string job_id        # ExecuteDelivery.action, Goal — that is the whole goal
```

**The poses are resolved once, at booking, and frozen into the job.** Re-resolving them when
the action runs would let an edit to `courier.yaml` move the target of a job already accepted
— the requester is told one thing and the robot does another.

### 2.2 Packages and nodes

| package | contents |
|---|---|
| `acadbot_courier_msgs` | `RequestDelivery.srv`, `ExecuteDelivery.action` |
| `acadbot_courier` | `courier_server`, `courier_bt_server`, `initial_pose_seeder`, config, launch, rviz, `behavior_trees/` |

Interfaces are their own package so consumers can depend on the contract without dragging in
the node that implements it.

`initial_pose_seeder` is a separate executable rather than folded into the courier: a delivery
service that also sets up localization is harder to defend than a sixty-line node whose name
says what it does — and it can be dropped from the launch if you prefer to click.

`courier_server` builds from four sources rather than a library plus a thin `main`. Nothing
outside the package links against it, no tests depend on it, and every other package in the
tree builds plain executables.

### 2.3 The state machine

Two axes, not one enum, mirroring what the feedback publishes:

```
leg   ∈ { PICKUP, DROPOFF }
phase ∈ { NAVIGATING, RETRY_WAIT, CANCELING }
```

Driven entirely by callbacks and timers. **Nothing blocks** — no `spin_until_future_complete`,
no worker thread, no waiting on a future. That is what makes a single-threaded executor
sufficient for a node that is simultaneously an action server and an action client, and it is
why none of the state needs a mutex.

The one genuinely subtle piece is `CancelReason`. A `CANCELED` result from Nav2 is ambiguous:
the client cancelled, our leg timeout gave up, or Nav2 abandoned the goal by itself. Three
outcomes, so the *reason* is recorded when the cancel is issued rather than guessed at when
the result lands.

This is one of the two mission engines. The same logic exists a second time as a
BehaviorTree.CPP tree in `courier_bt_server`, selected with `mission:=bt`, over the identical
service and action — §11. The state machine described here is unchanged by its existence, which
is deliberate: it is the verified implementation and the demo fallback.

### 2.4 Configured versus deliberately fixed

| configured | fixed in code |
|---|---|
| the four locations (`name: [x, y, yaw]`) | one job at a time |
| `max_retries`, per leg | the two legs, in that order |
| `retry_delay`, `feedback_period`, `leg_timeout` | terminal-state semantics |
| `goal_frame` | jobs remembered for the process lifetime |
| AMCL's initial pose (`initial_x/y/yaw`) | Nav2's planner and controller |

"One job at a time" is fixed on purpose: a running job owns the robot, and a queue would be a
different product with different failure modes.

---

## 3. The procedure, chronologically

### Phase 0 — Reading the ground

Read the whole repository first. The brief says to read `patrol_commander.cpp` before writing
anything, and it is right for a reason it does not state: that file is a working example of
the exact composition needed (a C++ Nav2 action client, fully async) **and** it gets three
things wrong for a courier — it discards the goal handle so cancel is impossible, it treats
`SUCCEEDED`, `ABORTED`, `CANCELED` and *rejected* identically, and it has no retry.

Also found: the map-frame offset (`map = gazebo + (3, 2)`) and the fact that
`patrol_waypoints.yaml`'s compiled defaults are stale world-frame values that land outside the
building — a live demonstration of why requirement 8 exists.

### Phase 1 — The interfaces

`acadbot_courier_msgs` first, because it is the only piece with zero dependency on anything
unverified and everything else compiles against it. Built and confirmed with
`ros2 interface show`.

### Phase 2 — Location validation, and a detour

Drove all four candidate locations as raw `navigate_to_pose` goals to check they were
reachable. Results in §4.2. `lab_bench` at `(6.20, 4.20)` aborted from both directions.

**This phase was sequenced wrong**, and it is worth recording. The early code does not depend
on real coordinates — placeholders would have carried for days — and a "read off four
coordinates" task turned into an extended Nav2 investigation. What it *did* produce was
requirement-9 knowledge that would otherwise have been discovered later: the seed must precede
Nav2, and `distance_remaining` is populated.

### Phase 3 — Value types and the location table

`types.hpp`, `location_table.*`, `location_markers.*`, one file at a time, syntax-checked
individually with `g++ -fsyntax-only` against the ROS include tree so each could be verified
before the package was buildable.

The location encoding changed twice under review: three scalar parameters per location became
one `[x, y, yaw]` array (fewer lines, reads like a table), and the `locations.names` list was
then removed entirely in favour of reading parameter overrides — because a list and a set of
poses can disagree in one direction silently.

### Phase 4 — The courier node

`courier_server.hpp` first, agreeing the state machine shape before any implementation:
the two axes, the event list, the transition table, the two cancel races. Then
`courier_server.cpp` and `main.cpp`. First successful build of the package.

### Phase 5 — Testing against a stand-in

Rather than bring up Gazebo for every iteration, wrote a `rclpy` `navigate_to_pose` server that
succeeds, aborts, hangs or rejects on command. That turned a two-minute test cycle into five
seconds and made the failure paths *reproducible on demand* instead of hoping the real robot
misbehaved conveniently.

All of requirements 1–7 verified here before the real stack was involved, including both
cancel paths.

### Phase 6 — The launch, and the seeding problem

`courier.launch.py`, `amcl.yaml`, `courier.rviz`. Then requirement 9's real difficulty: AMCL
publishes nothing until told where the robot is, and "one command" cannot mean "one command
and then go and click something".

This took eight attempts across two mechanisms and is the subject of §5.1.

### Phase 7 — Real-stack verification

Every requirement re-verified against Gazebo + AMCL + Nav2. Requirement 7 was proven by an
accident rather than a contrived test: Nav2 genuinely exhausted its recoveries on the
`lab_bench` dropoff, and the courier reported the failure naming the leg.

### Phase 8 — Route selection

With `reception → lab_bench` failing reproducibly, tested `charging_dock → storage` on the
**stock** stack — no tuning, nothing modified. It completes. That became the delivery to
demonstrate, and the failing route was kept deliberately as the honest-failure demonstration.

### Phase 9 — The behaviour tree, and a review that paid for itself

Only after all nine requirements were verified. The tree was written, it compiled, and it ran a
delivery — at which point a deliberate review found **three** defects in it, each one something
the state machine already handled correctly (§11.4). One of them would have left the robot
driving after the client was told the delivery was cancelled.

The lesson is about sequencing rather than about trees: a second implementation of working
behaviour is *the* situation where "it ran successfully" is worthless as evidence, because the
happy path is the easy part and it is already known to be reachable. What the review actually
did was compare the new code against the old one line by line, asking of every piece of state in
`courier_server.hpp` where it went. Two of the three bugs fell straight out of that question.

Phase 9 also produced the only tooling kept in the repository,
[`tools/fake_nav2.py`](../tools/fake_nav2.py) — the scratchpad version had been lost once
already, and it was the thing that made all three fixes verifiable in seconds.

---

## 4. Results

### 4.1 The nine requirements, on the real stack

```
# req 1, 2
accepted=True,  job_id='job_0001', reason=''
accepted=False, job_id='',  reason="unknown pickup 'kitchen'; known locations are:
                                    charging_dock, lab_bench, reception, storage"
accepted=False, job_id='',  reason="pickup and dropoff are both 'reception';
                                    there is nothing to deliver"

# req 3, 4, 5, 7 — the successful delivery
job_0001: pickup leg, attempt 1 of 3 -> 'charging_dock' (0.50, 2.00)
job_0001: pickup leg, attempt 1 failed: Nav2 aborted the goal
job_0001: retrying the pickup leg in 3.0s (attempt 2 of 3)
job_0001: reached pickup 'charging_dock'
job_0001: dropoff leg, attempt 1 of 3 -> 'storage' (5.50, 0.60)
job_0001: delivered from 'charging_dock' to 'storage'

# req 6
terminal status  : CANCELED
result.success   : False
result.failed_leg: ''

# req 9
bt_navigator: active [3]        # t = 20 s, no RViz click
```

### 4.2 Location reachability

Measured by sending each candidate as a raw `navigate_to_pose` goal.

| location | map (x, y, yaw) | result | recoveries |
|---|---|---|---|
| `reception` | 0.60, 4.20, 0.00 | SUCCEEDED | 0 |
| `storage` | 5.50, 0.60, 0.00 | SUCCEEDED | 0 |
| `charging_dock` *(original)* | 0.50, 0.50, −1.57 | SUCCEEDED | **4** |
| `lab_bench` *(original)* | 6.20, 4.20, 3.14 | **ABORTED** | 16 / 0 / 5 |

Two were moved as a result:

- **`lab_bench` 6.20 → 4.50.** Reaching the north-east corner means threading the gap between
  `obstacle_2` (a 0.3 m cylinder at map `(5.5, 3.5)`) and the east wall at `x ≈ 6.925`. That
  gap is **1.125 m** of physical space, of which **~0.22 m** survives 0.45 m of inflation from
  each side — against a robot needing 0.40 m. It aborted from the west (16 recoveries, wedged
  at `(2.88, 1.08)`) and from the east (0 recoveries, wedged at `(6.01, 1.71)`), and once
  wedged, the next goal failed too.
- **`charging_dock` (0.50, 0.50) → (0.50, 2.00).** `obstacle_1` is a 0.5 m box at map
  `(1.0, 0.5)`, edge at `x ≈ 0.75`. The goal sat 0.25 m from it, inside the inflation band —
  four recoveries on every visit.

### 4.3 Delivery routes

| route | outcome | time | recoveries | attempts |
|---|---|---|---|---|
| `charging_dock → storage` | **SUCCEEDED** | ~50 s | 0 | 3 |
| `reception → lab_bench` | pickup ok, **dropoff ABORTED** | — | 15 | 4 |

The successful one is richer than a clean run: Nav2 aborted the first pickup attempt, the
retry recovered it, and the delivery completed. **Requirement 7's retry demonstrated in the
success path**, not only the failure path.

### 4.4 Cancel

| cancel landed | courier → terminal | chain |
|---|---|---|
| robot driving at 0.19 m/s | **17 ms** | `controller_server: Cancellation was successful. Stopping the robot.` |
| during the 3 s retry delay | **4.5 ms** | no Nav2 goal in flight; deferred finish |
| at `distance_remaining: 0.19` | `CANCELED` | inside the 0.20 m goal tolerance and still not reported as success |

`/cmd_vel` measured at 0.19 m/s immediately before the cancel, and silent afterwards.

### 4.5 Bringup and environment

| quantity | value |
|---|---|
| Nav2 `active` after launch | 20 s |
| seeder attempts needed | 2 |
| simulation real-time factor | 0.708 |
| `/scan` rate | 7.7 Hz |
| map | 160 × 120 cells @ 0.05 m = 8.0 × 6.0 m |
| map origin | `[-1.054, -1.009]` |

---

## 5. Problems faced

### 5.1 Seeding AMCL without a mouse — eight attempts

The single hardest problem, and the most instructive.

**The requirement.** AMCL is handed a map it did not build and refuses to guess where the
robot is, so it publishes nothing until told. Without `map → odom` the Nav2 costmaps cannot
configure, the lifecycle manager times out and aborts the *entire* bringup, and every goal
afterwards is rejected. The symptom appears three layers from the cause.

**Attempt 1 — `set_initial_pose`.** The obvious tool. The parameter loaded correctly and AMCL
genuinely applied it:

```
[amcl] Setting pose (0.000000): 0.000 0.000 0.000
```

**t = 0.** AMCL applies it the instant it activates — ~0.4 s after its process starts, before
it has received a single `/clock` message — so under `use_sim_time` the pose is stamped at a
time for which no `odom→base_footprint` has ever existed. Accepted, and no transform ever
validates.

**Attempt 2 — delay localization by 8 s**, so the clock is running. Still `(0.000000)`. The
race is between AMCL's own activation and its own first clock message, so it moves along with
the delay. *This attempt also made things worse* (attempt 6).

**Attempt 3 — `transform_tolerance: 1.0`.** No effect. tf2 never permits **future**
extrapolation regardless of tolerance; the parameter only admits stale data.

**Attempt 4 — publish `/initialpose` three times.** All three lost a millisecond-scale race:

```
Requested time  9.867 but the latest data is at time  9.860    (7 ms into the future)
Requested time 10.571 but the earliest data is at time 10.900  (buffer starts later)
```

**Attempt 5 — publish ten times, later.** Still lost.

**Attempt 6 — start localization with the sim again.** The delay from attempt 2 was actively
harmful: AMCL keeps its own TF buffer, and that buffer only starts filling when the node does,
so a late start leaves it holding a single sample.

**Attempt 7 — a seeder node that verifies.** Publish, check, repeat until it worked. Right
idea; wrong check, three times over:

| check | why it lied |
|---|---|
| does `map→odom` exist? | tf2 caches ten seconds and `TimePointZero` means *latest available* — a transform emitted once and abandoned kept answering yes |
| exists three times running? | three checks a second apart all read the same dead transform |
| how old is it? | age is in **simulated** time, which barely advances while Gazebo starts — exactly the window the check polices |

**Attempt 8 — ask whether the stamp is _changing_.**

```cpp
const bool advancing = have_stamp_ && stamp > last_stamp_;
```

A stamp that keeps moving means AMCL is still publishing. That holds whatever the clock is
doing. **Nav2 active at 20 s, two published attempts, no click.**

Two things also learned along the way. A side experiment stamping the pose three ways — zero,
`now()`, `now() − 0.5 s` — showed **all three working** once TF was mature, so the timestamp
was never the discriminating variable, contrary to what four attempts assumed. And the seeder
needed a second gate nobody would guess: **AMCL only publishes `map→odom` off the back of a
filter update, and a filter update needs a scan.** Seeding before the laser streams gets the
pose accepted, a transform emitted briefly, then silence.

**The lesson, which is the most transferable thing in this project:** the fix was right on
attempt 7; the *verification* was wrong three times. Each check tested something adjacent to
what mattered and reported success on a broken system.

### 5.2 `map_server` and parameter precedence

AMCL sat on `Waiting for map....` forever. `map_server` was `active` but empty:

```
map_server: yaml-filename parameter is empty, set map through 'load_map'-service
$ ros2 param get /map_server yaml_filename
String value is:
```

Cause: I had written `yaml_filename: ""` into `amcl.yaml` intending it to be explicit and
self-documenting. `nav2_bringup`'s `localization_launch.py` passes the map as
`parameters=[params_file, {'yaml_filename': map_yaml_file}]` — and **the params file wins over
the dict override**, the opposite of the precedence I assumed. Declaring it did not document
the map source; it silently blanked it.

Fix: no `map_server` block at all — which is exactly why `nav2_params.yaml` has none either.

### 5.3 The corner-cutting controller

`reception → lab_bench` reaches the pickup, then fails the dropoff:

```
robot at (3.25, 0.90)
GridBased plugin failed to plan from (3.25, 0.90) to (4.50, 4.20)
distance_remaining: min 0.0, last 0.0        # never had a valid plan
15 recoveries
```

The divider's west face is at map `x = 3.425`. The robot sits at `x = 3.25` — **0.175 m away,
with a 0.20 m footprint radius.** Its footprint is inside the wall. That is why the planner
cannot plan and `backup failed`: it is in collision, not merely near an obstacle.

The corridor itself is fine — 1.925 m of gap, 1.025 m still clear after inflation, and
`spawn → storage` crosses it with **zero** recoveries. The problem is the *approach*: from the
far north-west the robot runs down the divider's west face and clips the tip while rounding
it. `nav2_params.yaml` sets `BaseObstacle.scale: 0.02` against `PathAlign.scale: 32.0`, so DWB
weights staying on the line 1600× more than keeping clear of walls.

Fixing that means retuning the controller, which the brief puts out of scope. So the route is
kept as the honest-failure demonstration, which is what it is good at.

### 5.4 Three bugs in my own test scripts

Each hid or faked a result and each cost a full run:

- `grep -o "job_[0-9]*"` matches **zero** digits, so it also matched the field name `job_id`
  and passed `"job_\njob_0002"` as an id. Two tests silently never ran, and a third "passed"
  for the wrong reason.
- `timeout 4 ros2 run tf2_ros tf2_echo map odom` — `tf2_echo` prints "Waiting for transform"
  a couple of times before resolving and needs ~5 s. The timeout killed it at the boundary and
  **reported failure on a working stack**, which sent me back into §5.1 for another round.
- `head -24` truncated the cancel client at exactly the line before the terminal status — the
  one line that test existed to produce.

**A check that truncates is a check that can lie**, and it lies toward "no result", which
reads as failure.

The eventual fix for the second was to stop probing with `tf2_echo` at all and wait on
`bt_navigator` reaching `active` — which *cannot* happen without `map→odom`, so it proves the
same thing without needing a clock of its own to be right about.

### 5.5 Two rclcpp API surprises

`declare_parameter<T>(name)` with no default throws
**`UninitializedStaticallyTypedParameterException`** — not the name anyone guesses. Checked in
the Jazzy headers rather than assumed.

`automatically_declare_parameters_from_overrides` looks like the right way to read a location
table, but it declares *every* key at construction, so the node's own
`declare_parameter("max_retries", 2)` then throws `ParameterAlreadyDeclaredException`. Every
parameter would need a `has_parameter` guard to fix one feature. Reading
`get_parameter_overrides()` directly touches nothing else.

### 5.6 A name collision worth knowing

A private static `to_string(Phase)` would **hide** the free `to_string(Leg)` and
`to_string(JobState)` from `types.hpp` — member lookup wins — so every call inside the class
would need qualifying. Renamed to `phase_name`.

---

## 6. Ablations and negative results

### 6.1 Reducing controller speed — partial, and revealing

Hypothesis (proposed after watching a run): 0.26 m/s and 1.0 rad/s are too fast, and the robot
overshoots into walls.

| speed | result |
|---|---|
| 0.26 / 1.0 (stock) | wedges at `(3.24, 1.22)`, planner fails |
| 0.20 / 0.70 | **planner failures gone**; leg times out at 120 s, 7 recoveries |
| 0.20 / 0.70, 240 s timeout | still stuck at `(3.25, 0.90)`, 15 recoveries |

**Partly right.** Slowing down genuinely stopped the robot wedging hard enough to break the
planner — the `Failed to create plan` errors vanished at 0.20 m/s. It did not get the robot
round the corner, because the cause is critic weighting rather than speed. Kept out of the
final configuration: it costs a third more delivery time for a fix that does not fix.

An earlier speed test was **invalid** and worth flagging as a methodology note: all three
`/initialpose` messages were rejected on TF races, so localization never established and every
goal failed for want of a map, not for want of speed. Changing two things at once (speed *and*
the launch composition) hid it.

### 6.2 `set_initial_pose` — the right-looking wrong tool

Covered in §5.1. Loads correctly, is applied correctly, and cannot work under `use_sim_time`
because activation precedes the first `/clock` message. No delay fixes it.

### 6.3 `transform_tolerance` — a misreading of tf2

Set to 1.0 expecting it to absorb a 7 ms overshoot. It does not: tf2 refuses **future**
extrapolation regardless of tolerance, which only permits *stale* data. Kept in `amcl.yaml`
anyway because it is correct to set explicitly and it does absorb the stale-side misses.

### 6.4 Delaying localization — actively harmful

Introduced to give the clock time to start; removed once measured. AMCL's TF buffer begins
filling when the node does, so starting it late leaves it with a single sample and makes the
seeding race *worse*. A fix for a wrong diagnosis that broke a working configuration.

---

## 7. Analysis

**Why the successful route works and the other does not.** Both cross the same corridor.
`charging_dock → storage` approaches it from mid-height and passes through the clear band;
`reception → lab_bench` descends the divider's west face and has to round a corner at the tip.
With obstacle proximity weighted at 0.02 against path-following at 32.0, DWB will hug the wall
to stay on its line — fine in open space, fatal at a 0.19 m clearance.

**Why the failure is a good result, not a hidden one.** Requirement 7 asks the courier to
retry a configurable number of times and then fail *naming the leg*. That is exactly what
happens, against a real Nav2 failure rather than a stubbed one:

```
attempt 1 failed → attempt 2 failed → attempt 3 failed
success: false, failed_leg: dropoff, attempts_used: 4
```

A courier that silently skipped the leg — which is what `patrol_commander` does with a
rejected goal — would have reported the delivery done.

**Why the retry matters in the success path too.** `charging_dock → storage` needed
`attempts_used: 3`: Nav2 aborted the first pickup attempt, the retry recovered it. The same
mechanism saves a delivery here and refuses to lie about one there.

**Why no mutex is needed.** The node is an action server and an action client at once — the
classic deadlock setup. Because nothing blocks, a single-threaded executor serializes every
callback by construction. That is a design constraint held deliberately, and it is why the
state machine can be read linearly.

**Why cancel is fast.** 17 ms driving, 4.5 ms mid-retry. The retry case is the interesting
one: no Nav2 goal is in flight, so no result is coming, and without the deferred finish the
action would hang forever waiting for one.

---

## 8. Engineering

**Build.** Two ament_cmake packages, C++17, `-Wall -Wextra -Wpedantic`, no warnings.
Interfaces generate cleanly and are confirmed with `ros2 interface show`.

**Incremental verification.** Each source file was syntax-checked on its own with
`g++ -fsyntax-only` against the ROS include tree before the package was buildable, so errors
surfaced one file at a time rather than in a heap at first link.

**The stand-in Nav2** is the piece of tooling that paid for itself most. A `rclpy`
`navigate_to_pose` server that aborts, stalls or succeeds on command turned a two-minute test
cycle into three seconds and made the failure paths reproducible on demand. Requirements 3, 4,
6 and 7 were all developed against it before the real stack was involved, and every one of the
behaviour tree's three defects was confirmed fixed against it (§11.4).

It is the only tooling kept in the repository, at [`tools/fake_nav2.py`](../tools/fake_nav2.py),
and that decision was made the hard way — the scratchpad version was lost once and rewritten.
Its two later switches exist because the real stack *cannot* produce the conditions they create:
`FAIL_GOALS` makes a specific attempt fail on demand, and `ACCEPT_DELAY` holds open a cancel
window that is a few milliseconds wide on a real Nav2.

**Commits.** 40, each one change with the reasoning in the body — including the ones that
record a wrong turn, because `d065d7e`'s explanation of why `set_initial_pose` cannot work is
worth more than the two lines it deleted.

---

## 9. What I would do next

- ~~**The behaviour-tree bonus.**~~ **Done** — §11. It landed as predicted:
  `RetryUntilSuccessful` and `Timeout` replaced the per-leg retry and the leg timeout, and
  keeping the state machine alongside under `mission:=fsm|bt` made the comparison the
  deliverable. What was *not* predicted is that reimplementing verified behaviour would
  introduce three defects, one of them serious (§11.4).
- **Share the plumbing between the two engines** — but only now that both are trusted. About
  500 lines are duplicated, which was the right call while the tree was unproven and is the
  wrong one to leave standing. A common `JobDesk` holding the service, the job registry and the
  feedback timer would leave each engine as nothing but its mission logic, which is what the
  comparison is actually about.
- **Raise the tick rate, or measure what it should be.** 10 Hz costs 0.4 s per delivery (§11.3)
  and was chosen by intuition. The right number is a measurement, not a guess.
- **A queue.** "One job at a time" is a real limitation, honestly stated. A queue means
  deciding what cancel means for a job that has not started.
- **Bounded job history.** `jobs_` grows for the life of the process so replayed ids can be
  refused. Fine at demo scale, not in a building.
- **The corridor.** Out of scope here, but the fix is known: raise `BaseObstacle.scale`, or add
  an intermediate waypoint at the corridor mouth so the robot approaches the tip head-on.

---

## 10. Presentation appendix

### 10.1 Slide sequence

1. **The product.** *"Take this from reception to lab_bench."* One sentence, one photo of the map.
2. **Two primitives, two jobs.** Booking is a transaction → service. Delivery is a journey →
   action. Show the two `.srv`/`.action` files side by side.
3. **Why the goal carries only a job id.** Poses frozen at booking; a config edit cannot move
   an accepted job's target.
4. **The state machine.** Two axes; the transition diagram; `CancelReason` and why a bool
   cannot express it.
5. **Honesty.** `abort()` never `succeed()`; a rejected Nav2 goal is a failed leg, not a
   skipped waypoint — with the `patrol_commander` contrast.
6. **What is configurable.** `courier.yaml` on one slide.
7. **What broke: seeding AMCL.** Eight attempts, three wrong verifications. The best slide in
   the deck.
8. **What broke: the corner.** `(3.25, 0.90)`, 0.175 m from a wall, 0.20 m radius.
   `BaseObstacle.scale: 0.02` vs `PathAlign.scale: 32.0`.
9. **Results.** The nine-requirement table from §0.
10. **Bonus: the same mission as a behaviour tree.** `courier.xml` beside the four members and
    three callbacks it replaces; the identical demo commands; +0.4 s, which is the tick rate.
    And the three bugs the review found — one of which left the robot driving after the client
    was told CANCELED (§11.4).
11. **What I would change.** The queue; the sequencing mistake in Phase 2; sharing the plumbing
    between the two engines once both are trusted.

### 10.2 Live demo, in order

```bash
ros2 launch acadbot_courier courier.launch.py         # ~20 s to Nav2 active

# 1. a rejection
ros2 service call /request_delivery acadbot_courier_msgs/srv/RequestDelivery \
  "{pickup: 'kitchen', dropoff: 'storage'}"

# 2. a delivery that succeeds  (~50 s)
ros2 service call /request_delivery acadbot_courier_msgs/srv/RequestDelivery \
  "{pickup: 'charging_dock', dropoff: 'storage'}"
ros2 action send_goal /execute_delivery \
  acadbot_courier_msgs/action/ExecuteDelivery "{job_id: 'job_0002'}" --feedback

# 3. a cancel — Ctrl-C the send_goal mid-drive; expect CANCELED and a stopped robot

# 4. an honest failure
#    reception -> lab_bench, or drop an obstacle in the corridor during (2)

# 5. the bonus: the same demo, run by the behaviour tree
#    relaunch with mission:=bt, then repeat (2) verbatim -- the commands do not change
ros2 launch acadbot_courier courier.launch.py mission:=bt
```

Keep the feedback stream visible; keep RViz showing the location markers with the current
target highlighted.

Step 5 is the point of the bonus and it needs saying out loud, because *nothing visibly
happens*: the service, the action, the feedback fields and the RViz view are identical, and the
only difference on screen is the node name in the log prefix. Show
[`courier.xml`](../ros2_ws/src/acadbot_courier/behavior_trees/courier.xml) beside it — 30 lines
that replace the retry counter, the retry timer, the leg timeout and the leg sequencing — and
then the four decorators are the whole argument (§11.1).

### 10.3 Questions to expect

**Why a service and an action, rather than two services or two actions?**
§2.1. Booking has no duration and nothing to report; delivery has both, plus it must be
cancellable.

**What happens if I send the same job id twice?**
Rejected — `it is already SUCCEEDED`. Finished jobs are kept precisely so that check can be
made; deleting them on completion would let a replayed goal deliver twice.

**What if Nav2 is not running when I book?**
The service refuses with *"navigation is not available"*. Checked, not waited on, so the reply
stays immediate — that is requirement 2's "cannot be served right now".

**Why does your node not need a mutex?**
Nothing blocks, so a single-threaded executor serializes every callback. Deliberate: the
alternative is a multi-threaded executor and a mutex on a dozen members.

**One of your routes fails. Why did you not fix it?**
Because fixing it means retuning DWB's critics, and the brief says the courier does not write a
planner or a controller. The robot drives its own footprint into a wall at `(3.25, 0.90)`
because obstacle proximity is weighted at 0.02 against path-following at 32.0. The route is
kept as the honest-failure demonstration — and the courier's behaviour there is exactly right.

**What would you do differently?**
Sequence Phase 2 later — the early code never needed real coordinates, and validating them
first turned into a Nav2 investigation. And write the verification before the fix: the AMCL
seeder was correct on the first try, but three successive checks reported success on a broken
system.

**You built the mission twice. Which one would you ship?**
The state machine, for this mission — see §11.5. The tree wins as soon as the mission grows
priorities and preconditions, and it is the right answer at Nav2's scale. At two legs it is a
demonstration of a technique rather than a better courier, and it says so.

---

## 11. Bonus: the same mission as a behaviour tree

### 11.1 What was built, and what was deliberately not

A second executable, `courier_bt_server`, running the delivery from
[`courier.xml`](../ros2_ws/src/acadbot_courier/behavior_trees/courier.xml) instead of the
hand-written leg loop. `mission:=fsm|bt` selects one at launch; both claim the same service and
action names, so **the client commands are identical** for either.

Three constraints were set before writing anything:

1. **`courier_server.cpp` is not touched.** It is verified and it is the demo fallback. A
   comparison in which one side was disturbed to make room for the other proves nothing.
2. **Duplicate the plumbing; do not refactor to share it.** The service handler, job registry,
   feedback timer and marker publisher were copied — about 500 lines. Extracting a common base
   class would have been better code and a worse decision: it puts working, verified behaviour
   at risk to accommodate a bonus.
3. **`nav2_params.yaml` stays untouched**, as everywhere else in this project.

So the tree did **not** make the project smaller. Total lines went *up*:

| | state machine | behaviour tree |
|---|---|---|
| mission engine | 751 lines C++ | 872 lines C++ + 74 XML |
| of which plumbing duplicated from the other | — | ~500 |

What shrank is the mission logic specifically. Four pieces of hand-written state have no
counterpart in the tree at all:

| gone from the tree | replaced by |
|---|---|
| `attempt_`, `max_retries_`, `retry_timer_`, `on_retry_elapsed()` | `RetryUntilSuccessful` |
| `leg_timeout_timer_`, `on_leg_timeout()`, `CancelReason::LEG_TIMEOUT` | `Timeout` |
| `leg_`, and the pickup→dropoff transition inside `on_nav_result` | `Sequence` |
| "pickup failed, so do not attempt the dropoff" | `Sequence`, for free |

Concretely: three dedicated callbacks totalling **63 lines** (`on_leg_timeout`,
`on_retry_elapsed`, `leg_failed`), four members and two `create_wall_timer` calls, replaced by
four XML decorators that nobody wrote and that every Nav2 installation in the world exercises
daily. Retry count and leg timeout become an edit to a data file rather than a rebuild.

### 11.2 Equivalence and cancellation, measured

Both engines were run against [`tools/fake_nav2.py`](../tools/fake_nav2.py) with
`FAIL_GOALS=1,3` — each leg aborts once, then succeeds — which is the shape that exposes
per-leg bookkeeping.

```
state machine                          behaviour tree
pickup  leg, attempt 1 of 3            leg: pickup,  attempt 1
pickup  leg, attempt 2 of 3            leg: pickup,  attempt 2
dropoff leg, attempt 1 of 3            leg: dropoff, attempt 1
dropoff leg, attempt 2 of 3            leg: dropoff, attempt 2
attempts_used: 4                       attempts_used: 4
status: SUCCEEDED                      status: SUCCEEDED
```

Identical, including the distinction between per-leg `attempt` and cross-leg `attempts_used`.
`FAIL_GOALS=1,2,3` exhausts a leg and both report `success: false`, `failed_leg: pickup`, with
the dropoff never attempted.

**Cancellation** was verified in both of its interesting cases. Cancelling mid-drive:

```
342.1938  BT cancel requested
342.1938  GoToLocation: halted/cancelled
342.1953  cancelled during the pickup leg     ← reported to the client, 1.5 ms
342.2502  fake_nav2: goal 2: canceled         ← the drive actually stopped
```

And cancelling in the window before Nav2 has acknowledged the goal — forced open with
`ACCEPT_DELAY=3`, because on the real stack it is a few milliseconds wide:

```
399.882  GoToLocation: halted/cancelled                      ← no goal handle yet
401.990  goal acknowledged after the halt -- cancelling it
401.999  fake_nav2: goal 1: canceled                         ← drive stopped
```

**One divergence that must not be read as a speed-up.** The state machine's measured 17 ms
cancel (§4.4) waited for Nav2's CANCELED *result* before reporting. A halted tree has no result
callback left to finish on, so the BT reports CANCELED while the stop is still in flight — hence
1.5 ms. The goal is verifiably cancelled either way, as the traces above show, but **the two
numbers measure different things** and comparing them would be dishonest.

### 11.3 Timing: what a tick rate costs

Identical two-leg deliveries, no failures, 1.0 s of driving per leg, wall-clock from goal sent
to result received. The first run of the session was discarded as a cold start (4434 ms).

| engine | runs | mean | spread |
|---|---|---|---|
| `courier_server` (FSM) | 3 | **2915 ms** | 2851–2973 |
| `courier_bt_server` (BT) | 4 | **3360 ms** | 3153–3662 |

**+445 ms**, and the explanation is arithmetic rather than architectural. A tree only changes
state when it is ticked, and this one ticks at 10 Hz. A delivery has roughly four transitions
(start pickup, pickup done, start dropoff, dropoff done), each of which can wait up to one full
tick period: 4 × 100 ms = up to 400 ms. Measured 445 ms.

So this is a **tuning** number, not a design cost. Ticking at 50 Hz would recover most of it and
spend CPU instead. Worth knowing, and worth not over-reading: 0.4 s on a 50 s delivery is 0.9%.

One further cost is structural rather than measured. The `Delay → AlwaysFailure` construction
that gives the tree its inter-attempt pause also runs after the *final* attempt, so an exhausted
leg waits one extra `retry_delay` before being declared failed. The state machine checks its
counter first and reports immediately. Removing it means a scripted precondition inside the tree,
which buys three seconds at the price of the readability that justifies the tree in the first
place — so it was left in, and written down.

### 11.4 Three bugs the second implementation introduced

All three found by review rather than by running it — the delivery worked before any of them was
fixed. All three were things the state machine already got right, which is the useful part: they
are not tree bugs, they are *"reimplemented working behaviour and lost a detail"* bugs.

**1 — the per-leg attempt counter never reset.** `courier_server.cpp` does `attempt_ = 0` at the
leg transition; the tree version incremented forever, so feedback reported *attempt 4, 5, 6* on
the dropoff. There was a second-order mistake underneath it: the leg was overwritten *before* the
increment, so the code could not have detected the transition even if it had tried.

Severity: cosmetic — the delivery completes correctly. Which is exactly why it survived a working
end-to-end run, and why `FAIL_GOALS=1,3` exists.

**2 — a cancel arriving before Nav2 acknowledged the goal was silently dropped.** `onHalted`
cancelled only `if (nav_handle_)`, and in the window between `async_send_goal` and the
goal-response callback there is no handle. Nothing was cancelled; the response then arrived and
was stored into an already-halted node.

Severity: **serious.** The tree stops, the client is told CANCELED, and the robot keeps driving to
the pickup. The state machine handles this explicitly, with a comment saying so — this is a
regression introduced by reimplementation, not an oversight in a new design. It is also
essentially untestable on the real stack, where the window is a few milliseconds wide, which is
why `ACCEPT_DELAY` was added to the harness. The trace in §11.2 is that test.

**3 — Nav2 callbacks bound to an object with the wrong lifetime.** The callbacks captured `this`,
and `this` is a *tree leaf*. A leaf dies with its tree, which happens when the next job calls
`createTreeFromFile`. A result still in flight from a cancelled goal would then write into freed
memory. Fixed with a `weak_ptr` liveness token; verified by inspection only, since triggering it
needs a new job booked inside the few milliseconds while a cancelled goal's result is in flight.

This one generalises furthest: **moving logic into a tree changes object lifetimes.** State that
used to live as long as the process now lives as long as a tree, and every callback bound to it
silently inherits that shorter life. Nothing in the compiler or the tree library warns about it.

### 11.5 What the tree is actually worth

**For this mission, the state machine is the better implementation**, and the tree is a
demonstration of a technique. Two legs, one retry rule and one timeout do not need a tree; every
transition in the state machine is a readable line with a place to put a breakpoint, and the
tree's control flow is a property of tick traversal over a data file.

**The tree is unambiguously right one scale up.** Add "if the battery is low, dock first",
"if the corridor is blocked, wait and re-plan", "on three failures, ask a human" — and the state
machine becomes a combinatorial mess of flags while the tree grows a subtree and a priority. That
is exactly why Nav2's own navigator is a behaviour tree: recovery escalation is configuration
there, and it is `#ifdef`-free.

**The honest summary is that it restructured the easiest part of the project.** The three genuinely
hard problems here were the async discipline (§7), seeding AMCL under simulated time (§5.1) and a
controller that cuts corners (§5.3). A behaviour tree helps with none of them — and the async
discipline it *does* touch, it makes no easier: `StatefulActionNode` exists precisely because a
tick must not block, which is the same rule, restated.

What it did prove, and what makes it worth the bonus: **the mission logic and the interface are
genuinely separable.** Two engines, one service, one action, one config block, identical observed
behaviour on identical inputs. That the swap is invisible to the client is the strongest evidence
in this report that the layering was right.
