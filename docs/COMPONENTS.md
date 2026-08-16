# Every component, explained

A reference for the individual pieces the AcadBot Courier is built from. Each entry gives
**what it is**, **why it is there**, and **the code**, with a link to the file it lives in.

[`REPORT.md`](REPORT.md) covers what was built, what it measured and what broke;
[`BACKGROUND.md`](BACKGROUND.md) builds the underlying theory from first principles. This
document sits between them: one entry per moving part.

The whole design is organised around one rule from the brief: **the courier does not write a
planner or a controller.** Nav2 already plans and drives; `acadbot_navigation`'s
`nav2_params.yaml` is byte-for-byte untouched. Everything here is the mission layer above it —
which goals to send, in what order, what to do when one fails, and how to tell a human.

---

## Contents

| Group | Components |
|---|---|
| [A. The interfaces](#a-the-interfaces) | `RequestDelivery.srv` · `ExecuteDelivery.action` · why a separate package |
| [B. Value types and the location table](#b-value-types-and-the-location-table) | `Pose2D` / `Leg` / `JobState` · `Job` and frozen poses · parameter-override discovery · the failure paths · `known_names()` |
| [C. The courier node](#c-the-courier-node) | construction · the service handler · job ids · goal acceptance · cancel and the deferred finish · the state machine · retry · the three endings · feedback on its own clock |
| [D. Seeing it](#d-seeing-it) | location markers · `courier.rviz` |
| [E. Localization seeding](#e-localization-seeding) | `initial_pose_seeder` · the advancing-stamp check · `amcl.yaml` and its two deliberate absences |
| [F. Bringup and configuration](#f-bringup-and-configuration) | `courier.launch.py` ordering · `courier.yaml` |
| [G. The test harness](#g-the-test-harness-development-only) | stand-in Nav2 · cancel client · shell drivers |

---

# A. The interfaces

Two files in [`acadbot_courier_msgs`](../ros2_ws/src/acadbot_courier_msgs). They are the
contract everything else compiles against, and they are where the design is decided.

## A1. `RequestDelivery.srv` — booking

**What.** A service: name a pickup and a dropoff, get back an id or a refusal.

**Why a service.** Booking is a transaction — it either succeeds or it does not, it takes no
measurable time, and there is nothing to report while it happens. Requirement 1 says *"the
reply comes back immediately"*, which is the definition of a service and the opposite of an
action.

```
string pickup       # named location; must exist in the courier's location table
string dropoff      # named location; must exist, and must differ from pickup
---
bool   accepted
string job_id       # refer to this when sending the ExecuteDelivery goal;
                    # empty when accepted is false
string reason       # why it was rejected, in words meant for a human;
                    # empty when accepted is true
```

`reason` is a `string` rather than an error enum on purpose: requirement 2 asks for a
*human-readable* reason, and the messages this project produces name what you could have said
instead (§C2).

## A2. `ExecuteDelivery.action` — running the job

**What.** An action: run a booked job, streaming progress, cancellable.

**Why an action.** It takes minutes, it has something to say the whole time, and it must be
interruptible. Those three properties are exactly what an action provides and a service does
not.

```
string job_id
---
bool   success
string job_id
string message           # human-readable outcome, success or failure
string failed_leg        # "pickup" or "dropoff"; empty when success is true
uint8  attempts_used     # total navigate_to_pose goals sent across both legs
---
uint8 LEG_PICKUP  = 0
uint8 LEG_DROPOFF = 1

uint8   leg
string  target_location
float32 distance_remaining
uint8   attempt
uint16  nav2_recoveries
string  state            # NAVIGATING / RETRYING / CANCELING
```

Three decisions worth defending:

- **The goal carries only `job_id`.** The poses were resolved and frozen at booking (§B2).
  Re-sending them here could only introduce a disagreement between the job accepted and the
  job run.
- **`failed_leg` is a string, not the enum.** Requirement 7 says the result must *name* the
  leg that failed, and a string does that legibly in `ros2 action send_goal` output.
- **`nav2_recoveries` is surfaced.** It is Nav2's own count, and it is the most diagnostic
  number available — zero means a clean drive, fifteen means the robot is fighting something.
  It turns the blockage demonstration into a number instead of a vibe.

## A3. Why the interfaces are their own package

`rosidl_generate_interfaces` wants its own package. Mixing generation and consumers is
workable but fragile, and it would mean anyone depending on the contract drags in the node
that implements it. This is the standard split.

One build fact that catches people: **any package generating an action needs `action_msgs`**,
because the generated goal-handling services are built on its `GoalInfo`/`GoalStatus` types.

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RequestDelivery.srv"
  "action/ExecuteDelivery.action"
  DEPENDENCIES action_msgs
)
```

---

# B. Value types and the location table

## B1. `Pose2D`, `Leg`, `JobState`

**What.** The vocabulary, in [`types.hpp`](../ros2_ws/src/acadbot_courier/include/acadbot_courier/types.hpp).

**Why separate from the node.** So the pieces that only *describe* a delivery — the location
table, the markers — need not include the one that runs it. `location_table.cpp` compiles
without `rclcpp_action`, without `nav2_msgs`, without the courier interfaces. That is a
checkable statement that config parsing knows nothing about actions.

```cpp
struct Pose2D { double x{0.0}; double y{0.0}; double yaw{0.0}; };

enum class Leg { PICKUP, DROPOFF };

/// The spelling that goes into action feedback and into the result's
/// failed_leg field, defined once so the two cannot drift apart.
inline const char * to_string(Leg leg)
{
  return leg == Leg::PICKUP ? "pickup" : "dropoff";
}
```

`Pose2D` holds a yaw rather than a quaternion because that is how a location is written in
YAML; the conversion happens once, on the way into a Nav2 goal (§C12).

## B2. `Job` — and why the poses are frozen

**What.** A booked delivery.

**Why it stores poses and not just names.** This is the design decision most likely to be
asked about.

```cpp
struct Job
{
  std::string id;
  std::string pickup_name;
  std::string dropoff_name;
  Pose2D pickup;              // resolved once, at booking, and frozen
  Pose2D dropoff;
  JobState state{JobState::BOOKED};

  const std::string & name_of(Leg leg) const
  { return leg == Leg::PICKUP ? pickup_name : dropoff_name; }

  const Pose2D & pose_of(Leg leg) const
  { return leg == Leg::PICKUP ? pickup : dropoff; }
};
```

Both the names and the poses are kept: the names are what the requester asked for and what
feedback reports; the poses are what Nav2 is sent. **Resolving once, at booking, is the
point** — looking the names up again when the action runs would let an edit to `courier.yaml`
move the target of a job that had already been accepted. The requester would be told one thing
and the robot would do another.

`name_of()` / `pose_of()` keep `leg == PICKUP ? … : …` out of the state machine, which
otherwise branches on leg in six places.

**Terminal states are remembered, not discarded.** A finished job stays in the registry as
`SUCCEEDED` / `FAILED` / `CANCELED` so a replayed goal is rejected rather than delivering
twice (§C4).

## B3. `LocationTable::from_parameters` — discovery from parameter overrides

**What.** The name → pose table, in
[`location_table.cpp`](../ros2_ws/src/acadbot_courier/src/location_table.cpp). The only class
that knows a location has coordinates at all.

**Why it reads overrides rather than a declared list.** ROS 2 parameters have no map type, so
a name → pose table has to be encoded somehow. The obvious encoding is a list of names plus a
group per name — but that redundancy is one-directional: a name with no pose throws, while a
pose whose name was left out of the list is *silently ignored*. Add a room, forget to list it,
and you get an unusable location and no complaint.

Reading the **parameter overrides** — every key the YAML actually supplied, declared or not —
means a location exists by being written down and nothing else.

```cpp
const auto & overrides =
  node.get_node_parameters_interface()->get_parameter_overrides();

for (const auto & kv : overrides) {
  const std::string & key = kv.first;
  if (key.rfind(prefix, 0) != 0) { continue; }        // prefix is "locations."
  const std::string name = key.substr(prefix.size());

  std::vector<double> xyyaw;
  try {
    xyyaw = node.declare_parameter<std::vector<double>>(key);
  } catch (const rclcpp::exceptions::InvalidParameterTypeException &) {
    throw std::runtime_error(
      "location '" + name + "' must be a list of three numbers, [x, y, yaw].");
  }
  ...
}
```

**Why not `automatically_declare_parameters_from_overrides`,** which looks like the obvious
tool: it declares *every* key at construction, so the node's own
`declare_parameter("max_retries", 2)` would then throw `ParameterAlreadyDeclaredException`.
Every parameter in the node would need a `has_parameter` guard to fix one feature. Reading the
overrides touches nothing else.

A pleasant side effect: **duplicate names stop being possible.** The override map is keyed, so
YAML collapses them before the code sees them — one fewer failure mode to write and to
explain.

## B4. Refusing to start on a bad floor plan

**What.** Two failure paths, both fatal at construction.

**Why fatal.** A configuration mistake should stop the node at startup, not surface as a
failed delivery twenty minutes into a demo.

```cpp
// Refused rather than skipped. A short list is a typo, and skipping it
// would surface an hour later as "unknown location", sending you to the
// service handler when the actual fault is a forgotten yaw.
if (xyyaw.size() != 3) {
  throw std::runtime_error(
    "location '" + name + "' needs exactly 3 numbers [x, y, yaw], got " +
    std::to_string(xyyaw.size()) + ".");
}
```

The alternative — `if (coords.size() == 3) { use it }` — silently drops the malformed entry.
The delivery then comes back *"unknown location: reception"* and you go looking at the service
handler, when the real fault is a missing third number.

These messages only work because [`main.cpp`](../ros2_ws/src/acadbot_courier/src/main.cpp)
catches them:

```cpp
try {
  rclcpp::spin(std::make_shared<acadbot_courier::CourierServer>());
} catch (const std::exception & e) {
  RCLCPP_FATAL(rclcpp::get_logger("courier_server"), "%s", e.what());
  rclcpp::shutdown();
  return 1;
}
```

Without the catch, a missing coordinate prints `terminate called after throwing` and an abort
trace, which tells the reader nothing.

## B5. `known_names()` — the rejection message

**What.** The configured names as `"charging_dock, lab_bench, reception, storage"`.

**Why.** It goes into the refusal, so a mistyped location tells the requester what they could
have said instead of merely that they were wrong:

```
unknown pickup 'kitchen'; known locations are: charging_dock, lab_bench, reception, storage
```

---

# C. The courier node

All in [`courier_server.cpp`](../ros2_ws/src/acadbot_courier/src/courier_server.cpp), declared
in [`courier_server.hpp`](../ros2_ws/src/acadbot_courier/include/acadbot_courier/courier_server.hpp).

## C1. Construction

Five parameters, the location table, the service, the action server, the Nav2 client, the
marker publisher and its timer.

```cpp
goal_frame_      = declare_parameter<std::string>("goal_frame", "map");
max_retries_     = declare_parameter<int>("max_retries", 2);
retry_delay_     = declare_parameter<double>("retry_delay", 3.0);
feedback_period_ = declare_parameter<double>("feedback_period", 0.5);
leg_timeout_     = declare_parameter<double>("leg_timeout", 120.0);

locations_ = LocationTable::from_parameters(*this);   // throws on a bad floor plan
```

There is deliberately **no `nav2_wait_timeout`**. The service checks
`action_server_is_ready()` at booking time and refuses with a reason instead of waiting — which
turns "Nav2 isn't up" into requirement 2's *"cannot be served right now"* rather than a hang.

## C2. `handle_request` — the five refusals

**What.** The service handler. Validates, mints an id, returns. No driving happens here, which
is the entire reason this is a service.

**Why five.** Requirement 2 asks for unknown names and "cannot be served right now"; the rest
fall out of what a single-robot courier can honestly promise.

```cpp
const auto reject = [&](const std::string & why) {
    response->accepted = false;
    response->job_id = "";
    response->reason = why;
    RCLCPP_WARN(get_logger(), "rejected '%s' -> '%s': %s",
      request->pickup.c_str(), request->dropoff.c_str(), why.c_str());
  };

if (!active_job_id_.empty()) {
  reject("a delivery is already running (" + active_job_id_ + ")");   return; }

const auto pickup = locations_.find(request->pickup);
if (!pickup) {
  reject("unknown pickup '" + request->pickup + "'; known locations are: " +
    locations_.known_names());                                        return; }
// ... same for dropoff ...

if (request->pickup == request->dropoff) {
  reject("pickup and dropoff are both '" + request->pickup +
    "'; there is nothing to deliver");                                return; }

if (!nav_client_->action_server_is_ready()) {
  reject("navigation is not available: Nav2's navigate_to_pose action "
    "server is not up");                                              return; }
```

Order matters: the Nav2 check is last, so a typo is reported as a typo even when the stack is
still coming up.

## C3. Job ids

```cpp
std::ostringstream id;
id << "job_" << std::setw(4) << std::setfill('0') << ++next_job_number_;
```

A counter, not a UUID — **someone has to retype it live, in the room.** `job_0007` is a
defensible choice; `f47ac10b-58cc-...` is not, at a whiteboard.

## C4. `handle_goal` — refusing a goal

Three refusals, and the middle one is why finished jobs are kept:

```cpp
if (!active_job_id_.empty())            return REJECT;  // one job owns the robot
const auto it = jobs_.find(goal->job_id);
if (it == jobs_.end())                  return REJECT;  // never booked
if (it->second.state != JobState::BOOKED) return REJECT; // already ran -> would deliver twice
if (!nav_client_->action_server_is_ready()) return REJECT;
```

## C5. `handle_cancel` — and the finish that cannot happen here

**What.** Accepts the cancel and propagates it to Nav2.

**Why it defers.** Two subtleties, both learned by hitting them.

```cpp
cancel_reason_ = CancelReason::CLIENT_REQUEST;

if (phase_ == Phase::RETRY_WAIT) {
  // Nothing is driving and no Nav2 goal is in flight, so no result is
  // coming to finish on. The goal does not enter the CANCELING state until
  // this callback returns, though, so canceled() cannot be called from
  // here -- finish on the next spin instead.
  if (retry_timer_) { retry_timer_->cancel(); }
  retry_timer_ = create_wall_timer(1ms, [this]() {
      retry_timer_->cancel();
      finish_canceled();
    });
  return rclcpp_action::CancelResponse::ACCEPT;
}

phase_ = Phase::CANCELING;
if (nav_handle_) { nav_client_->async_cancel_goal(nav_handle_); }
// If nav_handle_ is null, Nav2 has not acknowledged the goal yet.
// on_nav_goal_response cancels it the moment the handle arrives.
```

- **Cancelling during the retry delay** has no Nav2 goal to cancel, so waiting for a Nav2
  result would hang forever. Measured on the real stack: **4.5 ms** to `CANCELED`.
- **Cancelling before Nav2 acknowledges** leaves no handle to cancel. The flag is set, and
  `on_nav_goal_response` cancels the handle the moment it arrives.

## C6. `CancelReason` — disambiguating a CANCELED result

**What.** A three-valued enum recorded when a cancel is *issued*.

**Why.** A `CANCELED` result from Nav2 is ambiguous on its own. It can mean the client
cancelled the delivery, that our own leg timeout gave up on this attempt, or that Nav2
abandoned the goal unilaterally. Those want three different outcomes, and a `bool` cannot tell
them apart.

```cpp
enum class CancelReason { NONE, CLIENT_REQUEST, LEG_TIMEOUT };
```

```cpp
case rclcpp_action::ResultCode::CANCELED:
  switch (cancel_reason_) {
    case CancelReason::CLIENT_REQUEST: finish_canceled();               return;
    case CancelReason::LEG_TIMEOUT:    cancel_reason_ = CancelReason::NONE;
                                       leg_failed("attempt ran past the ... leg timeout");
                                                                        return;
    case CancelReason::NONE:           leg_failed("Nav2 cancelled the goal on its own");
                                                                        return;
  }
```

`NONE` at the moment a `CANCELED` lands means Nav2 did it by itself — **a failure, not a
success, and definitely not a silent stop.**

## C7. `send_leg_goal` — the async pattern

Everything hangs off callbacks; nothing blocks. That is what makes a single-threaded executor
sufficient and every mutex unnecessary (see [`BACKGROUND.md` §8](BACKGROUND.md)).

```cpp
rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
opts.goal_response_callback =
  [this](NavGoalHandle::SharedPtr handle) {on_nav_goal_response(handle);};
opts.feedback_callback =
  [this](NavGoalHandle::SharedPtr h, auto feedback) {on_nav_feedback(h, feedback);};
opts.result_callback =
  [this](const NavGoalHandle::WrappedResult & result) {on_nav_result(result);};

nav_client_->async_send_goal(goal, opts);

leg_timeout_timer_ = create_wall_timer(
  std::chrono::duration<double>(leg_timeout_),
  std::bind(&CourierServer::on_leg_timeout, this));

publish_markers();     // move the highlight to the new target
```

## C8. A rejected goal is a failed leg

**What.** The one place this node deliberately differs from the course's `patrol_commander`.

```cpp
if (!handle) {
  // patrol_commander skips a rejected waypoint. A courier cannot: a leg
  // that was never driven is a leg that failed.
  leg_failed("Nav2 rejected the goal");
  return;
}
```

`patrol_commander.cpp:101` logs a warning and advances to the next waypoint. For a patrol that
is fine — the point is to keep patrolling. For a courier it is the beginning of reporting a
delivery that never happened.

## C9. `leg_failed` — retry, then stop lying

```cpp
if (attempt_ <= max_retries_) {
  phase_ = Phase::RETRY_WAIT;
  retry_timer_ = create_wall_timer(
    std::chrono::duration<double>(retry_delay_),
    std::bind(&CourierServer::on_retry_elapsed, this));
  return;
}

finish_failed(std::string("the ") + to_string(leg_) + " leg failed after " +
  std::to_string(attempt_) + " attempts: " + why);
```

Retries are counted **per leg** — a job that struggles at pickup and again at dropoff gets its
allowance at each — while the result reports `attempts_used` across both.

## C10. The three endings

Exactly one runs per accepted goal, and **success is reachable only from the dropoff leg
returning `SUCCEEDED`.**

```cpp
void CourierServer::finish_failed(const std::string & message)
{
  ...
  result->success = false;
  result->failed_leg = to_string(leg_);
  job.state = JobState::FAILED;
  // abort(), never succeed(). A delivery reported as done is a delivery that
  // happened.
  execute_handle_->abort(result);
  reset_to_idle();
}
```

`finish_canceled()` leaves `failed_leg` **empty** — a cancelled delivery is not a failed one.
Nothing went wrong; the requester changed their mind.

There is also a branch for arriving *as* the cancel lands:

```cpp
case rclcpp_action::ResultCode::SUCCEEDED:
  if (cancel_reason_ == CancelReason::CLIENT_REQUEST) {
    // Arrived just as the cancel landed. The requester asked us to stop,
    // so this is a cancelled delivery, not a successful one.
    finish_canceled();
    return;
  }
```

Observed live: a cancel issued with `distance_remaining: 0.19` — inside the 0.20 m goal
tolerance — still returned `CANCELED`.

## C11. Feedback on the courier's own clock

**What.** A timer at `feedback_period` (0.5 s) publishing cached values.

**Why not just forward Nav2's feedback.** Because then a stalled Nav2 becomes *silence*, and
silence is indistinguishable from a crashed node. Publishing on our own clock means a stalled
leg reads as a **frozen distance** — visible, diagnosable, and honest.

```cpp
feedback->leg = (leg_ == Leg::PICKUP) ? ExecuteDelivery::Feedback::LEG_PICKUP
                                      : ExecuteDelivery::Feedback::LEG_DROPOFF;
feedback->target_location    = job.name_of(leg_);
feedback->distance_remaining = last_distance_;      // cached from Nav2
feedback->attempt            = static_cast<uint8_t>(attempt_);
feedback->nav2_recoveries    = last_recoveries_;
feedback->state              = phase_name(phase_);
```

0.5 s rather than 1.0 s deliberately: the contract is "at least once a second", and publishing
at exactly the promised rate leaves no margin for a late tick.

## C12. `to_goal_pose`, and one naming trap

```cpp
out.header.frame_id = goal_frame_;
out.pose.position.x = pose.x;
out.pose.position.y = pose.y;
tf2::Quaternion q;
q.setRPY(0.0, 0.0, pose.yaw);
out.pose.orientation = tf2::toMsg(q);
```

The private helper for the phase string is called **`phase_name`, not `to_string`** — a member
of that name would hide the free `to_string(Leg)` and `to_string(JobState)` from `types.hpp`,
and every call to those inside the class would need qualifying.

---

# D. Seeing it

## D1. `make_location_markers`

**What.** A free function turning the table plus the active target into a `MarkerArray`
([`location_markers.cpp`](../ros2_ws/src/acadbot_courier/src/location_markers.cpp)).

**Why a free function.** It is a pure mapping with no state to own between calls. Taking the
active location as an argument keeps this file ignorant of legs, jobs and Nav2 entirely.

**Why arrows rather than spheres.** `yaw_goal_tolerance` is 0.25 rad, so a location's heading
is real work the robot must do. A sphere would hide the one thing about a location that is
easy to get wrong.

```cpp
arrow.ns = "courier_locations";
arrow.id = id;
arrow.type = visualization_msgs::msg::Marker::ARROW;
tf2::Quaternion q;  q.setRPY(0.0, 0.0, pose.yaw);
arrow.pose.orientation = tf2::toMsg(q);

// Zero lifetime means "until replaced". Same ns + id overwrites, so
// republishing on every change never accumulates stale arrows.
arrow.lifetime = rclcpp::Duration(0, 0);
```

Ids come from map iteration order, which is stable for a table fixed at startup — so each
location keeps its id across republishes and markers are *replaced* rather than stacked. That
is what makes republishing on every leg change free.

**This is diagnostic, not decorative.** Drawn over the global costmap, a location sitting
inside the inflation band is visible before it costs anyone a failed delivery — which is how
two of the four locations were found to be badly placed.

## D2. `courier.rviz`

`nav2.rviz` plus a `MarkerArray` display on `/courier_server/locations`, at **Transient Local**
durability to match the publisher — RViz's MarkerArray display defaults to volatile, and a
volatile subscriber gets no retained sample, so an RViz started after the node would show
nothing.

Two other differences: the local costmap starts **disabled** (a rolling 4 × 4 m window that
paints over the markers during a delivery), and **Publish Point** is enabled, which is how new
location coordinates get read off the map.

---

# E. Localization seeding

## E1. `initial_pose_seeder`

**What.** A second, small node
([`initial_pose_seeder.cpp`](../ros2_ws/src/acadbot_courier/src/initial_pose_seeder.cpp)) that
tells AMCL where the robot starts.

**Why it exists.** AMCL publishes nothing until told, and without `map → odom` the Nav2
costmaps cannot configure — the lifecycle manager times out, aborts the whole bringup, and
every goal afterwards is rejected. Requirement 9 says one command brings it up, so the
**2D Pose Estimate** click has to go.

**Why a separate executable rather than folding it into the courier.** A delivery service that
also sets up localization is harder to defend than a sixty-line node whose name says what it
does. It is also optional: drop it from the launch and click in RViz instead.

**Why it does not simply publish once.** Publishing at a fixed moment is unreliable — AMCL
looks up `base_footprint → odom` around the pose's timestamp, and early in a run its TF buffer
holds a fraction of a second. Five different fixed delays each failed differently. So the
seeder does not guess: it publishes, checks whether it worked, and repeats.

Three gates before it publishes at all:

```cpp
if (!odometry_alive())        return;   // nothing to place the pose against yet
if (scans_seen_ < min_scans_) return;   // AMCL cannot hold map->odom without a laser
publish_pose();
```

The scan gate is not obvious and cost a full debugging round: **AMCL only publishes `map→odom`
off the back of a filter update, and a filter update needs a scan.** Seed at sim time 0.4 and
it accepts the pose, emits a transform briefly, and then goes quiet.

And it leaves an already-localized robot alone:

```cpp
if (attempts_ == 0) {
  finish("map->odom is already advancing: something has set a pose "
         "already, so nothing to do.");
```

Without that guard, launching after you had set a good pose by hand would replace it with a
guess at the origin.

## E2. `transform_advancing()` — the check that took four attempts

**What.** The test for "is AMCL actually localized?"

**Why it is written this way.** Three earlier versions were wrong in the same way — each
*passed while the system was broken*.

| version | why it lied |
|---|---|
| does `map→odom` exist? | tf2 caches for ten seconds and `TimePointZero` means *latest available*, so a transform emitted once and abandoned kept answering yes |
| exists, three times in a row? | three checks a second apart all read the same dead transform |
| how old is it? | the age is measured in **simulated** time, which barely advances while Gazebo is starting — exactly the window the check exists to police |

The working version asks none of those. It asks whether the stamp is **changing**:

```cpp
bool transform_advancing()
{
  try {
    const auto tf =
      tf_buffer_->lookupTransform(map_frame_, odom_frame_, tf2::TimePointZero);
    const rclcpp::Time stamp(tf.header.stamp);
    const bool advancing = have_stamp_ && stamp > last_stamp_;
    last_stamp_ = stamp;
    have_stamp_ = true;
    return advancing;
  } catch (const tf2::TransformException &) {
    have_stamp_ = false;
    return false;
  }
}
```

A stamp that keeps moving means AMCL is still publishing; one frozen at last second's value
means it has stopped. **That holds whatever the clock is doing** — which is the property the
other three lacked.

It also gives up honestly, rather than spinning silently:

```cpp
RCLCPP_ERROR(get_logger(),
  "gave up after %.0fs: AMCL never published map->odom. Set the pose by "
  "hand with RViz's 2D Pose Estimate.", give_up_after_);
```

## E3. `amcl.yaml` — two deliberate absences

[`amcl.yaml`](../ros2_ws/src/acadbot_courier/config/amcl.yaml) carries the course's AMCL block
verbatim plus `transform_tolerance`. Both things it *omits* are documented in the file,
because both look like obvious things to add.

**No `map_server` block.** `nav2_bringup`'s `localization_launch.py` passes the map path to
that node itself as `parameters=[params_file, {'yaml_filename': map_yaml_file}]` — and a
`yaml_filename` written in the params file **wins over that override rather than losing to
it**, leaving `map_server` active but empty and AMCL stuck on `Waiting for map....` forever.

**No `set_initial_pose`.** It looks like exactly the right tool. AMCL applies it the instant it
activates — about 0.4 s after its process starts, before it has received a single `/clock`
message — so under `use_sim_time` the pose is stamped **t = 0**, a time for which no
`odom→base_footprint` has ever existed:

```
[amcl] Setting pose (0.000000): 0.000 0.000 0.000
```

Delaying the launch does not help: the race is between AMCL's own activation and its own first
clock message, so it moves along with it.

---

# F. Bringup and configuration

## F1. `courier.launch.py`

**What.** The one command
([`courier.launch.py`](../ros2_ws/src/acadbot_courier/launch/courier.launch.py)): simulation,
`map_server` + AMCL, the seeder, the Nav2 servers, RViz, the courier.

**Why it composes the stack rather than including `autonomy.launch.py`.** That file does not
forward `params_file`, and AMCL here needs this package's parameters while the Nav2 servers
keep the course's untouched `nav2_params.yaml`.

**Why the ordering is what it is.** Localization starts **with** the simulation — AMCL keeps
its own TF buffer, and that buffer only starts filling when the node does, so a late start
leaves it holding a single sample. Nav2 is held back behind `nav2_delay`, because its costmaps
cannot configure without `map→odom`, and a timed-out lifecycle transition aborts the entire
bringup.

The courier itself starts immediately, not with Nav2: until `navigate_to_pose` exists the
service refuses bookings with a reason saying so, which is the behaviour requirement 2 asks
for anyway.

## F2. `courier.yaml`

Everything tunable, in one file
([`courier.yaml`](../ros2_ws/src/acadbot_courier/config/courier.yaml)).

```yaml
courier_server:
  ros__parameters:
    use_sim_time: true
    goal_frame: "map"
    max_retries: 2          # per leg, so up to 3 attempts each
    retry_delay: 3.0
    feedback_period: 0.5
    leg_timeout: 120.0

    locations:
      reception:     [0.60, 4.20,  0.00]
      lab_bench:     [4.50, 4.20,  3.14]
      storage:       [5.50, 0.60,  0.00]
      charging_dock: [0.50, 2.00, -1.57]
```

The file carries a long comment block explaining that coordinates are in the **map** frame
(`map = gazebo + (3, 2)`), that locations must be at least `inflation_radius` (0.45 m) clear
of walls, and which two coordinates were moved after measurement and why. That is the one
place in this package where heavy comments earn their keep: someone editing coordinates needs
the frame explained, and needs to know why `lab_bench` is not in the obvious corner.

---

# G. The test harness (development only)

These live in the scratchpad, not the package. They are why requirements 3, 4, 6 and 7 could
be exercised in seconds rather than minutes, and why cancel could be tested against an
*abort on demand* instead of hoping the real robot misbehaved at the right moment.

## G1. `fake_nav2.py` — a stand-in `navigate_to_pose`

A `rclpy` action server that accepts `NavigateToPose` and behaves as instructed:

| `outcome` | behaviour |
|---|---|
| `succeed` | drive for `drive_time`, then SUCCEED |
| `abort` | drive, then ABORT — as if recoveries were exhausted |
| `hang` | accept and never finish, to trip `leg_timeout` |
| `reject` | refuse every goal |

It publishes decreasing `distance_remaining` and a nonzero `number_of_recoveries` so the
courier's feedback path is exercised too. No Gazebo, no map, no costmap — the full state
machine in about five seconds per scenario.

## G2. `cancel_client.py`

`ros2 action send_goal` cannot be scripted into cancelling, so this sends the goal, streams
feedback, cancels after N seconds and prints the terminal status. It is what produced the
literal evidence for requirement 6:

```
terminal status  : CANCELED
result.success   : False
result.failed_leg: ''
```

One trap worth recording: `self.handle` collides with a read-only property on rclpy's `Node`
and raises at construction. It is `self.goal_handle`.

## G3. Shell drivers

`test_courier.sh`, `test_cancel.sh`, `test_real.sh`, `test_cancel_real.sh` — bring up a stack,
book, run, cancel, and grep the outcome. Three separate bugs in *these* hid or faked results
and each cost a full run: a `grep -o "job_[0-9]*"` that matched zero digits and so also matched
the field name `job_id`; a `timeout 4` on `tf2_echo` that killed it a moment before it
resolved; and a `head -24` that truncated exactly the line before the terminal status.

**A check that truncates is a check that can lie** — and it lies in the direction of "no
result", which reads as failure. That lesson generalises well past this project.

---

## Which requirement each component serves

| Requirement | Components |
|---|---|
| 1 · service, immediate, identifier | A1, C2, C3 |
| 2 · reject with a reason | A1, B3, B5, C2, C4 |
| 3 · action for execution | A2, C4–C10 |
| 4 · feedback a human can follow | A2, C11 |
| 5 · drive with Nav2 | C7, C8, C12 |
| 6 · cancel means stop | A2, C5, C6, C10 |
| 7 · honest failure | B2, C8, C9, C10 |
| 8 · everything in YAML | B3, B4, F2, C1 |
| 9 · one command | E1, E2, E3, F1, D2 |
