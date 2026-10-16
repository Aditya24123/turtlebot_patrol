# TurtleBot Patrol

ROS 2 Jazzy package for repeated waypoint patrol, reactive laser obstacle avoidance,
and a persistent emergency-stop service that can be triggered and reset.

The included live demonstration uses a lightweight 2D differential-drive TurtleBot3
Burger model. The controller receives real ROS `LaserScan` and `Odometry` messages,
publishes `Twist`, and handles real `std_srvs/SetBool` requests. Simulator, controller
and dashboard are separate ROS nodes communicating through DDS.

## What it does

- Cycles four odometry-frame waypoints and counts completed patrol laps.
- Stops forward motion before an obstacle, rotates toward the clearer side, then
  follows a short bypass before returning to waypoint tracking.
- Uses separate stop/clear thresholds to avoid rapid switching near an obstacle.
- Commands zero motion for stale/missing odometry, invalid/incomplete laser data,
  or an obstacle inside the hard clearance.
- Latches an emergency stop, persists it atomically to disk, and preserves it across
  controller restarts. Reset is refused while sensors are stale or clearance is unsafe.
- Includes a command watchdog in the simulator: missing commands stop its motion.

## Setup

### Windows (the tested environment)

Install [Pixi](https://pixi.sh/) from its official distribution, then:

```powershell
git clone https://github.com/Aditya24123/turtlebot_patrol.git
cd turtlebot_patrol
pixi install
pixi run colcon build --packages-select turtlebot_patrol
```

The committed `pixi.lock` records the dependency environment. No system-wide ROS or
Linux installation is needed. For a short Windows installation path, use `C:\ros_ws`.

In each terminal, enter the environment and source the package:

```powershell
pixi shell
cmd /k install\setup.bat
```

Commands below run in that sourced command prompt. The Python simulation dashboard
uses Tk, supplied with the Pixi Python distribution.

### Ubuntu 24.04 / ROS 2 Jazzy

Install ROS 2 Jazzy and `python3-colcon-common-extensions`, `python3-tk`,
`python3-pytest`, and `python3-yaml` through the official ROS/Ubuntu instructions.
Clone the repository into a workspace, then:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select turtlebot_patrol
source install/setup.bash
```

The package is portable Python; Ubuntu and physical TurtleBot operation have not
been exercised in this submission. Windows Jazzy is the verified runtime.

## Launch

```bash
ros2 launch turtlebot_patrol simulation.launch.py
```

For a headless simulation:

```bash
ros2 launch turtlebot_patrol simulation.launch.py gui:=false
```

To run the controller against an already running TurtleBot driver or an external
simulator that publishes `/scan` and `/odom` and accepts `/cmd_vel`:

```bash
ros2 launch turtlebot_patrol patrol.launch.py
```

Set waypoints in `src/turtlebot_patrol/config/patrol.yaml` for that environment's
odometry frame before launching. A robot that expects `TwistStamped` needs a message
adapter; this package publishes `geometry_msgs/Twist`.

## Emergency stop and reset

Trigger from a separate sourced terminal:

```bash
ros2 service call /emergency_stop std_srvs/srv/SetBool "{data: true}"
```

Reset and resume:

```bash
ros2 service call /emergency_stop std_srvs/srv/SetBool "{data: false}"
```

The dashboard buttons call that same service. A successful trigger immediately
publishes a zero Twist; the timer continues publishing zero while latched. Reset
returns `success: false` if either sensor is stale or the hard clearance is violated.
The latch defaults to `~/.ros/turtlebot_patrol_estop.json`. An unreadable latch file
fails stopped. Use an explicit reset after fixing sensor or storage failures.

This is a software stop, not a certified hardware emergency-stop circuit. Real robots
also need a base command watchdog, a physical stop and a single authoritative
`cmd_vel` source or command mux. Competing velocity publishers are outside this
node's control. Do not treat process shutdown as a guaranteed hardware stop.

## Topics and architecture

```text
TurtleBot driver OR 2D simulator
  /scan (LaserScan) ----> patrol controller ----> /cmd_vel (Twist)
  /odom (Odometry) ----> waypoint tracking       |
                                               v
                                       driver / simulator
  /emergency_stop (SetBool) --> persistent stop latch
  /patrol/status + /sim/telemetry --> live dashboard
```

Sensor subscriptions use ROS sensor-data QoS. Status and commands use reliable QoS.
The controller uses a monotonic clock for sensor-receipt freshness. The default
sensor timeout is 0.6 s; control commands publish at 20 Hz. All topic names are relative,
so the controller can be placed in a namespace. Use the same namespace/remappings
for its driver and service client.

## Configuration

| Parameter | Default | Purpose |
|---|---:|---|
| `linear_speed` | 0.20 m/s | Cruise speed, validated at or below 0.22 m/s |
| `angular_speed` | 0.8 rad/s | Rotation limit |
| `stop_distance` | 0.55 m | Enter avoidance, measured from laser origin |
| `clear_distance` | 0.80 m | Leave obstacle-facing rotation |
| `hard_distance` | 0.24 m | Stop all motion inside this clearance |
| `sensor_timeout` | 0.6 s | Maximum receipt age for scan and odometry |
| `waypoint_tolerance` | 0.25 m | Goal arrival radius |
| `waypoints` | Four x,y pairs | Repeated patrol in odom coordinates |

The simulator uses an 8 by 6 metre room, four circular obstacles and a conservative
0.12 m robot radius. Laser readings are ray-cast at 1-degree intervals over 360 degrees.
It models planar kinematics and command timeout; it does not model wheel slip,
acceleration, friction, camera imagery or Gazebo physics. Collision-guard hits are
displayed separately, so a blocked simulator motion cannot masquerade as avoidance.

## Usage and debugging

```bash
ros2 topic echo /patrol/status
ros2 topic echo /cmd_vel
ros2 service type /emergency_stop
ros2 node list
```

States are `SENSOR_FAULT`, `PATROL`, `AVOID`, `BYPASS`, `BLOCKED`, and `ESTOP`.
`BLOCKED` will hold still until adequate clearance returns. This reactive planner
does not claim to solve arbitrary mazes or globally unreachable waypoints.

## Tests and evidence

From the repository root, after build and sourcing:

```bash
python -m pytest tests -v
python tests/closed_loop.py
```

Ten controller scenarios cover clear patrol, left avoidance, right avoidance,
hard blockage, stale/missing sensors, invalid scans, avoidance hysteresis/bypass,
stop/reset guards, cyclic waypoints and parameter validation. Live ROS integration
tests exercise topics, service requests and restart persistence separately.
`evidence/` contains the actual run results.

The accelerated closed-loop test uses the same robot/laser geometry as the live
simulator, replays two full patrol laps and checks collision-guard hits. It is a
deterministic simulation check, not a physical robot certification.

## Screenshots

The following are captured from the running ROS dashboard, not generated mockups.

![Live autonomous patrol](https://raw.githubusercontent.com/Aditya24123/turtlebot_patrol/main/docs/images/patrol.png)

![Obstacle avoidance](https://raw.githubusercontent.com/Aditya24123/turtlebot_patrol/main/docs/images/avoidance.png)

![Emergency-stop service latched](https://raw.githubusercontent.com/Aditya24123/turtlebot_patrol/main/docs/images/emergency_stop.png)

## Sources and technical decisions

- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) supplies rclpy, DDS and standard interfaces.
- [RoboStack](https://robostack.github.io/GettingStarted.html) supplies Windows ROS binaries.
- [TurtleBot3 specifications](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/)
  inform the Burger speed envelope; the simplified simulator footprint is explicitly conservative.
- Python keeps the controller testable independently of ROS. The ROS wrapper owns
  transport, persistence and service responses; motion decisions live in one core.
- A short bypass avoids repeatedly turning straight back toward the same obstacle.
- AI assistance was used to implement, document and test this package. Test results
  describe observed runs and limitations; no physical-hardware testing is claimed.

License: MIT. Dependencies retain their own licenses.
