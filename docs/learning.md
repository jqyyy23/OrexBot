# OrexBot ROS2 Learning Path
**Goal:** Understand the full software stack and get RViz showing the arm in 2 days.
**Audience:** New to ROS. Goal is to understand it, not shortcut it.

---

## Mental Model First: What Is ROS2?

Think of a busy restaurant kitchen. You have a chef, a dishwasher, a waiter, and a manager — each person has one job and they communicate by calling out to each other. If the dishwasher quits, you hire a new one. The rest of the kitchen keeps running.

ROS2 is the same idea but for a robot. Each "person" is called a **node** — a small program with one job. They shout information onto named channels called **topics**, and anyone who cares listens. You never write one giant program. You write small nodes that each do one thing, and wire them together.

```
[servo_driver node] --shouts onto--> /joint_states topic
                                            |
                                [robot_state_publisher node]  <-- listens, does math
                                            |
                                    [rviz2 node]  <-- listens, draws the arm
```

---

## Glossary — Every Term You'll Hit

### Core ROS2 Concepts

**Node**
One small running program with one job. Examples: "read servo positions", "plan a movement", "draw the 3D model". Nodes run independently — if one crashes, the others keep going.

**Topic**
A named public channel. Any node can broadcast onto it; any node can tune in. Think of it like a radio station — the station has a name (e.g. `/joint_states`), the broadcaster doesn't care who's listening, and the listeners don't care who's broadcasting. Data flows continuously, one-way.

**Message**
The format of data on a topic. Like a form with specific fields. The `/joint_states` topic uses the `JointState` message format, which has fields: joint names, positions, velocities. Every broadcaster and listener on that topic must speak the same format.

**Service**
A two-way, one-time call between nodes. One node asks a question, another answers it, done. Like calling the front desk — you ask "is table 5 ready?", they say yes or no, conversation over. Used for things like "turn servo torque ON".

**Action**
Like a service, but for tasks that take time. You request something, get updates while it's happening, then a final result. Like ordering food — you place the order, the kitchen gives you updates ("cooking now... almost ready..."), then it arrives. Used for "move arm to this position" — takes a few seconds with live progress.

**Parameter**
A setting for a node that you can change without rewriting code. Like a dial on a machine. You set it in a config file or a launch file. Example: the serial port name `/dev/hiwonder_linker` is a parameter — you don't hardcode it into the program.

**Launch File**
A script that starts multiple nodes at once and sets their parameters. Like a stage manager calling "lights, camera, action" — one command, everything starts in the right order with the right settings. Your launch file is `launch/orexbot.launch.py`.

**Package**
A folder of related code, configs, and resources bundled together. Has a manifest file that says what it is and what it depends on. Your package is `orex_description`. Think of it like a self-contained appliance — it has everything it needs to do its job.

**Workspace**
The project root folder. When you build the project, compiled outputs land in `build/` and `install/` inside it. Your workspace is `/Users/eddiejung/Desktop/robo/OrexBot`.

**colcon**
The build tool. You run `colcon build` from the workspace root. It finds every package in `src/`, compiles them, and puts the results in `install/`. Like running a factory — raw materials go in, finished parts come out.

**source / setup.bash**
After building, you run `source install/setup.bash`. This tells your terminal where all the installed packages live so you can actually use them. Think of it as punching in at work — until you do it, the system doesn't know you exist. Every new terminal window needs this.

**ament**
The underlying build system that colcon uses. You'll see `ament_cmake` in config files — it just means "this package uses CMake to compile". You don't need to understand it deeply.

---

### Visualization Tools

**RViz2**
A 3D viewer. It does NOT control the robot — it only *displays* what's happening based on data from topics. Think of it as the control room monitor that shows you the robot's state. You can see the 3D model, joint angles, coordinate axes, planned paths. It's passive — watching, not doing.

**Gazebo**
A physics simulator. The robot exists in a fake world with real gravity, weight, and collisions. You run your entire software stack against this fake robot instead of the real one. Like a flight simulator for pilots — you practice dangerous maneuvers without crashing a real plane. Not needed until the real arm works.

**MoveIt**
A motion planning system. You tell it "put the hand at this point in space" and it figures out the angles every single joint needs to be at to get there, checks that the arm won't hit anything along the way, and generates a smooth movement plan. You don't write the math — it's pre-built. You configure it to know your arm's dimensions and limits.

---

### Robot Description

**URDF** (Unified Robot Description Format)
An XML file that is the complete blueprint of the robot's body. It answers: what parts exist, how are they connected, what do they look like, how heavy are they, how far can each joint bend. Every software tool — RViz, Gazebo, MoveIt — reads this file to understand the robot.

Your URDF: `src/orex_description/urdf/OrexBot.urdf` — auto-generated from Onshape.

**Xacro** (XML Macros)
A smarter version of URDF that lets you use variables and templates to avoid copy-pasting. Like the difference between writing the same measurement 40 times vs. storing it in one variable and referencing it everywhere. The `.xacro` file is the source; the `.urdf` is the output after processing it.

**Link**
One rigid physical part of the robot in the URDF. The upper arm is a link. The forearm is a link. Has a shape (the STL mesh file), mass, and center of mass.

**Joint**
The connection between two links. Defines where they meet, what axis they rotate around, and how far. Types you'll see:
- `revolute` — rotates with limits (elbow bending, shoulder rotating)
- `continuous` — rotates without limits (a wheel)
- `fixed` — doesn't move (two parts bolted together permanently)

**TF / Transform Tree (tf2)**
Every part of the robot has its own coordinate system — a tiny set of X,Y,Z axes centered on it. The "transform tree" tracks how all those coordinate systems relate to each other as joints move. When the elbow bends, everything below it shifts. `robot_state_publisher` continuously calculates this and broadcasts it. RViz reads it to know exactly where in 3D space to draw each mesh.

Imagine a marionette — when you tilt the torso, the arms follow. The transform tree is the math that tracks all of that automatically.

**robot_state_publisher**
A built-in node. You give it the URDF and a live stream of joint angles. It continuously works out the full transform tree and broadcasts it. Every visualization and planning tool depends on this running.

**joint_state_publisher_gui**
A built-in node that opens a window with one slider per joint. Dragging a slider sends fake joint angles into the system. Used for testing the 3D model visually without any real hardware connected.

---

### Hardware Control

**ros2_control**
The hardware control framework. It runs a tight loop (100 times per second) that does three things on every tick:
1. Ask the hardware "what position are you at right now?"
2. Let the controller decide "what position should it be at?"
3. Tell the hardware "go here"

Think of it as a thermostat loop — constantly checking the temperature, constantly adjusting the heater.

**Hardware Interface**
The piece of code you write in C++ that knows how to talk to your specific hardware — in this case, the Hiwonder servos over the serial bus. It has two methods: `read()` (ask servos where they are) and `write()` (tell servos where to go). `ros2_control` calls these at 100Hz. It must be C++ because Python is too slow and unpredictable for a 100Hz loop.

**Controller**
A pre-built node that sits on top of `ros2_control` and implements a specific control strategy. You configure them with YAML — no coding. Examples:
- `joint_state_broadcaster` — just reads and publishes current joint positions
- `joint_trajectory_controller` — follows a time-series of positions (used by MoveIt)

---

## The Full Software Stack

From your command down to the physical servo:

```
You say: "move hand to this position"
        |
        v
MoveIt  — figures out joint angles, plans a safe path
        |
        v
joint_trajectory_controller  — sends the path as a series of timed positions
        |
        v
ros2_control framework  — runs the 100Hz control loop
        |
        v
Hardware Interface (C++)  — your code, talks to serial bus
        |
        v
/dev/hiwonder_linker  — the USB serial port on your computer
        |
        v
Hiwonder HX-30HM / HX-10HM servos  — the physical joints
```

Separately, for visualization:
```
ros2_control  --publishes joint positions--> /joint_states topic
                                                    |
                                       robot_state_publisher
                                       (computes transform tree)
                                                    |
                                            RViz2 (draws the arm)
```

---

## What's Already In This Repo

### `src/orex_description/` — the only ROS2 package
- `urdf/OrexBot.urdf` — the full robot blueprint, from Onshape
- `meshes/` — 60+ STL files, one per physical part
- `launch/orexbot.launch.py` — starts robot_state_publisher + slider GUI + RViz2
- `package.xml` — package manifest, lists dependencies
- `CMakeLists.txt` — tells colcon what files to install

### `src/rx1 (ROS1)/` — reference only, do not build
The complete working stack from the original RX1 project in the old ROS1 framework. Read this as a blueprint for what you need to build in ROS2. The `COLCON_IGNORE` file inside tells the build tool to skip it entirely.

### `src/test_hardware/` and `src/Move Arm/Left/`
Plain Python scripts. No ROS at all. Use these to test servos directly on the bench. The serial communication code here is what you'll eventually wrap inside a ROS2 node.

---

## Day 1 Plan: Understand and Launch RViz

### Morning — Install and Orient (~2 hours)

**Step 1: Verify ROS2 is installed**
```bash
printenv ROS_DISTRO
# Should print: jazzy
```
If nothing prints, ROS2 Jazzy needs to be installed first.
Install guide: https://docs.ros.org/en/jazzy/Installation/macOS-Install-Binary.html (macOS)

**Step 2: Source ROS2 in your terminal**
```bash
source /opt/ros/jazzy/setup.bash
```
Add to `~/.zshrc` so you don't have to do it every time:
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.zshrc
```

**Step 3: Install package dependencies**
```bash
cd /Users/eddiejung/Desktop/robo/OrexBot
rosdep install --from-paths src --ignore-src -r -y
```
This reads every `package.xml` and auto-installs anything missing.

**Step 4: Build the workspace**
```bash
colcon build
```
You'll see `build/`, `install/`, `log/` folders appear on success.

**Step 5: Source the workspace**
```bash
source install/setup.bash
```

**Step 6: Launch**
```bash
ros2 launch orex_description orexbot.launch.py
```
Two windows should open: a slider panel and RViz2.

**Step 7: Configure RViz to show the robot**
1. In the top-left "Fixed Frame" dropdown, type `root`
2. Click "Add" (bottom left) → "By display type" → `RobotModel`
3. In the RobotModel panel, set "Description Topic" to `/robot_description`
4. The arm appears. Drag sliders — joints should move.

---

### Afternoon — Read the URDF (~2 hours)

Open `src/orex_description/urdf/OrexBot.urdf` and read through it.

You're looking for the pattern:
- `<link name="upper_arm">` — a part, references an STL file
- `<joint name="elbow" type="revolute">` — a connection, has `<parent>` and `<child>`, has `<limit>`

The tree starts at one root link and branches outward joint by joint to the fingertips.

Run this to generate a visual diagram of the full tree:
```bash
ros2 run tf2_tools view_frames
# Opens a PDF showing every frame and how they connect
```

Inspection commands — use these constantly:
```bash
ros2 node list                    # what's currently running
ros2 topic list                   # what channels exist
ros2 topic echo /joint_states     # print live joint angle data
ros2 topic hz /joint_states       # how fast is data being published
```

---

### Evening — C++ Basics You Actually Need (~2 hours)

You need about 10 concepts. Not the whole language.

**1. `#include` — importing**
Brings in another file's definitions. Like opening a manual before using a tool.
```cpp
#include "hardware_interface/system_interface.hpp"
```

**2. Namespaces — grouping**
`hardware_interface::SystemInterface` means: inside the `hardware_interface` group, find `SystemInterface`. The `::` is just a separator, like a folder path.

**3. Classes and inheritance**
```cpp
// "OrexHardware is a type of SystemInterface, with extra stuff"
class OrexHardware : public hardware_interface::SystemInterface {
public:
    // your methods go here
};
```

**4. `std::vector<double>` — a list of numbers**
Used to store joint positions (one number per joint).
```cpp
std::vector<double> joint_positions_;
// The underscore at the end = convention for "this belongs to the class"
```

**5. `std::string` — text**
Exactly what it sounds like. Used for names, port paths.
```cpp
std::string port_name_ = "/dev/hiwonder_linker";
```

**6. Return types — declared upfront**
In C++, every function says what type it returns before its name.
```cpp
// "this function returns a CallbackReturn"
hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
```
`override` means: this method exists in the parent class and I'm replacing it with my version.

**7. `const` and `&` — read-only and by-reference**
`const` = "I promise not to change this". `&` = "don't copy it, just point to the original" (faster for large data).
```cpp
hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
```

**8. Smart pointers (`std::shared_ptr`)**
A safe way to hold a reference to an object. It automatically cleans up when nothing uses the object anymore. You don't manage memory manually.
```cpp
std::shared_ptr<SerialClient> serial_;
```

**9. CMakeLists.txt — the build recipe**
Tells colcon: here are my source files, here are the libraries I need, here's what to install. You mostly copy an existing example and change the package name and filenames. You don't need to understand it deeply.

**10. Logging**
```cpp
RCLCPP_INFO(get_logger(), "Port opened: %s", port_name_.c_str());
RCLCPP_ERROR(get_logger(), "Failed to connect");
```
These print to the terminal with timestamps and severity labels.

That's it. Everything else in the hardware interface is following the ROS2 API pattern.

---

## Day 2 Plan: Fix Issues + Write Your First Node

### Morning — Debug URDF/RViz Problems (~2 hours)

The URDF is auto-generated from Onshape. Common issues:

**Problem: "Fixed Frame [root] does not exist"**
The launch file expects a link named exactly `root`. Check:
```bash
grep 'name="root"' "src/orex_description/urdf/OrexBot.urdf"
```
If nothing comes back, the root link has a different name. Either rename it in the URDF or update the launch file's `--child-frame-id` argument to match.

**Problem: Parts appear floating in wrong positions**
Joint `<origin>` coordinates from Onshape may be off. You'll need to cross-check them against physical measurements. See `docs/ros2_learnings_log.md` section 6 for how to fix this.

**Problem: Sliders appear but arm doesn't move**
Check `ros2 topic echo /joint_states` — if it's empty, the slider GUI isn't publishing. Make sure both nodes are running: `ros2 node list`.

---

### Afternoon — Write Your First Python ROS2 Node (~2 hours)

Before writing the C++ hardware interface, write a Python node that reads one servo and publishes its position to a topic. This proves the full data pipeline works.

Create `src/orex_description/scripts/servo_reader.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial  # pip install pyserial

class ServoReader(Node):
    def __init__(self):
        super().__init__('servo_reader')
        # Create a publisher: broadcast on /joint_states, keep last 10 messages
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        # Call read_and_publish 50 times per second
        self.timer = self.create_timer(0.02, self.read_and_publish)
        # Open the serial port to the servo bus
        self.ser = serial.Serial('/dev/hiwonder_linker', 1000000, timeout=0.1)
        self.get_logger().info('ServoReader started')

    def read_and_publish(self):
        # Read position from servo (copy logic from src/test_hardware/scan_servos.py)
        position = 0.0  # replace with actual serial read

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['shoulder_joint']   # must match joint name in URDF
        msg.position = [position]
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ServoReader()
    rclpy.spin(node)  # spin = "keep running until Ctrl+C"
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run it:
```bash
python3 src/Orex\ \(ROS2\)/orex_description/scripts/servo_reader.py
```

Then in another terminal (sourced):
```bash
ros2 topic echo /joint_states
```
You should see live position data printing. If RViz is also open, the arm will move as the servo moves.

---

### Evening — Study the Reference C++ Code (~1 hour)

Read `src/rx1 (ROS1)/rx1_motor/src/rx1_motor.cpp`. This is the ROS1 equivalent of what you'll write in C++. Notice:
- It opens a serial port at the top
- It has a timer that fires at regular intervals
- The timer callback reads servo positions and publishes them
- It subscribes to a command topic and writes those commands to servos

The ROS2 `orex_hardware` C++ plugin follows the exact same logic, just structured as `read()` and `write()` methods instead of timer callbacks.

---

## Packages You Will Build

| Package | Language | What It Does | Status |
|---|---|---|---|
| `orex_description` | URDF + XML | Robot blueprint, meshes, initial launch | EXISTS |
| `orex_hardware` | C++ | Reads/writes Hiwonder servos at 100Hz | Week 2 |
| `orex_bringup` | Python | One launch file that starts everything | Week 2 |
| `orex_moveit_config` | YAML + Python | Motion planning setup | Week 3 |

Gazebo simulation comes after the real arm works reliably. It is not on the critical path.

---

## Key Commands Cheat Sheet

```bash
# Build
colcon build
colcon build --packages-select orex_description  # one package only

# Source (run after every build, in every new terminal)
source install/setup.bash

# Launch
ros2 launch orex_description orexbot.launch.py

# Inspect what's running
ros2 node list
ros2 topic list
ros2 topic echo /joint_states
ros2 topic hz /joint_states
ros2 run tf2_tools view_frames           # generate TF tree diagram as PDF

# Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y
```

---

## MoveIt and Gazebo — Plain English

**MoveIt** is the arm's navigation system. You give it a destination in 3D space ("put the hand 30cm in front of the shoulder, 10cm to the right"). It works out every joint angle needed to get there, makes sure the arm won't hit itself or the table along the way, then hands a movement plan to `ros2_control` to execute. You configure it with YAML files that describe your arm's geometry and limits. You don't write the math.

**Gazebo** is a stunt double for the real robot. It simulates physics — gravity, mass, motor friction — so you can run your full software stack against a virtual arm. Useful for testing moves that might break something on the real hardware. Think of it as a crash test facility. Skip it until the real arm is working.
