# NAO Position Recorder

A ROS2 package for recording NAO robot movements and saving them as `.pos` files.

## Overview

This package allows you to manually move the NAO robot's joints and record keyframes that can be saved as `.pos` files compatible with the `nao_pos` package.

## Features

- **Real-time joint state monitoring**: Subscribes to `/joint_states` topic
- **Keyframe recording**: Record the current robot position at any time
- **Automatic duration calculation**: Calculates time between keyframes
- **Easy file export**: Saves recordings in standard `.pos` format
- **Multiple recordings**: Creates timestamped files to avoid overwriting

## Installation

The package is already in your workspace. Build it with:

```bash
cd ~/UNI/robots/ros2/nao_ws
colcon build --packages-select nao_pos_recorder
source install/setup.bash
```

## Usage

### 1. Start the recorder node

For recording **arm movements**:

```bash
ros2 launch nao_pos_recorder pos_recorder.launch.py movement_type:=arms
```

For recording **leg movements**:

```bash
ros2 launch nao_pos_recorder pos_recorder.launch.py movement_type:=legs
```

For recording **full body movements** (arms + legs):

```bash
ros2 launch nao_pos_recorder pos_recorder.launch.py movement_type:=all
```

**CAUTION**: When using `movement_type:=all`, all leg joints will be relaxed. Ensure the robot is properly supported (sitting or held) to prevent falling!

The node will automatically set the stiffness of the selected joints to 0, making them easy to move manually while keeping the other joints stable.

Or run the node directly:

```bash
ros2 run nao_pos_recorder pos_recorder --ros-args -p movement_type:=arms
```

### 2. Move the robot and record keyframes

Manually move the NAO robot through the desired motion sequence, recording keyframes at important positions:

```bash
# Move robot to position 1
ros2 service call /record_keyframe std_srvs/srv/Empty

# Move robot to position 2
ros2 service call /record_keyframe std_srvs/srv/Empty

# Move robot to position 3
ros2 service call /record_keyframe std_srvs/srv/Empty
# ... etc
```

### 3. Set the total duration (optional)

Define how long the entire movement should take (in **milliseconds**):

```bash
ros2 param set /pos_recorder_node total_duration 5000
```

This will divide 5000ms (5 seconds) evenly among all keyframes. If you don't set a duration, it will use the default (1000ms per keyframe).

### 4. Save the recording

When you're done recording all keyframes:

```bash
ros2 service call /save_pos_file std_srvs/srv/Trigger
```

This will save the recording to `~/nao_recordings/recorded_YYYYMMDD_HHMMSS.pos`

### 5. Clear keyframes (optional)

To start a new recording without restarting the node:

```bash
ros2 service call /clear_keyframes std_srvs/srv/Empty
```

## Services

- `/record_keyframe` (std_srvs/srv/Empty): Record current joint positions as a keyframe
- `/save_pos_file` (std_srvs/srv/Trigger): Save all recorded keyframes to a .pos file
- `/clear_keyframes` (std_srvs/srv/Empty): Clear all recorded keyframes

## Parameters

- `joint_states_topic` (string, default: `/joint_states`): Topic to subscribe for joint states
- `stiffness_topic` (string, default: `/effectors/joint_stiffnesses`): Topic to publish joint stiffness commands
- `output_dir` (string, default: `~/nao_recordings`): Directory where .pos files will be saved
- `default_duration` (int, default: `2000`): Default duration in **milliseconds** per keyframe when total_duration is not set
- `movement_type` (string, default: `arms`): Type of movement - `"arms"`, `"legs"`, or `"all"`. Sets the corresponding joints to zero stiffness
- `total_duration` (int, optional): Total duration in **milliseconds** for the entire sequence. Set this parameter before calling save_pos_file to distribute the duration evenly across all keyframes

### Safety Parameters

- `max_velocity` (float, default: `120.0`): Maximum allowed joint velocity in degrees per second. Prevents overcurrent by limiting movement speed
- `min_duration` (int, default: `2000`): Minimum duration in **milliseconds** per keyframe to ensure safe movements
- `enable_safety_checks` (bool, default: `true`): Enable/disable safety validation before saving files
- `auto_adjust_duration` (bool, default: `true`): Automatically increase duration if movements are too fast for safety

**Safety features prevent fuse damage from overcurrent:**

- Validates joint velocities before saving
- Warns about unsafe movements
- Can automatically adjust durations for safety
- Enforces minimum duration between keyframes

## Example Workflow

```bash
# Terminal 1: Start the recorder for arm movements
ros2 launch nao_pos_recorder pos_recorder.launch.py movement_type:=arms

# Terminal 2: Record a wave sequence
# The arms are now relaxed and easy to move manually
# Move robot arm to starting position
ros2 service call /record_keyframe std_srvs/srv/Empty

# Move robot with arm up
ros2 service call /record_keyframe std_srvs/srv/Empty

# Move robot with arm to the side
ros2 service call /record_keyframe std_srvs/srv/Empty

# Move robot back to starting position
ros2 service call /record_keyframe std_srvs/srv/Empty

# Set total duration to 4 seconds (4000 milliseconds)
ros2 param set /pos_recorder_node total_duration 4000

# Save the recording
ros2 service call /save_pos_file std_srvs/srv/Trigger
```

The resulting `.pos` file will have 4 keyframes, each with 1000ms (1 second) duration (4000ms / 4 keyframes).

**Note**: With safety checks enabled (default), the duration may be automatically increased if 1000ms per keyframe is too fast for the recorded movements.

## Output Format

The generated `.pos` files follow the standard format used by `nao_pos_server`:

```
  HY    HP    LSP   LSR   LEY   LER   LWY   LHYP  LHR   LHP   LKP   LAP   LAR   RHR   RHP   RKP   RAP   RAR   RSP   RSR   REY   RER   RWY   LH    RH    DUR
! 0     5     90    10    -15   0     0     -     -     -25   50    -25   0     0     -25   50    -25   0     90    -10   15    0     0     0     0     1000
! -     -     80    20    -     -     -     -     -     -     -     -     -     -     -     -     -     -     80    -20   -     -     -     -     -     2000
```

## Joint Abbreviations

The `.pos` file format uses abbreviated joint names. Here's the complete mapping:

### Head

- **HY** - HeadYaw (left/right rotation)
- **HP** - HeadPitch (up/down tilt)

### Left Arm

- **LSP** - LShoulderPitch (forward/backward)
- **LSR** - LShoulderRoll (in/out from body)
- **LEY** - LElbowYaw (elbow rotation)
- **LER** - LElbowRoll (elbow bend)
- **LWY** - LWristYaw (wrist rotation)
- **LH** - LHand (0=open, 100=closed)

### Right Arm

- **RSP** - RShoulderPitch (forward/backward)
- **RSR** - RShoulderRoll (in/out from body)
- **REY** - RElbowYaw (elbow rotation)
- **RER** - RElbowRoll (elbow bend)
- **RWY** - RWristYaw (wrist rotation)
- **RH** - RHand (0=open, 100=closed)

### Left Leg

- **LHYP** - LHipYawPitch (hip rotation, shared between both legs)
- **LHR** - LHipRoll (lateral hip movement)
- **LHP** - LHipPitch (forward/backward hip)
- **LKP** - LKneePitch (knee bend)
- **LAP** - LAnklePitch (ankle forward/backward)
- **LAR** - LAnkleRoll (ankle lateral tilt)

### Right Leg

- **RHR** - RHipRoll (lateral hip movement)
- **RHP** - RHipPitch (forward/backward hip)
- **RKP** - RKneePitch (knee bend)
- **RAP** - RAnklePitch (ankle forward/backward)
- **RAR** - RAnkleRoll (ankle lateral tilt)

### Duration

- **DUR** - Duration in milliseconds to reach this keyframe from the previous one

**Note**: Values are in degrees. Use `-` to indicate a joint should not be commanded (will maintain its current position).

## Tips

- **Smooth movements**: Wait appropriate time between recordings for natural motion
- **Test recordings**: Copy generated `.pos` files to the `nao_pos_server/pos/` directory to test them
- **Joint safety**: Ensure the robot is in a stable position when recording
- **Multiple takes**: Use different recording sessions to create variations

## License

Apache-2.0
