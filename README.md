# ROS2 DDS to DIS Bridge for Ship Entity States

## Overview

This prototype demonstrates a complete ROS2 DDS to DIS (Distributed Interactive Simulation) bridge system that converts ship entity state updates into standard DIS packets. The system simulates multiple ships with realistic movement patterns and broadcasts their positions as DIS EntityStatePdu packets, enabling integration with DIS-compatible military simulations and training systems.

## System Architecture

The bridge consists of four main components:

1. **Ship Entity State Publisher** - Simulates ship movements and publishes position updates
2. **DDS to DIS Bridge** - Subscribes to ship updates and converts them to DIS packets
3. **Custom ROS2 Message Definition** - Defines ship entity state structure
4. **DIS Receiver Test Tool** - Validates DIS packet transmission

```
┌─────────────────┐    ROS2 DDS     ┌─────────────────┐    UDP/DIS    ┌─────────────────┐
│  Ship Publisher │ ──────────────> │ DDS-DIS Bridge  │ ────────────> │ DIS Simulations │
│                 │  ShipEntityState│                 │ EntityStatePdu│                 │
└─────────────────┘                 └─────────────────┘               └─────────────────┘
```

## Dependencies and Installation

### System Requirements
- Ubuntu 20.04+ or similar Linux distribution
- ROS2 Humble (or compatible version)
- C++17 compatible compiler
- CMake 3.8+

### Required Packages

#### ROS2 Dependencies
```bash
# Install ROS2 Humble (if not already installed)
sudo apt update
sudo apt install ros-humble-desktop

# Install additional ROS2 development tools
sudo apt install ros-humble-ament-cmake
sudo apt install ros-humble-rosidl-default-generators
sudo apt install ros-humble-rosidl-default-runtime
```

#### Build Dependencies
```bash
# Essential build tools
sudo apt install build-essential cmake pkg-config

# Networking libraries (for DIS UDP transmission)
sudo apt install libc6-dev

# Git (to clone open-dis-cpp if needed)
sudo apt install git
```

### Project Setup

1. **Create ROS2 Workspace** (if you don't have one):
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

2. **Clone or Copy Project Files**:
   - Ensure the `open-dis-cpp` library is in your `src` directory
   - Copy the `my_cpp_package` with all the implemented files

3. **Source ROS2 Environment**:
```bash
source /opt/ros/humble/setup.bash
```

4. **Build the Project**:
```bash
cd ~/ros2_ws
colcon build --packages-select my_cpp_package
```

5. **Source the Workspace**:
```bash
source install/setup.bash
```

## File Structure

```
~/ros2_ws/src/my_cpp_package/
├── CMakeLists.txt                 # Build configuration
├── package.xml                    # ROS2 package definition
├── msg/
│   └── ShipEntityState.msg       # Custom message definition
└── src/
    ├── ship_publisher.cpp         # Ship position simulator
    ├── dds_dis_bridge.cpp         # DDS to DIS converter
    └── dis_receiver_test.cpp      # DIS packet validator
```

## Key Components

### 1. ShipEntityState Message Definition
```
uint16 entity_id
string entity_name  
float64 latitude
float64 longitude
float64 altitude
float64 heading
float64 pitch
float64 roll
float64 velocity_x
float64 velocity_y
float64 velocity_z
uint8 force_id
uint32 timestamp
```

### 2. Ship Publisher Features
- Simulates 3 ships: USS Enterprise, USS Nimitz (Blue Force), Red Destroyer
- Realistic movement patterns with different headings and speeds
- Dead reckoning position updates every 500ms
- Starting positions around Norfolk, VA area

### 3. DDS to DIS Bridge Features
- **Configurable networking**: Loopback (127.0.0.1) by default for safety
- **Efficient transmission**: Only sends DIS packets when position changes >10m
- **Standard compliance**: Generates valid DIS 6 EntityStatePdu packets
- **Entity type mapping**: Properly categorizes ships as naval platforms

### 4. Configuration Parameters
```bash
# Network configuration
ros2 param set /dds_dis_bridge dis_address "127.0.0.1"  # Default: loopback
ros2 param set /dds_dis_bridge dis_port 3000             # Default: 3000
ros2 param set /dds_dis_bridge use_multicast false       # Default: false

# Transmission efficiency
ros2 param set /dds_dis_bridge position_threshold_meters 10.0  # Default: 10m
```

## Running the System

### Basic Operation
Open three terminals and run each component:

**Terminal 1: Start Ship Publisher**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run my_cpp_package ship_publisher
```

**Terminal 2: Start DDS to DIS Bridge**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run my_cpp_package dds_dis_bridge
```

**Terminal 3: Monitor DIS Packets**
```bash
cd ~/ros2_ws
source install/setup.bash
./install/my_cpp_package/lib/my_cpp_package/dis_receiver_test
```

### Configuration for Network Testing
To broadcast to actual network (use with caution):
```bash
ros2 run my_cpp_package dds_dis_bridge --ros-args \
  -p dis_address:="224.0.0.1" \
  -p use_multicast:=true \
  -p position_threshold_meters:=5.0
```

## Expected Output

### Ship Publisher
```
[INFO] [ship_publisher]: Published ship 1001: lat=36.897022, lon=-76.022972
[INFO] [ship_publisher]: Published ship 1002: lat=36.900000, lon=-75.999955
[INFO] [ship_publisher]: Published ship 2001: lat=36.999987, lon=-75.800017
```

### DDS to DIS Bridge
```
[INFO] [dds_dis_bridge]: UDP socket configured for DIS transmission to 127.0.0.1:3000 (multicast: disabled)
[INFO] [dds_dis_bridge]: Received ship entity state for ID 1001: lat=36.897494, lon=-76.022382
[INFO] [dds_dis_bridge]: Sent DIS EntityStatePdu for entity 1001 (144 bytes)
```

### DIS Receiver
```
Successfully parsed EntityStatePdu:
  Entity ID: 1001
  Force ID: 1
  Position: (-6.7678e+06, 4.10743e+06, 0)
  Orientation: (0.785398, 0, 0)
```

## Technical Details

### DIS Packet Structure
- **Protocol Version**: 6 (DIS 2012)
- **PDU Type**: EntityStatePdu
- **Entity Types**: Naval surface platforms (ships)
- **Coordinate System**: Cartesian coordinates converted from lat/lon
- **Dead Reckoning**: DRM_FVW (Fixed velocity, world coordinates)

### Performance Characteristics
- **Update Rate**: Ship positions updated at 2Hz
- **DIS Transmission**: Only on significant position changes (configurable threshold)
- **Packet Size**: 144 bytes per EntityStatePdu
- **Latency**: <1ms bridge processing time

## Safety Features

1. **Loopback Default**: All DIS traffic uses 127.0.0.1 by default
2. **Configurable Addressing**: Easy to switch between loopback and network modes
3. **Multicast Control**: Multicast disabled by default
4. **Position Filtering**: Prevents excessive packet transmission

## Potential Extensions

1. **Additional Entity Types**: Aircraft, ground vehicles, munitions
2. **Real Data Integration**: Connect to actual ship tracking systems
3. **Bi-directional Bridge**: Convert DIS packets back to ROS2 messages
4. **Advanced Dead Reckoning**: More sophisticated movement prediction
5. **Entity State Interpolation**: Smooth position updates between transmissions

## Troubleshooting

### Build Issues
```bash
# Clean build
rm -rf build install log
colcon build --packages-select my_cpp_package
```

### Network Issues
```bash
# Check if port is available
netstat -ulnp | grep 3000

# Verify loopback interface
ping 127.0.0.1
```

### Runtime Issues
```bash
# Check ROS2 topics
ros2 topic list
ros2 topic echo /ship_entity_states

# Verify parameters
ros2 param list /dds_dis_bridge
```

This prototype provides a solid foundation for integrating ROS2-based autonomous systems with legacy DIS simulation environments, enabling realistic multi-platform training scenarios and system interoperability testing.