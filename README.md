# DVS ROS2

ROS 2 packages for working with Dynamic Vision Sensors (DVS) / Event Cameras.

## Packages

### dvs_msgs

Message definitions for DVS event data.

- `Event.msg` - Single DVS event (timestamp, x, y, polarity)
- `EventArray.msg` - Array of DVS events with image dimensions

### dvs_view_ros2

Visualization and processing tools for DVS event streams. Includes nodes for 2D histogram views, time surfaces, and 3D voxel grid representations. See the [package README](dvs_view_ros2/README.md) for detailed node documentation and parameters.

### dvs_tracker

Real-time corner detection on DVS event streams using the Arc* algorithm. Implements greedy bidirectional arc expansion on two Bresenham circles with polarity-separated surfaces and a refractory filter. See the [package README](dvs_tracker/README.md) for algorithm details and references.

## Installation

### Prerequisites

- ROS 2 (tested on Jazzy)
- OpenCV
- cv_bridge

### Build

```bash
cd ~/ros2_dvs_ws/src
git clone git@github.com:renatto-tommasi/dvs_ros2.git .

cd ~/ros2_dvs_ws
colcon build
source install/setup.bash
```

## License

Apache-2.0

## Author

Renatto Tommasi
