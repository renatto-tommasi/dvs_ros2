# dvs_view_ros2

ROS2 package for visualizing Dynamic Vision Sensor (DVS) event data.

## Nodes

### histogram_2D

Accumulates DVS events over a fixed time window and publishes multiple histogram visualizations.

**Subscriptions:**
- `dvs/events` (`dvs_msgs/msg/EventArray`)

**Publications:**
- `dvs/histogram_no_polarity` (`sensor_msgs/msg/Image`) - Event count regardless of polarity
- `dvs/histogram_with_polarity` (`sensor_msgs/msg/Image`) - Event count where positive adds and negative subtracts
- `dvs/histogram_binary` (`sensor_msgs/msg/Image`) - Binary image (white if any event occurred)
- `dvs/histogram_color_polarity` (`sensor_msgs/msg/Image`) - Color image (red = positive, blue = negative)

**Usage:**
```bash
ros2 run dvs_view_ros2 histogram_2D
```

### time_surface

Computes a Time Surface (Surface of Active Events) where each pixel's brightness represents how recently an event occurred, using exponential decay.

**Subscriptions:**
- `dvs/events` (`dvs_msgs/msg/EventArray`)

**Publications:**
- `dvs/time_surface` (`sensor_msgs/msg/Image`) - Grayscale image where bright = recent event, dark = old/no event

**Parameters:**
- `decay_time` (`double`, default: `0.05`) - Decay constant tau in seconds. Controls how fast the trail fades. Typical values: 0.03 - 0.05.

**Usage:**
```bash
ros2 run dvs_view_ros2 time_surface --ros-args -p decay_time:=0.03
```
