# DVS ROS2

ROS 2 packages for working with Dynamic Vision Sensors (DVS) / Event Cameras.

## Overview

This repository contains ROS 2 packages for processing and visualizing event-based camera data from Dynamic Vision Sensors (DVS).

## Packages

### dvs_msgs

Message definitions for DVS event data.

**Messages:**
- `Event.msg` - Single DVS event (timestamp, x, y, polarity)
- `EventArray.msg` - Array of DVS events

### dvs_view_ros2

Visualization and processing tools for DVS event streams.

**Nodes:**
- `histogram_2D` - Creates multiple histogram-based visualizations from DVS event streams
- `time_surface` - Computes a Time Surface with exponential decay visualization

## Features

### histogram_2D

The `histogram_2D` node subscribes to DVS event streams and publishes 4 different visualization modes:

#### Published Topics

1. **`/dvs/histogram_no_polarity`** (sensor_msgs/Image)
   - Grayscale histogram of all events
   - Events accumulated without polarity distinction
   - Normalized to [0, 255] range

2. **`/dvs/histogram_with_polarity`** (sensor_msgs/Image)
   - Grayscale histogram considering event polarity
   - Positive events add, negative events subtract
   - Normalized with 127 as neutral gray (0-126 = negative dominant, 128-255 = positive dominant)

3. **`/dvs/histogram_binary`** (sensor_msgs/Image)
   - Binary visualization
   - Pixels are 255 where events occurred, 0 otherwise
   - No accumulation or normalization

4. **`/dvs/histogram_color_polarity`** (sensor_msgs/Image)
   - Color visualization separating polarities
   - **Red channel**: Positive polarity events (normalized)
   - **Blue channel**: Negative polarity events (normalized)
   - Pure red = only positive events, pure blue = only negative events, purple = both polarities

#### Subscribed Topics

- `/dvs/events` (dvs_msgs/EventArray) - Input DVS event stream

### time_surface

The `time_surface` node computes a Time Surface (Surface of Active Events). Each pixel's brightness represents how recently an event occurred, using exponential decay: `exp(-(t_now - t_last(x,y)) / tau)`.

#### Published Topics

- **`/dvs/time_surface`** (sensor_msgs/Image) - Grayscale image where bright = recent event, dark = old/no event

#### Subscribed Topics

- `/dvs/events` (dvs_msgs/EventArray) - Input DVS event stream

#### Parameters

- **`decay_time`** (`double`, default: `0.05`) - Decay constant tau in seconds. Typical values: 0.03 - 0.05.

## Installation

### Prerequisites

- ROS 2 (tested on Jazzy)
- OpenCV
- cv_bridge

### Install Dependencies

```bash
sudo apt update
sudo apt install ros-jazzy-cv-bridge ros-jazzy-image-transport
```

### Build

```bash
# Clone the repository
cd ~/ros2_dvs_ws/src
git clone git@github.com:renatto-tommasi/dvs_ros2.git .

# Build the packages
cd ~/ros2_dvs_ws
colcon build

# Source the workspace
source install/setup.bash
```

## Usage

### Running the Histogram Node

```bash
ros2 run dvs_view_ros2 histogram_2D
```

### Running the Time Surface Node

```bash
ros2 run dvs_view_ros2 time_surface --ros-args -p decay_time:=0.03
```

### Visualizing Output

Use `rqt_image_view` to visualize the different outputs:

```bash
ros2 run rqt_image_view rqt_image_view
```

Then select one of the following topics:
- `/dvs/histogram_no_polarity`
- `/dvs/histogram_with_polarity`
- `/dvs/histogram_binary`
- `/dvs/histogram_color_polarity`
- `/dvs/time_surface`

## Technical Details

### Event Accumulation (histogram_2D)

The node accumulates events over time windows (33ms / ~30 FPS) and creates different representations:

1. **No Polarity Mode**: Counts all events regardless of polarity
2. **With Polarity Mode**: Adds +1 for positive polarity, -1 for negative polarity
3. **Binary Mode**: Marks pixels where any event occurred
4. **Color Polarity Mode**: Separately counts and visualizes positive (red) and negative (blue) events

### Time Surface (time_surface)

Stores the timestamp of the last event at each pixel. At publish time, applies exponential decay relative to the current event time. Recent events appear bright, older events fade to black. The decay rate is controlled by the `decay_time` parameter.

### Normalization

- Event counts are stored as 32-bit signed integers to prevent overflow
- Normalization is applied during publishing to map counts to [0, 255] range
- Each visualization mode uses appropriate normalization for optimal contrast

## Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

## License

Apache-2.0

## Author

Renatto Tommasi
