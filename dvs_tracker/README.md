# dvs_tracker

A ROS 2 package for real-time corner detection on Dynamic Vision Sensor (DVS) event streams using the Arc* algorithm.

## Overview

This package implements a corner detector node that processes asynchronous events from a DVS camera and identifies corner features. Detected corners are published as an annotated image overlaid on an exponential-decay time surface.

## Algorithm: Arc*

The Arc* algorithm detects corners by analyzing the temporal structure of events on discretized circles around each incoming event. Unlike frame-based detectors (e.g., Harris, FAST), Arc* operates directly on the asynchronous event stream, achieving microsecond-level temporal resolution.

### Pipeline

1. **Surface of Active Events (SAE):** Each incoming event updates a per-polarity timestamp surface (`sae_[0]` for negative, `sae_[1]` for positive polarity). This separates light-to-dark and dark-to-light transitions, preventing cross-polarity interference on edges.

2. **Refractory Filter:** Before processing, redundant events are suppressed. If the same pixel fires the same polarity within `filter_threshold` (default 50ms), the event is rejected. A polarity-flip at the same pixel overrides this filter, allowing real edge transitions through.

3. **Border Rejection:** Events within 4 pixels of the sensor boundary are discarded to prevent out-of-bounds access on the larger circle.

4. **Two-Circle Cascade:** Each event is tested on two Bresenham circles:
   - **Small circle** (radius 3, 16 pixels)
   - **Large circle** (radius 4, 20 pixels)

   The small circle is tested first as a cheap pre-filter. Only events that pass the small circle test proceed to the large circle test. Both must pass for a corner to be declared.

5. **Greedy Bidirectional Arc Expansion:** For each circle, the algorithm:
   - Finds the pixel with the **newest timestamp** on the circle.
   - Expands bidirectionally (clockwise and counterclockwise), always extending toward whichever neighbor has the larger timestamp.
   - **Phase 1 (forced):** Expands unconditionally for `min_thresh` iterations to reach the minimum arc size.
   - **Phase 2 (conditional):** Continues expanding, but only increments the "newest segment size" if the newly added pixel's timestamp is >= the running minimum of the segment. This ensures temporal contiguity.

6. **Arc Classification:** A corner is detected if the newest segment size falls within the acceptance range:
   - Small circle: `[3, 6]` or inverted `[10, 13]`
   - Large circle: `[4, 8]` or inverted `[12, 16]`

   Edges produce arcs of ~half the circle size, which falls outside both ranges. The inverted arc range detects corners where the "old" region is the minority.

### Key Design Decisions

- **No absolute time threshold:** Unlike simpler approaches that threshold `delta_t < k`, Arc* uses purely relative timestamp ordering. This makes it adaptive to varying event rates without parameter tuning.
- **Hardcoded Bresenham circles:** The circle pixel coordinates are precomputed lookup tables (not trigonometric sampling), ensuring consistent discretization and zero runtime overhead.
- **Polarity separation:** Processing positive and negative events on separate surfaces is critical for avoiding false detections along edges.

## Node: corner_detector

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/dvs/events` | `dvs_msgs/EventArray` | Raw DVS event stream |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/dvs/corner_image` | `sensor_msgs/Image` | Exponential-decay time surface with detected corners overlaid in red |

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `decay_time` | double | 0.1 | Time constant (seconds) for the exponential decay visualization |
| `filter_threshold` | double | 0.05 | Refractory period (seconds) for same-polarity event suppression |

### Sensor Dimensions

The sensor width and height are automatically read from the first `EventArray` message's `width` and `height` fields. No manual configuration is needed.

## Usage

```bash
# Build
colcon build --packages-select dvs_tracker

# Launch
ros2 launch dvs_tracker corner_detection.launch.py

# View output
ros2 run rqt_image_view rqt_image_view /dvs/corner_image
```

## References

1. I. Alzugaray and M. Chli, **"Asynchronous Corner Detection and Tracking for Event Cameras in Real-Time,"** IEEE Robotics and Automation Letters, vol. 3, no. 4, pp. 3177-3184, 2018. doi: [10.1109/LRA.2018.2849882](https://doi.org/10.1109/LRA.2018.2849882)

2. I. Alzugaray and M. Chli, **"ACE: An Efficient Asynchronous Corner Tracker for Event Cameras,"** International Conference on 3D Vision (3DV), 2018. doi: [10.1109/3DV.2018.00080](https://doi.org/10.1109/3DV.2018.00080)

3. Reference implementation: [ialzugaray/arc_star_ros](https://github.com/ialzugaray/arc_star_ros)
