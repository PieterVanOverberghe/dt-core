# My Packages - Custom Lane Following

This package contains a custom lane_following.launch file that starts the essential nodes for lane following.

## Files

- `launch/lane_following.launch` - Custom launch file for lane following
- `package.xml` - Package manifest
- `CMakeLists.txt` - CMake build configuration

## Usage

To launch the custom lane following:

```bash
roslaunch my_packages lane_following.launch
```

## What it does

The launch file starts the following nodes:

1. **decoder_node** - Converts compressed images to raw images
2. **anti_instagram_node** - Performs color correction
3. **line_detector_node** - Detects line segments
4. **ground_projection_node** - Projects lines to ground plane
5. **lane_filter_node** - Estimates lane pose
6. **lane_controller_node** - Controls the robot to follow the lane

## Troubleshooting

If the bot doesn't start driving:

1. Check if all nodes are running: `rosnode list`
2. Check if topics are being published: `rostopic list`
3. Check if the camera is publishing: `rostopic hz /duckiebot05/camera_node/image/compressed`
4. Check if the decoder_node is running: `ps aux | grep decoder_node`

## Notes

- This launch file does NOT use the FSM (Finite State Machine), so nodes are always active
- The camera topic defaults to `camera_node`, but can be changed via the `camera_topic` argument
- All nodes are launched in the vehicle namespace (e.g., `/duckiebot05/`)
