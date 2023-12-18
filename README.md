# ROS4HRI helper library

A wrapper library for the [ROS4HRI](https://wiki.ros.org/hri) framework,
significantly easing accessing the robot's perceptions of surrounding humans.

It contains two packages:
- `hri`: the C++ implementation package
- `pyhri`: the Python implementation package, wrapping the C++ one with Pybind11

**Note: this branch only contains ROS 2 support. For ROS 1, check the `main` branch.**

## Example

For an example of usage, you can check the example:
- C++: `hri/src/example.cpp` _or_
- Python: `pyhri/pyhri/example.py`

They are installed by default and can be tested by executing:
- In a separate terminal:
  1. `apt install ros-humble-usb-cam`
  2. `ros2 run usb_cam usb_cam_node_exe`
- In a separate terminal:
  1. `apt install ros-humble-hri-face-detect`
  2. `ros2 launch hri_face_detect face_detect.launch.py`
- In a separate terminal, either:
  - C++: `ros2 run hri hri_show_faces`  _or_
  - Python: `ros2 run pyhri hri_show_faces`

Note that the Python implementation actually spawns two nodes.
