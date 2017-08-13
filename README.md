# jetson_csi_cam
A ROS package for easily connecting to CSI cameras from the Jetson TX1/TX2 via GStreamer.

# Setup
## Dependencies
### GSCam with gstreamer-1.0 support
First, clone `gscam` ([ROS documentation](http://wiki.ros.org/gscam), [Github page](https://github.com/ros-drivers/gscam)) 
into your `catkin_workspace`.

```
cd ~/catkin_workspace/src
git clone https://github.com/ros-drivers/gscam.git
```

Then edit `./gscam/Makefile` and add the CMake flag `-DGSTREAMER_VERSION_1_x=On` to the first line of the file, so 
that it reads:

    EXTRA_CMAKE_FLAGS = -DUSE_ROSBUILD:BOOL=1 -DGSTREAMER_VERSION_1_x=On

While this is only necessary if you have both `gstreamer-0.1` and `gstreamer-1.0` installed simultaneously, it 
is good practice. We will build this package in the 'Build' step.

### jetson_csi_cam install
Now that we have gscam with gstreamer-1.0 support, clone this repository into you `catkin_workspace`.

```
cd ~/catkin_ws/srs
git clone https://github.com/peter-moran/jetson_csi_cam.git 
```
### Building
Now we build and register `gscam` and `tegra_csi_cam` in ROS.
```
cd ~/catkin_ws
catkin_make
source ~/.bashrc
```
# Usage
## Publishing and viewing the camera stream
To publish the camera stream, use:
```
roslaunch jetson_csi_cam jetson_csi_cam.launch
```
