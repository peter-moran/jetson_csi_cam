# jetson_csi_cam
A ROS package for easily connecting to CSI cameras from the Jetson TX1/TX2 via GStreamer.

This package consists of a ROS launch file specifically configured for getting `gscam` to connect to the Nvidia
multimedia library via gstreamer and to make it easy to set resolution, set framerate, and calibrate the camera.

---

# Setup
In order to get the `jetson_csi_cam` package to publish the video stream from your CSI camera, you will need to download this 
repository and install and build the `gscam` ROS package. Just follow the steps below and you should be good to go.

As a note, this package was tested on a Nvidia Jetson TX2 with L4T R27.1, ROS Kinetic, and the 
[Leopard Imaging IMX377CS](https://www.leopardimaging.com/LI-JETSON-KIT-IMX377CS-X.html) CSI camera.

If you'd like to learn more about `gscam`, check out their [ROS wiki page](http://wiki.ros.org/gscam) or their 
[Github repository](https://github.com/ros-drivers/gscam).

## Dependencies
This package is intended to run on ROS Kinetic but may work with older versions of ROS if the correct version of `gscam` is used
and that version still supports gstreamer-1.0.

## 1. Download jetson_csi_cam install
Clone this repository into you `catkin_workspace`.

```
cd ~/catkin_ws/srs
git clone https://github.com/peter-moran/jetson_csi_cam.git 
```

## 2. Install `gscam` with gstreamer-1.0 support
Clone `gscam` into your `catkin_workspace`.

```
cd ~/catkin_workspace/src
git clone https://github.com/ros-drivers/gscam.git
```

Then edit `./gscam/Makefile` and add the CMake flag `-DGSTREAMER_VERSION_1_x=On` to the first line of the file, so 
that it reads:

    EXTRA_CMAKE_FLAGS = -DUSE_ROSBUILD:BOOL=1 -DGSTREAMER_VERSION_1_x=On

While this is only necessary if you have both `gstreamer-0.1` and `gstreamer-1.0` installed simultaneously, it 
is good practice to include.

## 3. Build everything
Now we build and register `gscam` and `tegra_csi_cam` in ROS.

```
cd ~/catkin_ws
catkin_make
source ~/.bashrc
```

# Publishing and viewing the camera stream
To publish the camera stream in ROS, use:

```
roslaunch jetson_csi_cam jetson_csi_cam.launch
```
