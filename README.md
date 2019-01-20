# Nvidia Jetson CSI camera launcher for ROS

<p align="left"><a href="http://petermoran.org/csi-cameras-on-tx2/"><img src="https://img.shields.io/badge/CSI_Cameras_on_the_TX2_(The_Easy_Way)-Learn_more_at_my_blog-blue.svg?style=social"></a></p> 

This ROS package makes it simple to use CSI cameras on the Nvidia Jetson TK1, TX1, or TX2 with ROS via gstreamer and the Nvidia multimedia API. This is done by properly configuring [`gscam`](http://wiki.ros.org/gscam) to work with the Nvidia hardware.

**Features**

* Control resolution and framerate.
* Camera calibration support.
* Works efficiently, allowing for high resolution and/or high fps video by taking advantage of gstreamer and the Nvidia multimedia API.
* Multi camera support.

---

# Installation

In order to get started, you will need to download the `jetson_csi_cam`  repository as well as its dependencies. Just follow the steps below and you should be good to go.

If you'd like to learn more about `gscam`, check out their [ROS wiki page](http://wiki.ros.org/gscam) or their [Github repository](https://github.com/ros-drivers/gscam).

**Note:** This package was tested on a Nvidia Jetson TX2 with L4T R27.1, ROS Kinetic, and the [Leopard Imaging IMX377CS](https://www.leopardimaging.com/LI-JETSON-KIT-IMX377CS-X.html) CSI camera.

## Dependencies

For the purpose of this guide, we will assume you already have:

* Gstreamer-1.0 and the Nvidia multimedia API (typically installed by Jetpack)
* ROS Kinetic
  * Older versions of ROS may work, provided that a version of  `gscam` that supports gstreamer-1.0 is available for that ROS version, but this is untested.
* `gscam` with gstreamer-1.0 support.
  * The following steps will show how to build `gscam` from source to support this, so don't worry about it yet.

With these dependencies accounted for, lets get everything installed.

## 1. Download `jetson_csi_cam`

Clone this repository into you `catkin_workspace`.

```
cd ~/catkin_workspace/src
git clone https://github.com/peter-moran/jetson_csi_cam.git 
```

## 2. Install `gscam` with gstreamer-1.0 support

Clone `gscam` into your `catkin_workspace`.

```
cd ~/catkin_workspace/src
git clone https://github.com/ros-drivers/gscam.git
```

Then edit `./gscam/Makefile` and add the CMake flag `-DGSTREAMER_VERSION_1_x=On` to the first line of the file, so that it reads:

    EXTRA_CMAKE_FLAGS = -DUSE_ROSBUILD:BOOL=1 -DGSTREAMER_VERSION_1_x=On

While this flag is only necessary if you have both `gstreamer-0.1` and `gstreamer-1.0` installed simultaneously, it is good practice to include.

## 3. Build everything

Now we build and register `gscam` and `jetson_csi_cam` in ROS.

```
cd ~/catkin_workspace
catkin_make
source ~/.bashrc
```

At this point everything should be ready to go.

---

# Usage

> **TL;DR:** To publish the camera stream to the ROS topic `/csi_cam_0/image_raw`, use this command in the terminal:
>
> ```
> roslaunch jetson_csi_cam jetson_csi_cam.launch width:=<image width> height:=<image height> fps:=<desired framerate>
> ```
> If you have another camera on your Jetson TX2, to publish the other camera stream to the ROS topic `/csi_cam_1/image_raw`, use this command in the terminal:
>
> ```
> roslaunch jetson_csi_cam jetson_csi_cam.launch sensor_id:=1 width:=<image width> height:=<image height> fps:=<desired framerate>
> ```
> If you would like to learn more of the details, read ahead.

## Capturing Video

### Turning on the video stream

To publish your camera's video to ROS (using the default settings) execute the following:

```
roslaunch jetson_csi_cam jetson_csi_cam.launch
```

> **Wait, where is the video?** This launch file only *publishes* the video to ROS, making it available for other programs to use. This is because we don't want to view the video every time we use the camera (eg the computer may be processing it first). Thus we use separate programs to view it. I'll discuss this in a later section, but if you can't wait, run `rqt_image_view` in a new terminal to see the video.

You can confirm the video is running by entering `rostopic list` in the terminal. You should be able to see the  `/csi_cam/image_raw` topic (aka your video) along with a bunch of other topics with similar names -- unless you changed the `camera_name` argument from the default.

### Setting video options

Most of the time we'll want to use settings other than the defaults. We can easily change these by passing command line arguments to `roslaunch`. For example, if I want the camera to run at 4k resolution at 15 fps, I would use the following:

```
roslaunch jetson_csi_cam jetson_csi_cam.launch width:=3840 height:=2160 fps:=15
```

In other words, to set any of the arguments use the `<arg_name>:=<arg_value>` options for `roslaunch`.

#### Accepted arguments for `jetson_csi_cam.launch`

* `sensor_id` -- The sensor id of each camera
* `width` -- Image Width
* `height` -- Image Height
* `fps` -- Desired framerate. True framerate may not reach this if set too high.
* `cam_name` -- The name of the camera (corrsponding to the camera info).
* `frame_id` -- The TF frame ID for the camera.
* `sync_sink` -- Whether to synchronize the app sink. Setting this to false may resolve problems with sub-par framerates.

## Testing your video stream

### View the video

To view the video, simply run `rqt_image_view` in a new terminal. A window will pop up and the video should be inside. You may need to go to the pulldown in the top left to choose your camera's video topic, by default you want to use  `/csi_cam/image_raw`.

### Calculate true framerate

To check the true framerate of your video you can use the `rostopic hz` tool, which shows the frequency any topic is published at.

```
rostopic hz /csi_cam/image_raw
```

When the true framerate is below the framerate you asked for, it is either because the system cannot keep up or the camera does not have that setting. If you are using the Nvidia Jetson TX2, you can get higher resolution video at higher true FPS by switching to a [higher power mode](http://www.jetsonhacks.com/2017/03/25/nvpmodel-nvidia-jetson-tx2-development-kit/). It can also be good to dial back your requested FPS to be close or slightly above to your true FPS.

## Camera Calibration

The `jetson_csi_cam` package is set up to make camera calibration very simple. To calibrate your camera, all you need to do is follow the [monocular camera calibration guide](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration) on the ROS wiki, with the following notes:

* As the guide states, you'll need a printout of a chessboard. If you want something quick, use this [chessboard for 8.5"x11" paper](http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/pattern.pdf) with an 8x6 grid. Please note: while there are nine by seven *squares*, we use a size of 8x6 because we are counting the *internal vertices*. You will need to measure the square size yourself since printing will slightly distort the chessboard size, but the squares should be 30mm on each side.

* In Step 2, make sure to start the camera via `roslaunch` as discussed above.

* In Step 3, make sure to set your `image` and `camera` arguments correctly as below, also *make sure that the chessboard size and square size are set correctly* for your chessboard.

  ```
  rosrun camera_calibration cameracalibrator.py --size 8x6 --square <square size in meters> image:=/csi_cam/image_raw camera:=/csi_cam
  ```

After following this guide and clicking **COMMIT** (as it tells you to do), your calibration data should be automatically published with your video under the topic `/csi_cam/camera_info`. Most other ROS packages needing this information will automatically recognize it.
