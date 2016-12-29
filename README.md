
# REBIVO
## Realtime Edge Based Inertial Visual Odometry for a Monocular Camera

Tarrio, J. J., & Pedre, S. (2015). Realtime Edge-Based Visual Odometry
for a Monocular Camera. In Proceedings of the IEEE International Conference
on Computer Vision (pp. 702-710).

REBVO tracks a camera in Realtime using edges. The system is split in 2 components.
An on-board part (rebvo itself) doing all the processing and sending data over UDP 
and an OpenGL visualizer. 

Introductory video: https://youtu.be/7pn29iGklgI

### System requirements

In ubuntu and most linux dist this libraries can be downloaded directly from the
 repos, except for TooN.

-- C++11

-- Linux, X11, v4l2

-- OpenGL development libraries (GL,GLU,glut)

-- TooN 2.2 mathematical library (http://www.edwardrosten.com/cvd/toon.html - ZIP provided in the repo)

-- Lapack (for advanced TooN functions)

-- LibAV (Video Codecs)

-- LibGD (Image managment)

-- Optionally NE10 for ARM Neon optimizations

### Compiling

REBVO has been developed using QT creator, so a project file is provided in each of the
components folder, (rebvo and visualizer). Also two makefiles are provided on each 
of the folders, for amd64 and arm architectures. The first can easily be edited for 
x86.

#### Compile using NE10

The system can use the NE10 for neon simd support. In order to use it, the
correspondent line has to be uncommented in the arm makefile. 

### Imu Integration

IMU measurements are integrated trough the ImuGrabber class, that implements a circular buffer, timestamp search utilities and inter-frame integration.

For IMU data in a csv dataset style the ImuGraber supports a file loadding function (Config IMUMode=2).
For a custom interface IMU a class similar to archimu should be written (Config IMUMode=1), that reads the IMU in a separate thread and pushes timestamped data using IMUGrabber::pushData().

#### Imu Fussion

IMU fussion is done using a two stage bayesian filter. Sensor noise covariances should be set for optimal performance. Scale estimation reponse dinamics should be tunned ussing

Scale estimation response dynamics should be tuned for a trade-off between precision and robustness.

An initial guess for Giro Bias should be provided for highly biased sensors, an initial automatic guess could be used if the system is started still.

### Configuring the system

A GlobalConfig text file can be found in each of the folders. This should be tuned to
your camera and network configuration. All the configuration options has to be present on the files
in order for the system to start. The structure of the file is self explanatory.

Two files are provided as an example, GlobalConfig (and standart config to run onboard Quad) and GlobalConfig_EuRoC (for testing the EuRoC dataset).

#### Basic configs in rebvo GlobalConfig

The things you have to configure in order for the system to work.

-- CameraDevice: the v4l2 camera device or dataset camera file and directory

-- ZfX,Y: Camera focal length

-- PPx,y: Camera principal point

-- ImageWidth,Height: size to use

-- Distortion parameters (if enabled)

-- FPS: Frames per Seconds

-- VideoNetHost,Port: IP,Port of the host to send data to

-- SetAffinity flag,CamaraTn: if the flag is set to 1, the system will use the pthread_setaffinity_np() call
		to distribute the threads according to the CamaraT1..3 configuration.
		This numbers have to point to existing processors or the call wil fail,
		and this setting usually require superuser privileges.
		Setting affinity and distributing the threads can increase speed.

-- DetectorThresh: This is a fixed threshold to the edge detector only used
	 	   if autothresh is off (DetectorAutoGain = 0)
		   Is a camera dependent parameter so it should be tunned for
		   best performance. 

-- ReferencePoints: If auto thresh is on (DetectorAutoGain>0), number of KL
		   points to use. Ussual values are 5000 for 320x240 and 
		   15000 for 640x480 

-- DetectorMax,MinThresh: Limits on threshold tunning, to prevent excessive 
		   noise on images with no edges. Similar to DetectorThresh,
		   camera dependant parameters.



#### Basic configs in visualizer GlobalConfig

ImageSize, Focal Length and Principal Point should match the config in rebvo.

### System Usage

Just run the two programs on the same o network connected computers and move
the camera trying to maximize translation.

REBVO component accept the following command line commands:

q: Quit

r: System full reset

s: Saves in SimVideoFile (rebvo GlobalConfig) SimVideoNFrames frames of
   uncompressed video for further replay. It does a direct write to disk,
   so it can loose frames depending on the system. The format is described
   in videocam.h

p: Takes Snapshot of the current image


The visualizer opens 2 kinds of windows, XLib windows (front and top) 
displays the image and the edge detection, OpenGL windows display 
the edgemap and the image in a 3D fashion.

#### Commands in the XLib windows

q: Quit

b: Clear trajectory

i: Toggle show image

t: Show frame to frame matching

z,x: Increase/Decease maximum depth in top view

#### Commands in the OpenGL windows

w,s,a,d: Walk the view like DukeNuke3D
arrows: Rotate the view

f,r: flight

1..9: Load i view

Shift+1..9: Save i view

Escape: Reset View

m: Toggle image rendering

n: Toggle edgemap rendering

c: Toggle camera rendering

t,g: Increase/decrease max distance for color scale

y,h: Same for min distance 

i: Render edgemap depth uncertainty

v: Render Trajectory

b: Clear trajectory

,: Fix the view to world coordinates

End: Quit

### Output files

If the savelog option is enabled the system outputs 2 files, a .m log file and a
trajectory file (timestamp tx ty tz qx qy qz qw) that can be used to benchmark
the algorithm.

### Camera Drivers

Three classes are provided for camera managment:

-- v4lCam is a wrapper to the C functions provided in video_io for interacting with
   the v4l2 lib. 

-- SimCamera is a simple class designed to load uncompresed video from a file. For compressed video
   formats check the Video2SimCam section.

-- DataSetCam is used to load the images from the TUM datasets used to benchmark the
   paper (add the dataset directory and image file list to the config file).

All three classes inherit from VideoCam class, this class is able to generate the 
video files used by simcam. 

If you want to use your own camera loader, just write a child overwriting the virtual
functions in VideoCam and add it to the first thread of rebvo.

#### Video2SimCam Utility

Currently rebvo canot load compressed video directly (a feature that is gonna be added soon), so
a simple utility is provided in the Video2SimCam folder that uses OpenCV VideoCapture to uncompress
the video in the SimCam format (can take a lot of disk space!).



### FAQ

-- Why rebvo doesn't use OpenCV?

OpenCV is great, the system originally used OpenCV for image acquisition 
and preprocessing. 
Then at some point we figure that it was a waste having to install
the whole library just for image acquisition, so I write small a wrapper 
to the V4L2 library trying to make the system easier to install and 
less dependant on third party libraries.

-- What about that simulation feature?

If you use the rebvo 's' command it will save video for further repetition.
You can visualize that video toggling CameraType to 1 (SimCamera)

### Contact

Juan Jose Tarrio

juan.tarrio@gmail.com
