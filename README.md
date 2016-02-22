
# REBVO
## Realtime Edge Based Visual Odometry for a Monocular Camera

Tarrio, J. J., & Pedre, S. (2015). Realtime Edge-Based Visual Odometry
for a Monocular Camera. In Proceedings of the IEEE International Conference
on Computer Vision (pp. 702-710).

REBVO tracks a camera in Realtime using edges. The system is split in 2 components.
An on-board part (rebvo itself) doing all the processing and sending data over UDP 
and an OpenGL visualizer. 

### System requirements

In ubuntu and most linux dist this libraries can be downloaded directly from the
 repos, except for TooN.

-- C++11

-- Linux, X11, v4l2

-- OpenGL development libraries (GL,GLU,glut)

-- TooN mathematical library (http://www.edwardrosten.com/cvd/toon.html)

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

### Configuring the system

A GlobalConfig text file can be found in each of the folders. This should be tuned to
your camera and network configuration. All the configuration options has to be present on the files
in order for the system to start. The structure of the file is self explanatory.

#### Basic configs in rebvo GlobalConfig

The things you have to configure in order for the system to work.

-- CameraDevice: the v4l2 camera device

-- ZfX,Y: Camera focal length

-- PPx,y: Camera principal point

-- ImageWidth,Height: size to use

-- FPS: Frames per Seconds

-- VideoNetHost,Port: IP,Port of the host to send data to

-- SetAfinity: if the set to 1, the threads has to be distributed manually in 
		existing processors, this can increase speed

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

-- SimCamera is a simple class designed to load uncompresed video from a file. 

-- DataSetCam is used to load the images from the TUM datasets used to benchmark the
   paper (add the dataset directory and image file list to the config file).

All three classes inherit from VideoCam class, this class is able to generate the 
video files used by simcam. 

If you want to use your own camera loader, just write a child overwriting the virtual
functions in VideoCam and add it to the first thread of rebvo.
### FAQ

-- Depth is not correct, objects that are close appear far and viceversa

At to this point, rebvo relies only on filtering for initialization, this
can lead to initialization errors since convergence is not warrantied. If
the system didn't bootup right, just reset.

-- How is rebvo someway useful if initialization can fail

If gyroscope measurements are added to estimation the initialization
problem disappears. That is the version used to fly quadrotors.
If you want that version contact the authors.

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
