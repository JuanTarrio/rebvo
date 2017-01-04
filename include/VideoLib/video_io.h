/******************************************************************************

   REBVO: RealTime Edge Based Visual Odometry For a Monocular Camera.
   Copyright (C) 2016  Juan José Tarrio
   
   Jose Tarrio, J., & Pedre, S. (2015). Realtime Edge-Based Visual Odometry
   for a Monocular Camera. In Proceedings of the IEEE International Conference
   on Computer Vision (pp. 702-710).
   
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA

 *******************************************************************************/

 
#ifndef VIDEO_IO
#define VIDEO_IO

#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <libv4l2.h>
#include <X11/Xlib.h>


namespace  rebvo{


struct buffer_data {
	int buf_n;
	void   **start;
	size_t *length;
};

union RGB24Pixel{
	struct {
		__u8 r;
		__u8 g;
		__u8 b;
	} pix ;
	__u8 dat[3];
};



struct Point2DI{
    int x;
    int y;
};

struct Size2D{
    u_int w;
    u_int h;
};


struct Point2DF{
    float x;
    float y;
};




struct camera_context{
	int		    fd;
	struct buffer_data  buf_data;
	__u32		    width;
	__u32		    height;
	__u32		    pixelformat;
	struct v4l2_buffer  v2l_buf;
};


struct XVideoContext{
	 Display 	*display;
	 int 		defscreen;
	 Window 	window;
	 XWindowAttributes  windowAttributes;
	 XEvent 	event;
	 XImage 	*img;
     Pixmap     pixmap;
	 GC 		gc;
	 __u32 		width;
	 __u32 		height;
	 __u32		key;
	 Atom 		wmDeleteMessage;
};


typedef int DrawFrameFunc(struct XVideoContext *xvc,void *param);

int CamaraInit(const char * dev_name, struct camera_context *cam, struct Size2D frameSize, uint fps);
int CamaraWaitFrame(struct camera_context *cam);
int CamaraClose(struct camera_context *cam);
int CamaraGrabFrame(struct camera_context *cam, union RGB24Pixel *data, struct timeval *tstamp);
union RGB24Pixel * CamaraGrabBuffer(struct camera_context *cam, struct timeval *tstamp);
int CamaraReleaseBuffer(struct camera_context *cam);
int SavePPM(char * fine_name,union RGB24Pixel *data, __u32 width, __u32 height);
int IniciarVideoOutput(struct XVideoContext *xvc,__u32 width,__u32 height);
int MostrarVideo(struct XVideoContext *xvc,union RGB24Pixel *data,DrawFrameFunc *fc,void *param);
int TerminarVideoOutput(struct XVideoContext *xvc);
__u8 GetKey(struct XVideoContext *xvc);

}

#endif
