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

 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>

#include "VideoLib/video_io.h"

namespace  rebvo{

static void xioctl(int fh, int request, void *arg)
{
	int r;

	do {
		r = v4l2_ioctl(fh, request, arg);
	} while (r == -1 && ((errno == EINTR) || (errno == EAGAIN)));

	if (r == -1) {
		printf( "\nCamara: error ioctl %d, %s\n", errno, strerror(errno));
		exit(EXIT_FAILURE);
	}
}

int CamaraInit(const char * dev_name, struct camera_context *cam, struct Size2D frameSize, uint fps){
	struct v4l2_format              fmt;
	struct v4l2_buffer              buf;
	struct v4l2_requestbuffers      req;
	enum v4l2_buf_type              type;
	struct v4l2_streamparm			cap;
	int                             i;


	cam->fd = v4l2_open(dev_name, O_RDWR /*| O_NONBLOCK*/, 0);
	if (cam->fd < 0) {
		printf("\nCamara: No puedo abrir el dispocitivo!\n");
		return -1;
	}

	memset(&fmt,0,sizeof(fmt));
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width       = frameSize.w;
	fmt.fmt.pix.height      = frameSize.h;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
	fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;

	xioctl(cam->fd, VIDIOC_S_FMT, &fmt);

	cam->width=fmt.fmt.pix.width;
	cam->height=fmt.fmt.pix.height;
	cam->pixelformat=fmt.fmt.pix.pixelformat;

	if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_RGB24) {
		printf("\nCamara: v4l doesn't accept RGB24!\n");
		return -2;
	}

	if ((fmt.fmt.pix.width != frameSize.w) || (fmt.fmt.pix.height != frameSize.h)){
		printf("Camara: cannot configure framesize, driver size= %dx%d\n",
			fmt.fmt.pix.width, fmt.fmt.pix.height);
		return -3;
	}

	memset(&cap,0,sizeof(cap));

	cap.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;

	xioctl(cam->fd, VIDIOC_G_PARM, &cap);

	cap.parm.capture.capability=V4L2_CAP_TIMEPERFRAME;

    cap.parm.capture.timeperframe.numerator=1;
	cap.parm.capture.timeperframe.denominator=fps;


	xioctl(cam->fd, VIDIOC_S_PARM, &cap);

    if (cap.parm.capture.timeperframe.numerator!=1 || cap.parm.capture.timeperframe.denominator!=fps){
		printf("Camara: cannot configure FPS, driver set to %d / %d \n",
			cap.parm.capture.timeperframe.numerator,cap.parm.capture.timeperframe.denominator);
		return -3;
    }


	memset(&req,0,sizeof(req));
    req.count = 3;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;
	xioctl(cam->fd, VIDIOC_REQBUFS, &req);

	cam->buf_data.buf_n=req.count;

    cam->buf_data.start = (void **)calloc(cam->buf_data.buf_n, sizeof(void*));
    cam->buf_data.length = (size_t *)calloc(cam->buf_data.buf_n, sizeof(size_t));

	for (i = 0; i < cam->buf_data.buf_n; ++i) {
		memset(&buf,0,sizeof(buf));

		buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory      = V4L2_MEMORY_MMAP;
		buf.index       = i;

		xioctl(cam->fd, VIDIOC_QUERYBUF, &buf);

		cam->buf_data.length[i] = buf.length;
		cam->buf_data.start[i] = v4l2_mmap(NULL, buf.length,
			      PROT_READ | PROT_WRITE, MAP_SHARED,
			      cam->fd, buf.m.offset);


		if (cam->buf_data.start[i] == MAP_FAILED) {
			printf("\nCamara: Error mapeando memoria de buffers\n");
			return -4;
		}
	}

	for (i = 0; i < cam->buf_data.buf_n; ++i) {
		memset(&buf,0,sizeof(buf));
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		xioctl(cam->fd, VIDIOC_QBUF, &buf);
	}
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	xioctl(cam->fd, VIDIOC_STREAMON, &type);
	printf("\nCamara: Stream init correctly\n");

	return 0;

}

int CamaraWaitFrame(struct camera_context *cam){

	fd_set	fds;
	struct timeval	tv;
	int r;
	do {
		FD_ZERO(&fds);
		FD_SET(cam->fd, &fds);

		/* Timeout. */
		tv.tv_sec = 2;
		tv.tv_usec = 0;

		r = select(cam->fd + 1, &fds, NULL, NULL, &tv);
	} while ((r == -1 && (errno = EINTR)));
	if (r == -1) {
		printf("Camara: error en llamada a select");
		return -1;
	}
    return 0;
}

int CamaraClose(struct camera_context *cam){

	enum v4l2_buf_type	type;
	int i;

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	xioctl(cam->fd, VIDIOC_STREAMOFF, &type);

	for (i = 0; i < cam->buf_data.buf_n; ++i)
		v4l2_munmap(cam->buf_data.start[i], cam->buf_data.length[i]);
	v4l2_close(cam->fd);
    return 0;
}

int CamaraGrabFrame(struct camera_context *cam, union RGB24Pixel *data, struct timeval *tstamp){

	memset(&cam->v2l_buf,0,sizeof(cam->v2l_buf));
	cam->v2l_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	cam->v2l_buf.memory = V4L2_MEMORY_MMAP;

	xioctl(cam->fd, VIDIOC_DQBUF, &cam->v2l_buf);


	memcpy(data,cam->buf_data.start[cam->v2l_buf.index], cam->v2l_buf.bytesused);
	memcpy(tstamp,&cam->v2l_buf.timestamp,sizeof(struct timeval));

	xioctl(cam->fd, VIDIOC_QBUF, &cam->v2l_buf);


	return 0;
}


union RGB24Pixel * CamaraGrabBuffer(struct camera_context *cam, struct timeval *tstamp){

	memset(&cam->v2l_buf,0,sizeof(cam->v2l_buf));
	cam->v2l_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	cam->v2l_buf.memory = V4L2_MEMORY_MMAP;
	xioctl(cam->fd, VIDIOC_DQBUF, &cam->v2l_buf);

	memcpy(tstamp,&cam->v2l_buf.timestamp,sizeof(struct timeval));

    return ( RGB24Pixel *)cam->buf_data.start[cam->v2l_buf.index];
}

int CamaraReleaseBuffer(struct camera_context *cam){
	xioctl(cam->fd, VIDIOC_QBUF, &cam->v2l_buf);
    return 0;
}

int SavePPM(char * fine_name,union RGB24Pixel *data, __u32 width, __u32 height){

	FILE	*fout;

	fout = fopen(fine_name, "w");
	if (!fout) {
		perror("Video Out: Cannot open image!");
		return -1;
	}
	fprintf(fout, "P6\n%d %d 255\n",
		width, height);
	fwrite(data,width*height*sizeof(union RGB24Pixel), 1, fout);
	fclose(fout);
    return 0;
}





int IniciarVideoOutput(struct XVideoContext *xvc,__u32 width,__u32 height)
{

	XGCValues          gcValues;
	ulong	             gcValuesMask;

	xvc->display = XOpenDisplay(NULL);
	if (xvc->display == NULL)
	{
		printf("\nVideoOut: No puedo abrir el display!\n");
		return -1;
	}

	xvc->defscreen = DefaultScreen(xvc->display);

	xvc->window = XCreateSimpleWindow(xvc->display, RootWindow(xvc->display, xvc->defscreen), 10, 10, width, height, 1,
				BlackPixel(xvc->display, xvc->defscreen), WhitePixel(xvc->display, xvc->defscreen));


	XMapWindow(xvc->display, xvc->window);

	XGetWindowAttributes(xvc->display, xvc->window, &xvc->windowAttributes);

	if ((xvc->img = XCreateImage(xvc->display, xvc->windowAttributes.visual, xvc->windowAttributes.depth, ZPixmap, 0\
									, NULL, width, height,8, 0)) == NULL){
		printf("\nVideoOut: Error creando imagen!\n");
		return -1;
	}

	xvc->width=width;
	xvc->height=height;

	if ((xvc->img->data = (char *) malloc(xvc->img->bytes_per_line * xvc->img->height)) == NULL) {
		printf("\nVideoOut: Alocando imagen!\n");
		return -1;
	}

	if(xvc->windowAttributes.depth!=24){
		printf("\nVideoOut: La profundidad debe ser 24 bits!\n");
		return -1;
	}


	gcValues.function = GXcopy;
	gcValuesMask = GCFunction;
	if((xvc->gc = XCreateGC(xvc->display, xvc->window, gcValuesMask, &gcValues))==NULL){
		printf("\nVideoOut: Error creando graphic context!\n");
		return -1;
	}

    if((xvc->pixmap=XCreatePixmap(xvc->display,xvc->window,xvc->width,xvc->height,xvc->windowAttributes.depth))==0){
        printf("\nVideoOut: Error creando pixmap!\n");
        return -1;
    }

	xvc->wmDeleteMessage = XInternAtom(xvc->display, "WM_DELETE_WINDOW", False);
	XSetWMProtocols(xvc->display, xvc->window, &xvc->wmDeleteMessage, 1);

	XSelectInput(xvc->display, xvc->window, KeyPressMask);

	xvc->key=0;
	return 0;
}

int MostrarVideo(struct XVideoContext *xvc,union RGB24Pixel *data,DrawFrameFunc *fc,void *param){
	char *dptr=xvc->img->data,*sptr=(char*)data;
    uint i;
	XEvent event;

    if(XCheckWindowEvent(xvc->display,xvc->window,!NoEventMask,&event)){
        //XNextEvent(xvc->display,&event);

		if (event.type == KeyPress)
			xvc->key=XLookupKeysym(&event.xkey,0);
		else if(event.type == ClientMessage){
            if (event.xclient.data.l[0] == (uint)xvc->wmDeleteMessage)
				return -1;
		}

	}

	for(i=0;i<xvc->width*xvc->height;i++){

		*(dptr+2)=*(sptr++);
		*(dptr+1)=*(sptr++);
		*(dptr+0)=*(sptr++);
		*(dptr+4)=0;
		dptr+=4;
	}

    XPutImage(xvc->display, xvc->pixmap, xvc->gc, xvc->img, 0, 0, 0, 0, xvc->width, xvc->height);
	//XDrawString(xvc->display, xvc->window, DefaultGC(xvc->display, xvc->defscreen), 20, 20, text, strlen(text));
	if(fc)
	    fc(xvc,param);

    XCopyArea(xvc->display,xvc->pixmap,xvc->window,xvc->gc,0,0,xvc->width,xvc->height,0,0);
	XFlush(xvc->display);
	return 0;
}

int TerminarVideoOutput(struct XVideoContext *xvc)
{
	//XDestroyImage(xvc->img);
	if(xvc->img->data)
		free(xvc->img->data);
	XCloseDisplay(xvc->display);

    return 0;
}


__u8 GetKey(struct XVideoContext *xvc){
	__u8 k=xvc->key;
	xvc->key=0;
	return k;
}

}
