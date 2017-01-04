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
#include <iostream>
#include "err.h"

#include "VideoLib/video_io.h"
#include "VideoLib/video_mfc.h"

#define memzero(d) memset(&(d), 0, sizeof(d))


#define MFC_MAX_PLANES 2
namespace  rebvo{
inline v4l2_memory io_type_to_memory(MFC_io_type type)
{
    switch (type) {
    case IO_USERPTR: return V4L2_MEMORY_USERPTR;
    case IO_MMAP: return V4L2_MEMORY_MMAP;
    default: return (v4l2_memory)NULL;
    }
}

inline int is_buf_type(MFC_io_type type)
{
    return (type == IO_USERPTR) || (type == IO_MMAP);
}

inline v4l2_buf_type io_dir_to_type(MFC_io_dir dir)
{
    switch (dir) {
    case DIR_IN: return V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    case DIR_OUT: return V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    default: return (v4l2_buf_type)NULL;
    }
}

inline int align(int x, int a)
{
    return ((x + a - 1) / a) * a;
}


EncoderMFC::EncoderMFC(const char *name)
    :VideoEncoder()
{
    /*
    dev->ops = &mfc_dev_ops;

    dev->io[DIR_IN].type = IO_MMAP;
    dev->io[DIR_OUT].type = IO_MMAP;
*/
    fd = open(name, O_RDWR| O_NONBLOCK);
    if (fd < 0) {
        printf("\nEncoderMFC: VideoCannot open MFC device %s\n", name);
        state=MFC_ST_ERROR;
        return;
    }

    v4l2_event_subscription ev_sub;
    memzero(ev_sub);
    ev_sub.type = V4L2_EVENT_EOS;
    int ret = ioctl(fd, VIDIOC_SUBSCRIBE_EVENT, &ev_sub);
    if (ret != 0)
        printf("Cannot subscribe EOS event for MFC");

    printf("\nEncoderMFC: MFC device %s opened with fd=%d\n", name,fd);
    state=MFC_ST_OPEN;
}

EncoderMFC::~EncoderMFC(){


    SendStop();
    StreamOn(false);

    close(fd);

    printf("\nEncoderMFC: close fd\n");

}

int EncoderMFC::SendStop(){
    v4l2_encoder_cmd cmd;
    memset(&cmd, 0, sizeof cmd);
    cmd.cmd = V4L2_ENC_CMD_STOP;
    int ret = ioctl(fd, VIDIOC_ENCODER_CMD, &cmd);
    printf("\nEncoderMFC: Command Stop %d\n",ret);
    return ret;
}

int EncoderMFC::Initialize(int width, int height,int codec,float rate,int bitrate,int in_bufs,int out_bufs, int stream_b_size){

    if(state!=MFC_ST_OPEN)
        return -1;

    if(SetFormat(width,height)<0)
        return -1;

    if(SetCodec(codec,stream_b_size)<0)
        return -1;

    SetRate(rate);

    if(bitrate>0)
        SetBitRate(bitrate);

    if(InitBuffers(DIR_IN,in_bufs)<0)
        return -1;

    if(InitBuffers(DIR_OUT,out_bufs)<0)
        return -1;


    for (int bi=0;bi<out_bufs;bi++){


        struct v4l2_buffer buf;
        struct v4l2_plane planes[MFC_MAX_PLANES];

        buf.type = io_dir_to_type(DIR_OUT);
        buf.memory = V4L2_MEMORY_MMAP;
        buf.m.planes = planes;
        buf.length = NPPBuf[DIR_OUT];

        buf.index=bi;


        for (u_int i = 0; i < NPPBuf[DIR_OUT]; i++){
            planes[i].length=PlanesLen[DIR_OUT][i];
            planes[i].bytesused=0;
            planes[i].m.userptr=(u_long)BufAddr[DIR_OUT][bi][i];
        }

        int ret = ioctl(fd, VIDIOC_QBUF, &buf);
        if (ret != 0) {
            printf("\nEncoderMFC: Init, Queue buffer %d error! %s\n",buf.index,strerror(errno));
        }

    }

    BufInx[DIR_OUT]=0;

    state=MFC_ST_OK;
    return 0;

}


void EncoderMFC::Autoconfig(){

    //encoder->SetControl(V4L2_CID_MPEG_VIDEO_H264_8X8_TRANSFORM,1);

    SetControl(V4L2_CID_MPEG_VIDEO_MPEG4_I_FRAME_QP,20);
    SetControl(V4L2_CID_MPEG_VIDEO_MPEG4_P_FRAME_QP,20);
    SetControl(V4L2_CID_MPEG_VIDEO_MPEG4_B_FRAME_QP,31);

    SetControl(V4L2_CID_MPEG_VIDEO_MPEG4_MIN_QP,20);
    SetControl(V4L2_CID_MPEG_VIDEO_MPEG4_MAX_QP,31);

    SetControl(V4L2_CID_MPEG_VIDEO_HEADER_MODE,V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME);
    SetControl(V4L2_CID_MPEG_MFC51_VIDEO_H264_NUM_REF_PIC_FOR_P,1);

    SetControl(V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE,1);
    SetControl(V4L2_CID_MPEG_VIDEO_MB_RC_ENABLE,1);
    SetControl(V4L2_CID_MPEG_MFC51_VIDEO_RC_REACTION_COEFF,1000);
    SetControl(V4L2_CID_MPEG_MFC51_VIDEO_FORCE_FRAME_TYPE,V4L2_MPEG_MFC51_VIDEO_FORCE_FRAME_TYPE_DISABLED);

    SetControl(V4L2_CID_MPEG_VIDEO_B_FRAMES,0);
    SetControl(V4L2_CID_MPEG_VIDEO_GOP_SIZE,15);

}

int EncoderMFC::SetCodec(int codec,int stream_b_size)
{
    v4l2_format fmt;
    int ret;

    memzero(fmt);
    fmt.type = io_dir_to_type(DIR_OUT);
    fmt.fmt.pix_mp.pixelformat = codec;
    fmt.fmt.pix_mp.plane_fmt[0].sizeimage = stream_b_size;

    ret = ioctl(fd, VIDIOC_S_FMT, &fmt);

    if (ret != 0){
        printf("\nEncoderMFC: Cannot set codec!\n");
        state=MFC_ST_ERROR;
    }

    return ret;
}

int EncoderMFC::SetFormat(int width, int height)
{
    v4l2_format fmt;
    int ret;

    this->width=width;
    this->height=height;

    memzero(fmt);
    fmt.type = io_dir_to_type(DIR_IN);
    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12M;
    fmt.fmt.pix_mp.width = width;
    fmt.fmt.pix_mp.height = height;


    BytesPerLine=align(width, 128);

    fmt.fmt.pix_mp.num_planes = 2;
    fmt.fmt.pix_mp.plane_fmt[0].bytesperline = BytesPerLine;
    fmt.fmt.pix_mp.plane_fmt[0].sizeimage = align(BytesPerLine * height, 2048);
    fmt.fmt.pix_mp.plane_fmt[1].bytesperline = BytesPerLine;
    fmt.fmt.pix_mp.plane_fmt[1].sizeimage = align(BytesPerLine * (height / 2),2048);


    printf("\nEncoderMFC: bpl = %d,%d \n",BytesPerLine,fmt.fmt.pix_mp.plane_fmt[0].sizeimage);

    ret = ioctl(fd, VIDIOC_S_FMT, &fmt);
    if (ret != 0){
        printf("\nEncoderMFC: Cannot set format!\n");
        state=MFC_ST_ERROR;
        return -1;
    }

    ret = ioctl(fd, VIDIOC_G_FMT, &fmt);
    if (ret != 0){
        printf("\nEncoderMFC: Cannot check format!\n");
        state=MFC_ST_ERROR;
        return -1;
    }

    BytesPerLine=fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
    printf("\nEncoderMFC: get bpl = %d,%d \n",BytesPerLine,fmt.fmt.pix_mp.plane_fmt[1].bytesperline);
    return ret;
}

int EncoderMFC::SetRate(float rate)
{
    v4l2_streamparm fps;
    int ret;

    this->fps=rate;

    fps.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;

    fps.parm.output.timeperframe.numerator = 1;
    fps.parm.output.timeperframe.denominator = rate;

    ret = ioctl(fd, VIDIOC_S_PARM, &fps);
    if (ret != 0){
        printf("\nEncoderMFC: Cannot set rate!\n");
        state=MFC_ST_ERROR;
        return -1;
    }

    return ret;
}

int EncoderMFC::SetControl(int id, int value)
{
    struct v4l2_ext_control ctrl;
    struct v4l2_ext_controls ctrls;
    int ret;

    ctrl.id = id;
    ctrl.value = value;

    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;
    ctrls.count = 1;
    ctrls.controls = &ctrl;

    ret = ioctl(fd, VIDIOC_S_EXT_CTRLS, &ctrls);
    if (ret < 0){
        printf("\nEncoderMFC: Cannot set control %d to %d!\n", id, value);
        state=MFC_ST_ERROR;
    }

    printf("\nEncoderMFC: Set control %d,%d OK!\n", id, value);
    return ret;
}

int EncoderMFC::SetBitRate(int bitrate)
{
    struct v4l2_ext_control ctrl;
    struct v4l2_ext_controls ctrls;
    int ret;

    this->bitrate=bitrate;

    ctrl.id = V4L2_CID_MPEG_VIDEO_BITRATE;
    ctrl.value = bitrate;

    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;
    ctrls.count = 1;
    ctrls.controls = &ctrl;

    ret = ioctl(fd, VIDIOC_S_EXT_CTRLS, &ctrls);
    if (ret < 0){
        printf("\nEncoderMFC: Cannot set bitrate!\n");
        state=MFC_ST_ERROR;
    }

    //SetControl(V4L2_CID_MPEG_VIDEO_BITRATE_MODE,V4L2_MPEG_VIDEO_BITRATE_MODE_CBR);

    printf("\nEncoderMFC: Bitrate set OK!\n");

    return ret;
}


/* switch on/off streaming on v4l device */
int EncoderMFC::StreamOn(bool SetOn)
{
    int ret;
    int buf_type;

    buf_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    ret = ioctl(fd, SetOn ? VIDIOC_STREAMON : VIDIOC_STREAMOFF,&buf_type);
    if (ret != 0) {
        printf("\nEncoderMFC: Cannot %s output stream!\n",SetOn ? "start" : "stop");
        state=MFC_ST_ERROR;
        return -1;
    }


    buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    ret = ioctl(fd, SetOn ? VIDIOC_STREAMON : VIDIOC_STREAMOFF, &buf_type);

    if (ret != 0) {
        printf("\nEncoderMFC: Cannot %s capture stream!\n",SetOn ? "start" : "stop");
        state=MFC_ST_ERROR;
        return -1;
    }


    state=MFC_ST_STREAM_ON;

    printf("\nEncoderMFC: Stream %s\n", SetOn ? "started" : "stopped");


    return 0;
}


int EncoderMFC::ReqBufs(MFC_io_dir dir, int nbuf)
{
    int ret;
    struct v4l2_requestbuffers reqbuf;

    memzero(reqbuf);
    reqbuf.count = nbuf;
    reqbuf.type = io_dir_to_type(dir);
    reqbuf.memory = V4L2_MEMORY_MMAP;

    ret = ioctl(fd, VIDIOC_REQBUFS, &reqbuf);
    if (ret != 0) {
        printf("\nEncoderMFC: Failed to request %d buffers for %s device\n", nbuf,
               dir==DIR_IN?"Input":"Output");
        state=MFC_ST_ERROR;
        return -1;
    }

    printf("\nEncoderMFC: Succesfully requested %d buffers for %s device\n", nbuf,
           dir==DIR_IN?"Input":"Output");

    return reqbuf.count;
}

int  EncoderMFC::InitBuffers(MFC_io_dir dir,int nbuf)
{
    struct v4l2_buffer qbuf;
    int ret;

    v4l2_plane planes[MFC_MAX_PLANES];


    if (ReqBufs(dir,nbuf) < 0)
        return -1;

    NBufs[dir]=nbuf;

    BufAddr[dir] = new void**[nbuf];


    memzero(qbuf);
    qbuf.type = io_dir_to_type(dir);
    qbuf.memory = V4L2_MEMORY_MMAP;
    qbuf.m.planes = planes;
    qbuf.length = MFC_MAX_PLANES;

    for (int n = 0; n < nbuf; n++) {
        qbuf.index = n;

        ret = ioctl(fd, VIDIOC_QUERYBUF, &qbuf);
        if (ret != 0) {
            printf("\nEncoderMFC: QUERYBUF failed\n");
            return -1;
        }

        if (n == 0) {
            int i;
            for (i = 0; i < qbuf.length; ++i)
                if (qbuf.m.planes[i].length == 0)
                    break;
            NPPBuf[dir] = i;
            PlanesLen[dir]=new int[i];


            for (i = 0; i < NPPBuf[dir]; ++i)
                PlanesLen[dir][i] = qbuf.m.planes[i].length;


        }

        BufAddr[dir][n] = new void*[NPPBuf[dir]];

        for (int i = 0; i < NPPBuf[dir]; ++i) {
            BufAddr[dir][n][i] = (void *)mmap(NULL,qbuf.m.planes[i].length, PROT_READ | PROT_WRITE, MAP_SHARED, fd,\
                        qbuf.m.planes[i].m.mem_offset);
            if (BufAddr[dir][n][i]  == MAP_FAILED) {
                printf("\nEncoderMFC: Failed mmap buffer %d for plane %d!\n", n,i);
                state=MFC_ST_ERROR;
                return -1;
            }
        }
        /*ret = in->ops->enq_buf(in, DIR_OUT, n);
        if (ret < 0)
            return -1;*/
    }


    printf(" \nMFC Log: Dir = %d NB = %d \n",dir,NBufs[dir]);

    for(int i=0;i<NBufs[dir];i++){

        printf("\nNPP = %d ",NPPBuf[dir]);

        for(int j=0;j<NPPBuf[dir];j++)
            printf(" PL = %d A = %lx ",PlanesLen[dir][j],(unsigned long)BufAddr[dir][i][j]);

    }

    printf("\n");

    BufInx[dir]=0;

    return 0;
}


int EncoderMFC::PopFrame(char *stream_buf,int b_size)
{
    struct v4l2_buffer buf;
    struct v4l2_plane planes[NPPBuf[DIR_OUT]];
    int ret;

    memzero(buf);

    buf.type = io_dir_to_type(DIR_OUT);
    buf.memory = V4L2_MEMORY_MMAP;
    buf.m.planes = planes;
    buf.length = NPPBuf[DIR_OUT];

    buf.index=BufInx[DIR_OUT];


    ret = ioctl(fd, VIDIOC_DQBUF, &buf);
    if (ret != 0) {
        printf("\nEncoderMFC: Pop, Dequeue buffer %d error, %s!\n",buf.index,strerror(errno));
        return -1;
    }



    BufInx[DIR_OUT]=buf.index;

    if(BufInx[DIR_OUT]>=NBufs[DIR_OUT]){
        printf("\nEncoderMFC: Pop, Index error!\n");
        return -1;
    }


    if(buf.m.planes[0].bytesused>b_size){
        printf("\nEncoderMFC: Pop, Buffer %d overrun %d error!",buf.index,buf.m.planes[0].bytesused);
    }else{
        b_size=buf.m.planes[0].bytesused;
    }

    memcpy(stream_buf,BufAddr[DIR_OUT][buf.index][0],b_size);

    //printf("\nEncoderMFC: Enc frame %d,%d\n",buf.m.planes[0].bytesused,BufInx[DIR_OUT]);

    ret = ioctl(fd, VIDIOC_QBUF, &buf);
    if (ret != 0) {
        printf("\nEncoderMFC: Pop, Queue buffer %d error! %s\n",BufInx[DIR_OUT],strerror(errno));
        return -1;
    }

    //BufInx[DIR_OUT]++;

    return b_size;
}

/* enqueue buffer, start stream when needed */
int EncoderMFC::PushFrame(union RGB24Pixel *data)
{
    struct v4l2_buffer buf;
    struct v4l2_plane planes[MFC_MAX_PLANES];
    int ret;

    memzero(buf);

    buf.index=(BufInx[DIR_IN]);
    buf.type = io_dir_to_type(DIR_IN);
    buf.memory = V4L2_MEMORY_MMAP;
    buf.m.planes = planes;
    buf.length = NPPBuf[DIR_IN];




    float Y,Cr,Cb;

    for(int y=0,i=0;y<height;y++,i+=BytesPerLine-width){
        for(int x=0;x<width;x++,i++){

            Y=(0.299 * (float)data[i].pix.r) + (0.587 * (float)data[i].pix.g) + (0.114 * (float)data[i].pix.b);

            ((unsigned char *)(BufAddr[DIR_IN][buf.index][0]))[i]  = Y;

        }
    }


    planes[0].bytesused=height*BytesPerLine;
    planes[0].length = PlanesLen[DIR_IN][0];
    planes[0].m.userptr = (u_long)BufAddr [DIR_IN][buf.index][0];


    for(int y=0,i=0,j=0;y<height/2;y++,i+=width,j+=BytesPerLine-width){
        for(int x=0;x<width;x+=2,i+=2,j+=2){
            float r,g,b;

            r=(data[i].pix.r+data[i+1].pix.r+data[i+width].pix.r+data[i+width+1].pix.r)/4.0;
            g=(data[i].pix.g+data[i+1].pix.g+data[i+width].pix.g+data[i+width+1].pix.g)/4.0;
            b=(data[i].pix.b+data[i+1].pix.b+data[i+width].pix.b+data[i+width+1].pix.b)/4.0;

            Cr = (0.5 * r) - (0.4186 * g) - (0.08131 * b);
            Cb =-(0.1687 * r) - (0.3312 * g) + (0.5 * b);

            Cr+=128.0;
            Cb+=128.0;

            ((unsigned char *)(BufAddr[DIR_IN][buf.index][1]))[j+1]  = Cr;
            ((unsigned char *)(BufAddr[DIR_IN][buf.index][1]))[j]  = Cb;

        }
    }


    planes[1].bytesused=height*BytesPerLine/2;
    planes[1].length = PlanesLen[DIR_IN][1];
    planes[1].m.userptr = (u_long)BufAddr [DIR_IN][buf.index][1];




    ret = ioctl(fd, VIDIOC_QBUF, &buf);
    if (ret != 0) {
        printf("\nEncoderMFC: Push, Queue buffer %d error!\n",BufInx[DIR_IN]);
    }

    if(state==MFC_ST_OK)
        StreamOn(true);

    if((++(BufInx[DIR_IN]))>=NBufs[DIR_IN]){
        BufInx[DIR_IN]=0;
    }

    buf.index=(BufInx[DIR_IN]);
    buf.type = io_dir_to_type(DIR_IN);
    buf.memory = V4L2_MEMORY_MMAP;
    buf.m.planes = planes;
    buf.length = NPPBuf[DIR_IN];

    ret = ioctl(fd, VIDIOC_DQBUF, &buf);
    if (ret != 0) {
        printf("\nEncoderMFC: Push, Dequeue buffer %d error! %s\n",BufInx[DIR_IN],strerror(errno));
        return -1;
    }

    return 0;
}


void EncoderMFC::Convert_RGB_2_NV12(union RGB24Pixel *data,char **b_plane,int *s_planes){

    float Y,Cr,Cb;

    for(int y=0,i=0;y<height;y++){
        for(int x=0;x<width;x++,i++){

            Y=(0.257 * (float)data[i].pix.r) + (0.504 * (float)data[i].pix.g) + (0.098 * (float)data[i].pix.b) + 16;

            b_plane[0][i]  = Y;

        }
    }

    for(int y=0;y<height/2;y++){
        for(int x=0;x<width;x+=2){

            int i=y*2*width+x;

            Cr = (0.439 * (float)data[i].pix.r) - (0.368 * (float)data[i].pix.g) - (0.071 * (float)data[i].pix.b) + 128;
            Cb =-(0.148 * (float)data[i].pix.r) - (0.291 * (float)data[i].pix.g) + (0.439 * (float)data[i].pix.b) + 128;

            i++;

            Cr += (0.439 * (float)data[i].pix.r) - (0.368 * (float)data[i].pix.g) - (0.071 * (float)data[i].pix.b) + 128;
            Cb +=-(0.148 * (float)data[i].pix.r) - (0.291 * (float)data[i].pix.g) + (0.439 * (float)data[i].pix.b) + 128;

            i=(y*2+1)*width+x;

            Cr = (0.439 * (float)data[i].pix.r) - (0.368 * (float)data[i].pix.g) - (0.071 * (float)data[i].pix.b) + 128;
            Cb =-(0.148 * (float)data[i].pix.r) - (0.291 * (float)data[i].pix.g) + (0.439 * (float)data[i].pix.b) + 128;

            i++;

            Cr += (0.439 * (float)data[i].pix.r) - (0.368 * (float)data[i].pix.g) - (0.071 * (float)data[i].pix.b) + 128;
            Cb +=-(0.148 * (float)data[i].pix.r) - (0.291 * (float)data[i].pix.g) + (0.439 * (float)data[i].pix.b) + 128;


            b_plane[1][y*width+x]=Cr/4.0;
            b_plane[1][y*width+x+1]=Cb/4.0;


        }
    }

    s_planes[0]=width*height;
    s_planes[1]=width*height/2;
}

/*
int v4l_deq_event(struct io_dev *dev)
{
    struct v4l2_event ev;
    int ret;

    memzero(ev);
    ret = ioctl(dev->fd, VIDIOC_DQEVENT, &ev);
    if (ret != 0)
        return ret;

    switch (ev.type) {
    case V4L2_EVENT_EOS:
        dev->io[DIR_OUT].state = FS_END;
        dbg("EOS event received on %d", dev->fd);
    }

    dev->event = 0;

    return 0;
}

*/
}
