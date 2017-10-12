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
#include "VideoLib/v4lcam.h"
#include "UtilLib/util.h"
namespace  rebvo{
v4lCam::v4lCam(const char *dev_name,Size2D frame_size,int f_per_sec,const char *log_name)
    :VideoCam(log_name,frame_size)
{
    if(CamaraInit(dev_name, this,frame_size,f_per_sec)){
        printf("\nv4lCam: No puedo iniciar la camara %s\n",dev_name);
        error=true;
        return;
    }
    error=false;
}

v4lCam::~v4lCam()
{
    if(error)
        return;
    CamaraClose(this);
}

int v4lCam::WaitFrame(bool drop_frames){

    if(error)
        return -1;
    return CamaraWaitFrame(this);
}

int v4lCam::GrabFrame(RGB24Pixel *data, double &tstamp,bool drop_frames)
{

    timeval tv;
    if(error)
        return -1;
    int r=CamaraGrabFrame(this,data,&tv);

    tstamp=(double)tv.tv_sec+(double)tv.tv_usec*1.0e-6;

    paknum++;

    return r;
}

RGB24Pixel* v4lCam::GrabBuffer(double &tstamp, bool drop_frames)
{

    timeval tv;
    if(error)
        return NULL;

    RGB24Pixel* data=CamaraGrabBuffer(this,&tv);

    paknum++;

    tstamp=(double)tv.tv_sec+(double)tv.tv_usec*1.0e-6;
    return data;
}

int v4lCam::ReleaseBuffer()
{

    if(error)
        return -1;
    return CamaraReleaseBuffer(this);
}
}
