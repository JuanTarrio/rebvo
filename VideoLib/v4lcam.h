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


#ifndef V4LCAM_H
#define V4LCAM_H

#include "videocam.h"
namespace  rebvo{
//******* C++ Wrappr to video_io.h interface to V4L2 *************

class v4lCam:public camera_context,public VideoCam
{
    bool error;

public:
    v4lCam(const char *dev_name, Size2D frame_size, int f_per_sec, const char *log_name=NULL);
    ~v4lCam();
    int WaitFrame(bool drop_frames=true) override;
    int GrabFrame(RGB24Pixel *data, double &tstamp,bool drop_frames=true) override;
    RGB24Pixel* GrabBuffer(double &tstamp,bool drop_frames=true) override;
    int ReleaseBuffer() override;
    const bool & Error() override{return error;}
};

}

#endif // V4LCAM_H
