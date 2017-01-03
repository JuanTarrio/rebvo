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

#ifndef SIMCAM_H
#define SIMCAM_H

#include "videocam.h"
namespace  rebvo{
class simcam : public VideoCam
{
    bool error;

    FILE *fd;

    RGB24Pixel* buffer;

    VCFrameHdr hdr;

    bool frm_pending;


public:
    simcam(const char *sim_name,Size2D frame_size,const char *log_name=NULL);
    ~simcam();
    int WaitFrame(bool drop_frames=true) override;
    int GrabFrame(RGB24Pixel *data, double &tstamp, bool drop_frames=true) override;
    RGB24Pixel* GrabBuffer(double &tstamp,bool drop_frames=true) override;
    int ReleaseBuffer() override ;
    const bool & Error() override{return error;}
};

}

#endif // SIMCAM_H
