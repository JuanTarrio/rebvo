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


#include "VideoLib/customcam.h"
namespace  rebvo{
customCam::customCam(Pipeline<CustomCamPipeBuffer> &cam_pipe, Size2D frame_size, const char *log_name)
    :VideoCam(log_name,frame_size),pipe(cam_pipe)
{
}



int customCam::WaitFrame(bool drop_frames){



    return 0;

}

int customCam::GrabFrame(RGB24Pixel *data, double &tstamp, bool drop_frames){


    CustomCamPipeBuffer &pbuf=pipe.RequestBuffer(1);

    tstamp=pbuf.timestamp;
    paknum++;

    (*pbuf.img).copyTo(data);

    pipe.ReleaseBuffer(1);
    return 0;
}

RGB24Pixel* customCam::GrabBuffer(double &tstamp, bool drop_frames){


    CustomCamPipeBuffer *pbuf=pipe.RequestBufferTimeoutable(1,0.001);

    if(!pbuf)
    	return nullptr;


    tstamp=pbuf->timestamp;
    paknum++;
    return (*pbuf->img).Data();
}

int customCam::ReleaseBuffer(){
    pipe.ReleaseBuffer(1);
    return 0;
}
}
