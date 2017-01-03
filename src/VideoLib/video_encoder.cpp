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

 #include "VideoLib/video_encoder.h"
#include <string.h>
namespace  rebvo{
VideoEncoder::VideoEncoder(Size2D frame_size)
{
    f_size=frame_size;
    data=NULL;
}

VideoEncoder::VideoEncoder()
{
    f_size.h=0;
    f_size.w=0;
    data=NULL;
}

int VideoEncoder::PopFrame(char *stream_buf, int sb_size){

    if(data==NULL || sb_size<(int)(f_size.w*f_size.h*sizeof(RGB24Pixel)))
        return 0;

    sb_size=f_size.w*f_size.h*sizeof(RGB24Pixel);
    memcpy(stream_buf,data,sb_size);
    return sb_size;
}
int VideoEncoder::PushFrame(RGB24Pixel *data){
    this->data=data;
    return 0;
}
}
