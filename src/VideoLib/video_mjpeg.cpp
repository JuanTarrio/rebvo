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


#include "VideoLib/video_mjpeg.h"
#include "UtilLib/util.h"
#include <string.h>
namespace  rebvo{
MJPEGEncoder::MJPEGEncoder(const Size2D &f_size,int quality)
    :VideoEncoder()
{
    this->f_size=f_size;


    gdi=gdImageCreateTrueColor(f_size.w,f_size.h);
    jpeg_buf=0;

    this->quality=quality;
}


MJPEGEncoder::~MJPEGEncoder(){

    if(jpeg_buf!=0)
        gdFree(jpeg_buf);
    gdFree(gdi);

}

int MJPEGEncoder::PopFrame(char *stream_buf, int sb_size){

    if(jpeg_buf==0)
        return 0;

    util::keep_min(sb_size,b_size);

    memcpy(stream_buf,jpeg_buf,sb_size);
    jpeg_buf=0;
    b_size=0;
    return sb_size;
}

int MJPEGEncoder::PushFrame(RGB24Pixel *data){

    for(int y=0,i=0;y<f_size.h;y++)
        for(int x=0;x<f_size.w;x++,i++)
            gdImageSetPixel(gdi,x,y,gdTrueColor(data[i].pix.r,data[i].pix.g,data[i].pix.b));


    jpeg_buf=(char *)gdImageJpegPtr(gdi,&b_size,quality);

    return 0;

}
}
