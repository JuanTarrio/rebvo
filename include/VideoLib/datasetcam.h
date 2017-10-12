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

#ifndef DATACAM_H
#define DATACAM_H


#include "vector"

#include "image.h"

#include "videocam.h"
namespace  rebvo{

class DataSetCam : public VideoCam
{
    bool error=true;

    bool frm_pending=false;

    Image <RGB24Pixel> buffer;
    double time=0;

    std::string strDir;

    std::vector <std::string> img_list;
    std::vector <double> img_time;
    u_int img_inx=0;


public:
    DataSetCam(const char *DataSetDir, const char *DataSetFile, Size2D frame_size, double time_scale, const char *log_name=NULL);
    ~DataSetCam();
    int WaitFrame(bool drop_frames=true) override;
    int LoadImage(const std::string &i_name);
    int GrabFrame(RGB24Pixel *data, double &tstamp, bool drop_frames=true) override;
    RGB24Pixel* GrabBuffer(double &tstamp,bool drop_frames=true) override;
    int ReleaseBuffer() override ;
    const bool & Error() override{return error;}
    bool SearchFrame(RGB24Pixel *data, const double &tstamp);
};

}
#endif // DATACAM_H
