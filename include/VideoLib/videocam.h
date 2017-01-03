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


#ifndef VIDEOCAM_H
#define VIDEOCAM_H


#include "VideoLib/video_io.h"
#include "VideoLib/image.h"
#include <string>
namespace  rebvo{

struct VCFrameHdr{

    int n;
    double time;

};

class VideoCam
{


    bool recording;
    FILE *fd;
    std::string f_name;
    bool log_error;

    int rec_n_frames;

    int sec;



    bool Open();
    void Close();

protected:

    int paknum;
    Size2D frm_size;

public:
    VideoCam(const char *file_name,Size2D frame_size);
    virtual ~VideoCam();
    bool StartRecord();

    void StopRecord();

    bool RecordNFrames(int f_num);

    bool PushFrame(RGB24Pixel* data);

    virtual int WaitFrame(bool drop_frames=true){return 0;}


    virtual int GrabFrame(RGB24Pixel *data, double &tstamp,bool drop_frames=true){return 0;}
    virtual RGB24Pixel* GrabBuffer(double &tstamp,bool drop_frames=true){return 0;}

    virtual int ReleaseBuffer(){return 0;}

    virtual const bool & Error(){return log_error;}

    const int& PakNum(){return paknum;}
};


}
#endif // VIDEOCAM_H
