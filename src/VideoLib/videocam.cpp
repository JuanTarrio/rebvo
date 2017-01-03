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


#include "VideoLib/videocam.h"
#include "UtilLib/ttimer.h"
#include <time.h>
namespace  rebvo{

VideoCam::VideoCam(const char *file_name, Size2D frame_size)
{

    if(file_name)
        f_name = std::string(file_name);
    else
        f_name = std::string("default_video.log");

    log_error=false;
    fd=NULL;
    recording=false;
    frm_size=frame_size;
    sec=0;


    paknum=0;

}

VideoCam::~VideoCam()
{
    Close();
    printf("\nVideoCam: destroy\n");
}

bool VideoCam::Open()
{
    if((fd=fopen(f_name.data(),"w"))==NULL){
        printf("\nvideoCam: Error opening log-file %s\n",f_name.data());
        log_error=true;
        return false;
    }
    return true;
}

void VideoCam::Close()
{
    recording=false;
    if(fd)
        fclose(fd);

}


bool VideoCam::StartRecord()
{

    if(log_error){
        recording=false;
        return false;
    }

    if(fd==NULL && !Open()){
        recording=false;
        return false;
    }
    recording=true;
    rec_n_frames=-1;

    printf("\nvideoCam: Starting record\n");
    return true;
}


void VideoCam::StopRecord()
{
    recording=false;

    printf("\nvideoCam: Stoping record\n");
}

bool VideoCam::PushFrame(RGB24Pixel* data)
{

    if(!recording || log_error)
        return !log_error;

    VCFrameHdr hdr;

    timespec t_now;
    clock_gettime(CLOCK_MONOTONIC,&t_now);

    //timespec t_frame;
    //t_frame.tv_sec=tstamp->tv_sec;
    //t_frame.tv_nsec=((long int)tstamp->tv_usec)*1000;

    //double dt=Util::dift(&t_now,&t_frame);

    hdr.n=sec;
    hdr.time=GlobalTimer.GetTimer();

//    printf("\nPushF: %f\n",dt);

    fwrite(&hdr,sizeof(hdr),1,fd);

    fwrite(data,sizeof(RGB24Pixel),frm_size.w*frm_size.h,fd);

    if(rec_n_frames>=0){
        if((--rec_n_frames)<=0)
            StopRecord();

        //printf("\nVideoCam: %d frames left\n",rec_n_frames);
    }

    return true;

}


bool VideoCam::RecordNFrames(int f_num){

    bool r=StartRecord();
    rec_n_frames=f_num;
    return r;

}
}
