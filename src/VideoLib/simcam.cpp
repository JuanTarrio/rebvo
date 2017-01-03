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

#include "VideoLib/simcam.h"
#include "UtilLib/ttimer.h"
#include <string.h>
namespace  rebvo{

simcam::simcam(const char *sim_name, Size2D frame_size, const char *log_name)
    :VideoCam(log_name,frame_size)
{

    buffer=new RGB24Pixel[frame_size.w*frame_size.h];

    if((fd=fopen(sim_name,"r"))==NULL){
        printf("\nSimCam: Error opening sim-file %s\n",sim_name);
        error=true;
        fd=NULL;
        return;
    }

    frm_pending=false;
    paknum=0;

}

simcam::~simcam(){

    if(fd)
        fclose(fd);
    delete [] buffer;



}

int simcam::WaitFrame(bool drop_frames){
    if(error)
        return -1;

    while(1){

        int r=fread(&hdr,1,sizeof(hdr),fd);

        if(r!=sizeof(hdr)){
            printf("\nSimCam: error reading sim-file after %d packets\n",paknum);
            error=true;
            return -1;
        }

        r=fread(buffer,sizeof(RGB24Pixel),frm_size.w*frm_size.h,fd);


        if(r!=frm_size.w*frm_size.h){
            printf("\nSimCam: error reading sim-file (wrong format) after %d packets\n",paknum);
            error=true;
            return -1;
        }

        //printf("\n1:%d %f %f\n",paknum,hdr.time,GlobalTimer.GetTimer());

        paknum++;

        if(hdr.time>GlobalTimer.GetTimer() || !drop_frames)
            break;
    }

    while(hdr.time>GlobalTimer.GetTimer());

   // printf("\n0: %f %f\n",hdr.time,GlobalTimer.GetTimer());

    frm_pending=true;

    return 0;

}

int simcam::GrabFrame(RGB24Pixel *data, double &tstamp, bool drop_frames){

    if(!frm_pending)
        if(WaitFrame(drop_frames)<0)
            return -1;

    frm_pending=false;

    memcpy(data,buffer,sizeof(RGB24Pixel)*frm_size.w*frm_size.h);

    tstamp=hdr.time;

    return 0;
}

RGB24Pixel* simcam::GrabBuffer(double &tstamp, bool drop_frames){

    if(!frm_pending)
        if(WaitFrame(drop_frames)<0)
            return 0;


    frm_pending=false;


    tstamp=hdr.time;

    return buffer;
}

int simcam::ReleaseBuffer(){
    return 0;
}
}
