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

#include <iostream>

#include "rebvo.h"
#include "archimu.h"
#include "util.h"

#include "datasetcam.h"

void PrintHelp(){

    std::cout<<R"help(
               REBVO Commands:
               q: Quit
               r: Reset
               s: Start Save Video
               p: Take Snapshot
               )help";

}

int main(int argn,char ** argv)
{

    std::string ConfigName(argn>1?argv[1]:"GlobalConfig");



    REBVO cf(ConfigName.data());
	
    if(!cf.Init())
		return -1;

    archIMU *imu_dev=nullptr;

    DataSetCam cam("/home/juan/Datasets/MH_03_medium/mav0/cam0/data/","/home/juan/Datasets/MH_03_medium/mav0/cam0/data.csv",{752,480},1e-9);

    if(cf.getParams().ImuMode==1){


        imu_dev=new archIMU("/dev/ttySAC2",cf);

        if(imu_dev->error()){
            std::cout << "main.cpp: Failed to initialize the imu device\n";

            cf.CleanUp();
            return -1;
        }
    }

    PrintHelp();

    bool run=true;
    while(run && cf.Running()){

        double tstamp;
        std::shared_ptr <Image <RGB24Pixel> > ptr;
        RGB24Pixel* data=cam.GrabBuffer(tstamp);
        cf.requestCustomCamBuffer(ptr,tstamp);
        (*ptr).copyFrom(data);
        cam.ReleaseBuffer();
        cf.releaseCustomCamBuffer();
/*


        char c;
        std::cin >> c;

        switch(c){
        case 'q':
            run=false;
            std::cout << "\nExiting ...\n";
            break;
        case 's':
            cf.StartSimSave();
            break;
        case 'p':
            cf.TakeSnapshot();
            break;
        case 'r':
            cf.Reset();
            break;
        default:
            PrintHelp();
            break;
        }*/


    }


    cf.CleanUp();
    return 0;
}
