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

#include "rebvo/rebvo.h"
#include "archimu.h"
#include "UtilLib/util.h"


using namespace rebvo;

void PrintHelp(){

    std::cout<<R"help(
               REBVO Commands:
               q: Quit
               r: Reset
               s: Save KF and Quit
               p: Take Snapshot
               f: Frame by frame
               a: Advance frame
               )help";

}

class callclass{
public:

    bool callFunc(PipeBuffer &p){
        //do something on the timespan of the rebvo third thread
        return true;
    }
};



int main(int argn,char ** argv)
{

    std::string ConfigName(argn>1?argv[1]:"GlobalConfig");

    callclass callme;


    REBVO cf(ConfigName.data());
	
    if(!cf.Init())
		return -1;

    archIMU *imu_dev=nullptr;

    if(cf.getParams().ImuMode==1){


        imu_dev=new archIMU("/dev/ttySAC2",cf);

        if(imu_dev->error()){
            std::cout << "main.cpp: Failed to initialize the imu device\n";

            cf.CleanUp();
            return -1;
        }
    }

    cf.setOutputCallback(&callclass::callFunc,&callme);

    PrintHelp();

    bool run=true;
    bool savekf=false;
    while(run && cf.Running()){


        char c;
        std::cin >> c;

        switch(c){
        case 'q':
            run=false;
            std::cout << "\nExiting ...\n";
            break;
        case 's':
            run=false;
            savekf=true;
            std::cout << "\nExiting ...\n";
            break;
        case 'p':
            cf.TakeSnapshot();
            break;
        case 'r':
            cf.Reset();
            break;
        case 'k':
            cf.toggleKeyFrames();
            break;
        case 'f':
            cf.toggleFrameByFrame();
            cf.advanceFrameByFrame();
            break;
        case 'a':
            cf.advanceFrameByFrame();
            break;
        default:
            PrintHelp();
            break;
        }


    }

    if(savekf){
        std::cout <<"Saving KF: "<<(keyframe::saveKeyframes2File("kf_list.kf",cf.kf_list)?"OK":"Error")<<"\n";
    }

    cf.CleanUp();
    return 0;
}

