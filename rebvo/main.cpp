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

#include "util.h"



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


    Configurator GlobalConfig;

    //Leo el archivo de configuracion
    if(!GlobalConfig.ParseConfigFile(ConfigName.data(),false))
        return -1;


    REBVO cf(GlobalConfig);
	
    if(!cf.Init())
		return -1;

    PrintHelp();

    bool run=true;
    while(run && cf.Running()){

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
        }


    }

    cf.CleanUp();
    return 0;
}
