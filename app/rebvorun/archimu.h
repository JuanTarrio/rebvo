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

#ifndef ARCHIMU_H
#define ARCHIMU_H

#include <thread>
#include <mutex>
#include <atomic>
#include "UtilLib/configurator.h"
#include "UtilLib/imugrabber.h"
#include "rebvo/rebvo.h"

#pragma pack(push,1)

struct SensorFrame{
    unsigned char           Head1;
    unsigned char           Head2;
    unsigned char           Head3;
    unsigned char       	Opt;
    short int        	Reg;
    short int        	Giro[3];
    short int               Acel[3];
    unsigned short int	Crc;
};

#pragma pack(pop)

class archIMU
{


    int dsp_dev=-1;
    unsigned int rxerr=0;
    int rxnum=0;
    std::atomic_bool    quit;
    bool InitOk;

    rebvo::REBVO &grabber;
    std::thread readerThr;

    static void ioThread(archIMU *imu);
    bool    PushMeas(SensorFrame &sf);


public:
    archIMU(const char *device_name, rebvo::REBVO &rebvo_grab);

    void killIMU(){
        quit=false;
        readerThr.join();
    }

    bool error(){
        return !InitOk;
    }

};

#endif // ARCHIMU_H
