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

#include "archimu.h"

#include "UtilLib/ttimer.h"
#include "UtilLib/libcrc.h"
#include "UtilLib/util.h"


#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>
#include <iomanip>

#include <termios.h>
#include <string>

#define GiroSens    (1.5271e-04)
#define AcelSens    (9.81/250.0)


using namespace rebvo;

void archIMU::ioThread(archIMU *imu){



    SensorFrame sf;
    unsigned char sf_buf[sizeof(SensorFrame)];

    int rxpos=0;

    sf.Head1=0x7F;
    sf.Head2=0x7E;
    sf.Head3=0x7D;

    int out_send=0;

    char obuf[4];
    obuf[0]=4;
    obuf[1]=1;
    obuf[2]=0;
    obuf[3]=obuf[2]+obuf[1]+obuf[0];
    int r=write(imu->dsp_dev,obuf,4);
    std::cout <<"\nADQ: ImuOn! send:"<<r<<"\n";

    timespec t0,t1;
    clock_gettime(CLOCK_MONOTONIC,&t0);


    while(!imu->quit){


        int l=read(imu->dsp_dev,&sf_buf,sizeof(SensorFrame)-rxpos);


        clock_gettime(CLOCK_MONOTONIC,&t1);


        if(l<=0){
            if(util::dift(t1,t0)>1){

                char obuf[4];

                for (int i=0;i<4;i++){
                    obuf[0]=i;
                    obuf[1]=0;
                    obuf[2]=0;
                    obuf[3]=i;
                    write(imu->dsp_dev,obuf,4);
                }

                obuf[0]=4;
                obuf[1]=1;
                obuf[2]=0;
                obuf[3]=obuf[2]+obuf[1]+obuf[0];
                int r=write(imu->dsp_dev,obuf,4);
                std::cout <<"\nADQ: ImuOn! send:"<<r<<"\n";
                t0=t1;
            }
            continue;
        }
        t0=t1;


        for(int sf_pos=0;l>0;l--,sf_pos++){

            switch(rxpos){
            case 0:
                if(sf_buf[sf_pos]==sf.Head1){
                    rxpos++;
                }else{
                    imu->rxerr++;
                }
                break;
            case 1:
                if(sf_buf[sf_pos]==sf.Head2){
                    rxpos++;
                }else{
                    rxpos=0;
                    imu->rxerr++;
                }
                break;
            case 2:
                if(sf_buf[sf_pos]==sf.Head3){
                    rxpos++;
                }else{
                    rxpos=0;
                    imu->rxerr++;
                }
                break;
            default:


                ((char *)&sf)[rxpos++]=sf_buf[sf_pos];

                if((rxpos)>=sizeof(SensorFrame)){


                    ushort crc=util::CRC16((u_char*)&sf,sizeof(SensorFrame)-sizeof(ushort));
                    if(crc==sf.Crc){

                        imu->PushMeas(sf);

                        imu->rxnum++;

                        out_send=(out_send+1)%4;
                        char obuf[4];
                        obuf[0]=out_send;
                        obuf[1]=0;
                        obuf[2]=0;
                        obuf[3]=obuf[2]+obuf[1]+obuf[0];
                        write(imu->dsp_dev,obuf,4);

                    }else{
                        imu->rxerr++;
                    }

                    rxpos=0;



                }
                break;
            }


        }
    }



    return;


}


bool    archIMU::PushMeas(SensorFrame &sf){


    static ImuData data;

    timespec ts;
    clock_gettime(CLOCK_MONOTONIC,&ts);
    data.tstamp=(double)ts.tv_sec+(double)ts.tv_nsec*1e-9;


    for(int i=0;i<3;i++){
        data.giro[i]=sf.Giro[i]*GiroSens;
        data.acel[i]=sf.Acel[i]*AcelSens;
    }

    return grabber.pushIMU(data);


}


archIMU::archIMU(const char *device_name, REBVO &rebvo_grab)
    :grabber(rebvo_grab)
{

    InitOk=false;
    dsp_dev=open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if(dsp_dev==-1){
        std::cout <<"\narchIMU: cannot open serial device!\n";
        return;
    }


    struct termios newtio;

    newtio.c_cflag = B230400 | CS8  | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VMIN]=0;
    newtio.c_cc[VTIME]=10;
    tcflush(dsp_dev, TCIFLUSH);
    tcsetattr(dsp_dev,TCSANOW,&newtio);

    quit=false;
    readerThr=std::thread (ioThread,this);
    InitOk=true;
    std::cout <<"\narchIMU: imu started correctly!\n";
}
