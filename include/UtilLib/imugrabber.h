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

#ifndef IMUGRABBER_H
#define IMUGRABBER_H

#include <TooN/TooN.h>
#include <stdexcept>
#include <utility>
#include <thread>
#include <mutex>

#include "UtilLib/CircList.h"

namespace  rebvo{

//Structure to hold IMU data in vectorized circular buffer
struct ImuData{
    double tstamp=0;
    TooN::Vector<3> giro=TooN::Zeros;   //Giroscope 3 axis
    TooN::Vector<3> acel=TooN::Zeros;   //Acecelerometer
    TooN::Vector<3> comp=TooN::Zeros;   //Compass

    ImuData(){}
    ImuData(const double t,const TooN::Vector<3> &g,const TooN::Vector<3> &a,const TooN::Vector<3> &c=TooN::Zeros)
        :tstamp(t)
    {
        comp=c;
        acel=a;
        giro=g;
    }
};


//Structure to hold interframe integrated IMU data
struct IntegratedImuData{

    int n=0;
    double dt=0;              //interval of time

    TooN::Matrix <3,3> Rot=TooN::Identity;  //Interframe rotation
    TooN::Vector<3> giro=TooN::Zeros;       //Mean giro meassure
    TooN::Vector<3> acel=TooN::Zeros;       //Mean accel meassure
    TooN::Vector<3> comp=TooN::Zeros;       //Mean comp meassure
    TooN::Vector<3> dgiro=TooN::Zeros;      //Mean giroscope angular accelerartion
    TooN::Vector<3> cacel=TooN::Zeros;      //Compensated acceleration
};

class ImuGrabber
{

    ImuData * imu;                                      //Circular buffer
    util::CircListIndexer write_inx;                    //index of first slot to write to
    util::CircListIndexer read_inx;                     //index of first unread value
    const int size;

    double tsample;

    std::mutex rw_mut;

public:

    TooN::Matrix<3,3> RDataSetCam2IMU=TooN::Identity;   //SE3 transformation Camera to Imu, Pimu=RCam2Imu*Pcam+TCam2Imu
    TooN::Vector<3> TDataSetCam2IMU=TooN::Zeros;

public:
    ImuGrabber(int list_size,double tsamp);
    ImuGrabber(const std::vector<ImuData> &data_set_data);
    ~ImuGrabber();

    static std::vector <ImuData> LoadDataSet( const char *data_file, bool comp_data,double time_scale,bool &error);

    bool LoadCamImuSE3(const char *se3_file);
    bool LoadCamImuSE3(const TooN::Matrix<3,3> &RCam2IMU,const TooN::Vector<3> &TCam2IMU);

    bool PushData(const ImuData&data);

    std::pair <util::CircListIndexer,util::CircListIndexer> SeachByTimeStamp(double tstart,double tend);

    IntegratedImuData GrabAndIntegrate(double tstart,double tend);

};

}

#endif // IMUGRABBER_H
