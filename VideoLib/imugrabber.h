#ifndef IMUGRABBER_H
#define IMUGRABBER_H

#include <TooN/TooN.h>
#include "CircList.h"
#include <stdexcept>
#include <utility>


struct ImuData{
    double tstamp;
    TooN::Vector<3> giro;
    TooN::Vector<3> acel;
    TooN::Vector<3> comp;

    ImuData(){}
    ImuData(const double t,const TooN::Vector<3> &g,const TooN::Vector<3> &a,const TooN::Vector<3> &c=TooN::Zeros)
        :tstamp(t)
    {
        comp=c;
        acel=a;
        giro=g;
    }
};

struct IntegratedImuData{

    int n=0;
    double dt=0;              //interval of time

    TooN::Matrix <3,3> Rot=TooN::Identity;
    TooN::Vector<3> giro=TooN::Zeros;
    TooN::Vector<3> acel=TooN::Zeros;
    TooN::Vector<3> comp=TooN::Zeros;
};

class ImuGrabber
{

    ImuData * imu;
    util::CircListIndexer write_inx;
    util::CircListIndexer read_inx;
    const int size;

    double tsample;

public:
    ImuGrabber(int list_size,double tsamp);
    ImuGrabber(const std::vector<ImuData> &data_set_data);
    ~ImuGrabber();

    static std::vector <ImuData> LoadDataSet( const char *data_file, bool comp_data,double time_scale,bool &error);

    void PushData(const ImuData&data){
        if(write_inx==read_inx)
            throw std::overflow_error("ImuGrabber circular buffer full");
        imu[write_inx]=data;
        ++write_inx;
    }

    std::pair <util::CircListIndexer,util::CircListIndexer> SeachByTimeStamp(double tstart,double tend);

    IntegratedImuData GrabAndIntegrate(double tstart,double tend);
};

#endif // IMUGRABBER_H
