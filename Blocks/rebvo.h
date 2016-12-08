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


#ifndef CAMARAFRONTAL_H
#define CAMARAFRONTAL_H

#include "video_io.h"
#include "configurator.h"

#include <string>

#include <TooN/TooN.h>

#include <thread>
#include <atomic>


#include "v4lcam.h"
#include "simcam.h"


#include "net_keypoint.h"

#include "edge_finder.h"

#include "video_mfc.h"
#include "video_mjpeg.h"

#include "edge_tracker.h"
#include "pipeline.h"

#include "udp_port.h"
#include "global_tracker.h"
#include "pipeline.h"
#include "keyframe.h"
#include "imugrabber.h"


#define CBUFSIZE        0x08

//#define TIME_DEBUG

#ifdef TIME_DEBUG
#define COND_TIME_DEBUG(arg) arg
#else
#define COND_TIME_DEBUG(arg)
#endif


//Structure to save variables of the different estimation stages
struct IMUState{
    TooN::Vector <3> Vg=TooN::Zeros;                //IMU Stages velocity and rotation

    TooN::Vector <3> dVv=TooN::Zeros;
    TooN::Vector <3> dWv=TooN::Zeros;

    TooN::Vector <3> dVgv=TooN::Zeros;
    TooN::Vector <3> dWgv=TooN::Zeros;

    TooN::Vector <3> Vgv=TooN::Zeros;
    TooN::Vector <3> Wgv=TooN::Zeros;


    TooN::Vector <3> dVgva=TooN::Zeros;
    TooN::Vector <3> dWgva=TooN::Zeros;
    TooN::Vector <3> Vgva=TooN::Zeros;


    TooN::Matrix <3,3> P_Vg=TooN::Identity*1e50;    //IMU Stages velocity and rotation covariances


    TooN::Matrix <3,3> RGiro=TooN::Identity;
    TooN::Matrix <3,3> RGBias=TooN::Identity;

    TooN::Vector <3>   Bg=TooN::Zeros;              //Giroscope bias
    TooN::Matrix <3,3> W_Bg=TooN::Identity;         //Giroscope bias covariance


    TooN::Vector <3> Av=TooN::Zeros;                //visual acceleration
    TooN::Vector <3> As=TooN::Zeros;                //accelerometer acceleration

    TooN::Vector <7> X;
    TooN::Matrix <7,7> P;
    TooN::Matrix <3,3> Qrot;
    TooN::Matrix <3,3> Qg;
    TooN::Matrix <3,3> Qbias;
    double QKp;
    double Rg;
    TooN:: Matrix <3,3> Rs;
    TooN::Matrix <3,3> Rv;
    TooN::Vector<3> g_est;
    TooN::Vector<3> u_est;
    TooN::Vector<3> b_est;
    TooN::Matrix <6,6> Wvw;
    TooN::Vector <6> Xvw;


    TooN::Vector<3> Posgv=TooN::Zeros;
    TooN::Vector<3> Posgva=TooN::Zeros;

    bool init=false;
};

//Pipeline state buffer

struct PipeBuffer{

    sspace * ss;
    global_tracker *gt;
    edge_tracker *ef;
    Image<RGB24Pixel> *imgc;
    Image<float> *img;
    double t;
    double dt;

    double s_rho_p;

    TooN::Matrix <3,3> Rot;
    TooN::Vector <3> RotLie;
    TooN::Vector <3> Vel;


    TooN::Matrix <3,3> Pose;
    TooN::Vector <3> PoseLie;
    TooN::Vector <3> Pos;

    IMUState imustate;

    double dtp0;

    double K;
    double Kp;
    double RKp;

    int p_id;

    bool EstimationOK;

    bool quit;

    IntegratedImuData imu;

};




class REBVO
{

    std::thread Thr0;

    bool InitOK=true;


    //Pipeline and multithead

    std::atomic_bool    quit;
    Pipeline <PipeBuffer> pipe;
    std::atomic_bool start_record;


    std::atomic_bool saveImg;
    int snap_n=0;


    std::atomic_bool system_reset;

    double time;
    int encoder_type;
    std::string	encoder_dev;


    std::string SimFile;
    std::string CameraDevice;
    std::string	VideoNetHost;
    int	VideoNetPort;
    int VideoNetEnabled;

    double sim_save_nframes;
    int CameraType;

    ImuGrabber *imu=nullptr;


    //DSCam param

    std::string DataSetFile;
    std::string DataSetDir;

    //Camara parameters
    cam_model *cam;
    double config_fps;
    double CamTimeScale;

    //IMU parameters

    int ImuMode;
    std::string ImuFile;
    std::string SE3File;
    double ImuTimeScale;
    double GiroMeasStdDev;
    double GiroBiasStdDev;
    bool InitBias;
    int InitBiasFrameNum;
    double g_module;


    double AcelMeasStdDev;
    double g_module_uncer;
    double g_uncert;
    double VBiasStdDev;
    double ScaleStdDevMult;
    double ScaleStdDevMax;
    double ScaleStdDevInit;


    //Processor parameters

    int cpu0;
    int cpu1;
    int cpu2;
    int cpuSetAffinity;

    //Trayectory parameters

    TooN::Vector<3> pos_scaled;
    TooN::Vector<3> pose_lie;
    TooN::Vector<3> WCam;


    bool new_frame_proc;

    double t_frame=0;
    double dt_frame=0.020;

    timeval T0;

    int VideoSave;
    std::string VideoSaveFile;
    int VideoSaveBuffersize;


    std::string LogFile;
    std::string TrayFile;
    bool SaveLog;
    uint EdgeMapDelay;

    //Detector parameters

    double Sigma0;
    double Sigma1;

    int DetectorPlaneFitSize;
    double DetectorPosNegThresh;
    double DetectorDoGThresh;

    int ReferencePoints;
    int MaxPoints;

    double DetectorThresh;
    double DetectorAutoGain;
    double DetectorMaxThresh;
    double DetectorMinThresh;

    //Tracker-Mapper Parameters

    int MatchThreshold;

    double SearchRange;

    double QCutOffNumBins;
    double QCutOffQuantile;

    int TrackerIterNum;
    int TrackerInitIterNum;
    int TrackerInitType;

    double TrackerMatchThresh;

    double LocationUncertaintyMatch;
    double MatchThreshModule;
    double MatchThreshAngle;

    double ReweigthDistance;

    uint MatchNumThresh;

    double RegularizeThresh;
    double ReshapeQAbsolute;
    double ReshapeQRelative;
    double LocationUncertainty;
    double DoReScaling;



public:
    bool	Init();

    static void ThirdThread(REBVO *cf);
    static void SecondThread(REBVO *cf);
    static void	FirstThr(REBVO *cf);

    bool CleanUp();
    REBVO(Configurator &config);
    ~REBVO();


    void StartSimSave(){start_record=true;}
    void TakeSnapshot(){saveImg=true;}
    void Reset(){system_reset=true;}

    bool Running(){return !quit;}


    //Keyframe list

    std::vector <keyframe> kf_list;
};

#endif // CAMARAFRONTAL_H
