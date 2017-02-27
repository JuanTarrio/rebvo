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


#ifndef REBVO_H
#define REBVO_H


#include <string>
#include <TooN/TooN.h>
#include <thread>
#include <atomic>




#include "UtilLib/pipeline.h"
#include "mtracklib/edge_tracker.h"
#include "mtracklib/global_tracker.h"
#include "mtracklib/keyframe.h"
#include "UtilLib/imugrabber.h"
#include "VideoLib/customcam.h"
#include <functional>


namespace  rebvo{



constexpr int CBUFSIZE=0x08;
constexpr int CCAMBUFSIZE=0x04;

//#define TIME_DEBUG

#ifdef TIME_DEBUG
#define COND_TIME_DEBUG(arg) arg
#else
#define COND_TIME_DEBUG(arg)
#endif



struct REBVOParameters{



    //REBVO params
    int CameraType;                     //0=V4L, 1=SimCam, 2=Dataset cam, 3=custom cam



    std::string	VideoNetHost;           //Remote host to send image, keyline and nav data (where the visualizer is run)
    int	VideoNetPort;                   //Remote port
    int VideoNetEnabled;                //Enable video tranmission?
    int BlockingUDP;                    //Use blocking sokets? turn on if lossing packests on transmition, may slow down the system


    int VideoSave;                      //Optionally save video after running
    std::string VideoSaveFile;          //Video file to save to
    int VideoSaveBuffersize;            //Buffer size


    int encoder_type;                   //Video encoder type, 0=raw,1=Motion JPEG, 2=Samsung MFC
    std::string	encoder_dev;            //If using Samsung MFC encoder, encoder device
    uint EdgeMapDelay;                  //Frame delay between video and edge-maps


    bool SaveLog;                       //Save log on-off
    std::string LogFile;                //Log file in .m format
    std::string TrayFile;               //Trayectory in TUM dataset format [t pos quat]

    bool StereoAvaiable;                //Flag to determine if stereo data is avaiable


    //DataSetCam param

    std::string DataSetFile;            //The directory of the dataset
    std::string DataSetDir;             //The file of the dataset
    std::string DataSetFileStereo;      //The directory of the dataset
    std::string DataSetDirStereo;       //The file of the dataset
    double CamTimeScale;                //Timescale multiplier for the dataset file

    //Camara parameters


    Size2D ImageSize;                   //Frame size
    float z_f_x;                        //Camera XY focal length
    float z_f_y;
    float pp_y;                         //Camera principal point
    float pp_x;
    cam_model::rad_tan_distortion kc;   //Distortion parameters


    double config_fps;                  //Frames per second
    double soft_fps;                    //Forced frames per second
    int useUndistort;                   //Use undistortion
    bool rotatedCam;                    //Rotate camera 180deg


    std::string CameraDevice;           //V4L Camera device


    float z_f_x_stereo;                        //Camera XY focal length
    float z_f_y_stereo;
    float pp_y_stereo;                         //Camera principal point
    float pp_x_stereo;
    cam_model::rad_tan_distortion kc_stereo;   //Distortion parameters

    //simcam and simulation parameters
    std::string SimFile;                    //SimCam video file
    double sim_save_nframes;                //Number of frames to save for simulation (uncompressed video)

    int simu_time_on;                       //Simulate time or use system time
    int simu_time_step;                     //Timestep in nanosecons
    double simu_time_sweep;                 //Simulated time sweep
    double simu_time_start;                 //Simulated time start


    //IMU parameters


    int ImuMode;                            //0=no imu, 1= use the archquitecture specific class, 2=load full data from ImuFile
    std::string ImuFile;                    //Imu file
    bool UseCamIMUSE3File;
    std::string SE3File;                    //File containing the SE3 transformation from IMU to Camera
    double ImuTimeScale;                    //TimeScale of the IMU file

    bool InitBias;                          //0=Use initial guess, 1=use InitBiasFrameNum frames to estimate bias
    int InitBiasFrameNum;

    TooN::Vector <3> BiasInitGuess;         //Bias initial guess in camera frame

    double GiroMeasStdDev;                  //Giro Noise Std Dev
    double GiroBiasStdDev;                  //Giro Bias Random Walk Noise
    double AcelMeasStdDev;                  //Accelerometer Noise Std Dev
    double g_module;                        //meassured gravity module

    double g_module_uncer;                  //Uncertainty in the G module, can be keeped at a big value
    double g_uncert;                        //Process uncertainty on the G vector
    double VBiasStdDev;                     //Process uncertainty in the visual bias estimation (keep small)
    double ScaleStdDevMult;                 //Scale process uncertinty in relation to visual, USE THIS PARAMETER TO TUNE FILTER RESPONSE TIME
    double ScaleStdDevMax;                  //Max Scale process uncertinty, should leave fixed
    double ScaleStdDevInit;                 //Initial uncertainty for the scale


    double SampleTime;                  //Sample time for custom IMU
    int CircBufferSize;                 //Imu grabber circular buffer size

    double TimeDesinc;                  //Time desincrozitation IMU2Cam (sec)


    //Processor parameters

    int cpuSetAffinity;             //Switch to set afinity on off
    int cpu0;                       //Processor threads affiniy
    int cpu1;
    int cpu2;



    //Detector parameters

    double Sigma0;                  //The  scale and multiplier used for DoG calculation (Sigma1~Sigma0*k_sigma)
    double KSigma;

    int DetectorPlaneFitSize;       //Window size for plane fitting to the DoG = (DetectorPlaneFitSize*2+1)^2
    double DetectorPosNegThresh;    //Max percentual difference for DoG nonmaximal suppresion
    double DetectorDoGThresh;       //Relation between DoG threshold and Gradient threshold ~1/Sigma0^4

    int ReferencePoints;            //Reference to the number of points when autothreshold
    int MaxPoints;                  //Absolute maximun number of points

    double DetectorThresh;          //Manual theshold
    double DetectorAutoGain;        //Auto threshold gain, 0=manual
    double DetectorMaxThresh;       //Limits for autothreshold
    double DetectorMinThresh;


    //Tracker-Mapper Parameters


    int MatchThreshold;                     //Minimun number of keyline matches required for further procesing

    double SearchRange;                      //Pixel range for tracking and mapping

    double QCutOffNumBins;                  //Number of bins on the histogram for percentile calculation
    double QCutOffQuantile;                 //Percentile of the KLs to use

    int TrackerIterNum;                     //Tracker number of iterations
    int TrackerInitIterNum;                 //Double ititialization iteration number
    int TrackerInitType;                    //Tracker Initialization prior (0=zero,1=last frame,2=try both)

    double TrackerMatchThresh;              //Tracking thesh on the scalar product

    double LocationUncertaintyMatch;
    double MatchThreshModule;               //Matching thesh on the gradient module
    double MatchThreshAngle;                //Matching thesh on the gradient angle (degrees)

    double ReweigthDistance;                //Reweigh error hubber residual

    uint MatchNumThresh;                    //Minimun number of matches for tracking

    double RegularizeThresh;                //Regularization threshold on the gradient
    double ReshapeQAbsolute;                //EKF Modelled absolute error on Inv-Depth
    double ReshapeQRelative;                //EKF Modelled relative error on Inv-IDepth
    double LocationUncertainty;             //Modelled Pixel uncertainty on the matching step
    double DoReScaling;                     //Apply re-scaling after EKF


};


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

struct NavData{
    double t;
    double dt;
    double scale;

    TooN::Matrix <3,3> Rot=TooN::Identity;
    TooN::Vector <3> RotLie=TooN::Zeros;
    TooN::Vector <3> RotGiro=TooN::Zeros;
    TooN::Vector <3> Vel=TooN::Zeros;


    TooN::Vector <3> g=TooN::Zeros;

    TooN::Matrix <3,3> Pose=TooN::Identity;
    TooN::Vector <3> PoseLie=TooN::Zeros;
    TooN::Vector <3> Pos=TooN::Zeros;
};

//Pipeline state buffer

struct PipeBuffer{

    sspace * ss;
    global_tracker *gt;
    edge_tracker *ef;
    Image<RGB24Pixel> *imgc;
    Image<float> *img;


    sspace * ss_pair;
    edge_tracker *ef_pair;
    Image<RGB24Pixel> *imgc_pair;
    Image<float> *img_pair;

    double t;
    double dt;

    double s_rho_p;

    NavData nav;
    IMUState imustate;

    double dtp0;
    double dtp1;

    double K;
    double Kp;
    double RKp;

    int p_id;

    bool EstimationOK;

    bool quit;

    int stereo_match_num;

    IntegratedImuData imu;

};


//typedef bool outputCallback(PipeBuffer &data);


class REBVO
{

    REBVOParameters params;

    std::thread Thr0;
    bool InitOK=true;

    std::mutex nav_mutex;
    NavData nav;


    std::atomic_bool saveKeyframes;


    //Pipeline and multithead

    std::atomic_bool    quit;
    Pipeline <PipeBuffer> pipe;
    std::atomic_bool start_record;


    std::atomic_bool saveImg;
    int snap_n=0;
    std::atomic_bool system_reset;


    //Custom cam pipeline
    Pipeline <customCam::CustomCamPipeBuffer> cam_pipe;
    Pipeline <customCam::CustomCamPipeBuffer> cam_pipe_stereo;

    //camera model
    cam_model cam;


    cam_model cam_stereo;
    //Imu grabber

    ImuGrabber *imu=nullptr;

    //output callback

    std::mutex call_mutex;
    std::function<bool(PipeBuffer &)> outputFunc;

    bool callCallBack(PipeBuffer & pbuf){
        std::lock_guard<std::mutex> locker(call_mutex);

        if(outputFunc)
            return outputFunc(pbuf);
        return true;
    }

    void pushNav(const NavData &navdat){
        std::lock_guard<std::mutex> locker(nav_mutex);
        nav=navdat;
    }

    void construct();

    VideoCam * initCamera();
    VideoCam *initPairCamera();

    static bool setAffinity(int cpu){
        cpu_set_t cpusetp;

        CPU_ZERO(&cpusetp);
        CPU_SET(cpu, &cpusetp);
        return pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpusetp) == 0;
    }


public:
    bool	Init();

    static void ThirdThread(REBVO *cf);
    static void SecondThread(REBVO *cf);
    static void	FirstThr(REBVO *cf);

    bool CleanUp();
    REBVO(const char *configFile);
    REBVO(const REBVOParameters &parameters);
    ~REBVO();


    void StartSimSave(){start_record=true;}
    void TakeSnapshot(){saveImg=true;}
    void Reset(){system_reset=true;}

    bool Running(){return !quit;}

    void startKeyFrames(){

        saveKeyframes=true;
    }

    void endKeyFrames(){

        saveKeyframes=false;
    }
    bool toggleKeyFrames(){

        return saveKeyframes=!saveKeyframes;
    }

    //Keyframe list

    std::vector <keyframe> kf_list;

    //Get nav data (saved on the second thread)
    NavData getNav(){
        std::lock_guard<std::mutex> locker(nav_mutex);
        NavData navdat=nav;
        return navdat;
    }

    //Get REBVO params
    const REBVOParameters& getParams(){
        return params;
    }

    //Set transformation from IMU to Camera

    bool setCamImuSE3(const TooN::Matrix<3,3> &RCam2IMU,const TooN::Vector<3> &TCam2IMU){
        if(!quit)
            return false;
        if(imu)
            return imu->LoadCamImuSE3(RCam2IMU,TCam2IMU);

        return false;
    }

    //add timestamped IMU data to the pipeline (thread safe)
    bool pushIMU(const ImuData&data){

        if(imu)
            return imu->PushData(data);
        return false;
    }

    //request a ptr to an Image object to push image on the pipeline of the custom camera
    bool requestCustomCamBuffer(std::shared_ptr<Image <RGB24Pixel> > &ptr,double time_stamp,double timeout_secs=0){

        customCam::CustomCamPipeBuffer *ccpb=cam_pipe.RequestBufferTimeoutable(0,timeout_secs);
        if(ccpb==nullptr)
            return false;
        ptr=(*ccpb).img;
        (*ccpb).timestamp=time_stamp;
        return true;
    }


    //release the pointer for image loading on the customcam buffer
    void releaseCustomCamBuffer(){

        cam_pipe.ReleaseBuffer(0);
    }

    //request a ptr to an Image object to push image on the pipeline of the custom camera
    bool requestStereoCustomCamBuffer(std::shared_ptr<Image <RGB24Pixel> > &ptr,double time_stamp,double timeout_secs=0){

        customCam::CustomCamPipeBuffer *ccpb=cam_pipe_stereo.RequestBufferTimeoutable(0,timeout_secs);
        if(ccpb==nullptr)
            return false;
        ptr=(*ccpb).img;
        (*ccpb).timestamp=time_stamp;
        return true;
    }


    //release the pointer for image loading on the customcam buffer
    void releaseStereoCustomCamBuffer(){

        cam_pipe_stereo.ReleaseBuffer(0);
    }

    //set a callback funtion to be called on the third thread with a reference to the pipebuffer containing all the algorithm output
    //call with nullptr to release callback
    template <typename T>
    void setOutputCallback(bool(T::*method)(PipeBuffer &), T * obj){
        std::lock_guard<std::mutex> locker(call_mutex);
        outputFunc=std::bind(method,obj,std::placeholders::_1);
    }

    void setOutputCallback(bool(*func)(PipeBuffer &) ){
        std::lock_guard<std::mutex> locker(call_mutex);
        outputFunc=std::bind(func,std::placeholders::_1);
    }

	bool isInitOk() const {
		return InitOK;
    }

    TooN::Matrix <3,3> getCam2ImuRot(){
        if(imu)
            return imu->RDataSetCam2IMU;
        else
            return TooN::Identity;
    }
    TooN::Vector <3> getCam2ImuPos(){
        if(imu)
            return imu->TDataSetCam2IMU;
        else
            return TooN::Zeros;
    }
};

}

#endif // REBVO_H
