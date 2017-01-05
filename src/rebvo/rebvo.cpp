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



#include <TooN/so3.h>

#include <iostream>
#include <iomanip>
#include <TooN/Cholesky.h>

#include "rebvo/rebvo.h"
#include "mtracklib/scaleestimator.h"
#include "UtilLib/ttimer.h"
#include "VideoLib/datasetcam.h"
#include "UtilLib/configurator.h"



using namespace std;
namespace  rebvo{


REBVO::REBVO(const char *configFile)
    :pipe(CBUFSIZE,4),cam_pipe(CCAMBUFSIZE,2),outputFunc(nullptr)
{


    Configurator config;

    //Leo el archivo de configuracion
    if(!(InitOK=config.ParseConfigFile(configFile,false)))
        return;


    InitOK&=config.GetConfigByName("Camera","CameraDevice",params.CameraDevice,true);
    InitOK&=config.GetConfigByName("REBVO","CameraType",params.CameraType,true);


    InitOK&=config.GetConfigByName("SimuCamera","SimVideoFile",params.SimFile,true);
    InitOK&=config.GetConfigByName("SimuCamera","SimVideoNFrames",params.sim_save_nframes,true);

    InitOK&=config.GetConfigByName("SimuCamera","SimuTimeOn",params.simu_time_on,true);
    InitOK&=config.GetConfigByName("SimuCamera","SimuTimeSweep",params.simu_time_sweep,true);
    InitOK&=config.GetConfigByName("SimuCamera","SimuTimeStep",params.simu_time_step,true);
    InitOK&=config.GetConfigByName("SimuCamera","SimuTimeStart",params.simu_time_start,true);


    InitOK&=config.GetConfigByName("DataSetCamera","DataSetDir",params.DataSetDir,true);
    InitOK&=config.GetConfigByName("DataSetCamera","DataSetFile",params.DataSetFile,true);
    InitOK&=config.GetConfigByName("DataSetCamera","TimeScale",params.CamTimeScale,true);


    InitOK&=config.GetConfigByName("REBVO","VideoNetHost",params.VideoNetHost,true);
    InitOK&=config.GetConfigByName("REBVO","VideoNetPort",params.VideoNetPort,true);
    InitOK&=config.GetConfigByName("REBVO","VideoNetEnabled",params.VideoNetEnabled,true);
    InitOK&=config.GetConfigByName("REBVO","BlockingUDP",params.BlockingUDP,true);

    InitOK&=config.GetConfigByName("Camera","ImageWidth",params.ImageSize.w,true);
    InitOK&=config.GetConfigByName("Camera","ImageHeight",params.ImageSize.h,true);

    InitOK&=config.GetConfigByName("Camera","ZfX",params.z_f_x,true);
    InitOK&=config.GetConfigByName("Camera","ZfY",params.z_f_y,true);

    InitOK&=config.GetConfigByName("Camera","PPx",params.pp_x,true);
    InitOK&=config.GetConfigByName("Camera","PPy",params.pp_y,true);

    InitOK&=config.GetConfigByName("Camera","KcR2",params.kc.Kc2,true);
    InitOK&=config.GetConfigByName("Camera","KcR4",params.kc.Kc4,true);
    InitOK&=config.GetConfigByName("Camera","KcR6",params.kc.Kc6,true);
    InitOK&=config.GetConfigByName("Camera","KcP1",params.kc.P1,true);
    InitOK&=config.GetConfigByName("Camera","KcP2",params.kc.P2,true);
    InitOK&=config.GetConfigByName("Camera","UseUndistort",params.useUndistort,true);
    InitOK&=config.GetConfigByName("Camera","Rotate180",params.rotatedCam,true);

    InitOK&=config.GetConfigByName("Camera","FPS",params.config_fps,true);
    InitOK&=config.GetConfigByName("ProcesorConfig","SetAffinity",params.cpuSetAffinity,true);
    InitOK&=config.GetConfigByName("ProcesorConfig","CamaraT1",params.cpu0,true);
    InitOK&=config.GetConfigByName("ProcesorConfig","CamaraT2",params.cpu1,true);
    InitOK&=config.GetConfigByName("ProcesorConfig","CamaraT3",params.cpu2,true);
    InitOK&=config.GetConfigByName("REBVO","EncoderType",params.encoder_type,true);
    InitOK&=config.GetConfigByName("REBVO","EncoderDevice",params.encoder_dev,true);

    InitOK&=config.GetConfigByName("Detector","Sigma0",params.Sigma0,true);
    InitOK&=config.GetConfigByName("Detector","Sigma1",params.Sigma1,true);
    InitOK&=config.GetConfigByName("Detector","ReferencePoints",params.ReferencePoints,true);
    InitOK&=config.GetConfigByName("Detector","MaxPoints",params.MaxPoints,true);
    InitOK&=config.GetConfigByName("Detector","DetectorThresh",params.DetectorThresh,true);
    InitOK&=config.GetConfigByName("Detector","DetectorAutoGain",params.DetectorAutoGain,true);

    InitOK&=config.GetConfigByName("Detector","DetectorMaxThresh",params.DetectorMaxThresh,true);
    InitOK&=config.GetConfigByName("Detector","DetectorMinThresh",params.DetectorMinThresh,true);

    InitOK&=config.GetConfigByName("Detector","DetectorPlaneFitSize",params.DetectorPlaneFitSize,true);
    InitOK&=config.GetConfigByName("Detector","DetectorPosNegThresh",params.DetectorPosNegThresh,true);
    InitOK&=config.GetConfigByName("Detector","DetectorDoGThresh",params.DetectorDoGThresh,true);

    InitOK&=config.GetConfigByName("TrackMaper","SearchRange",params.SearchRange,true);
    InitOK&=config.GetConfigByName("TrackMaper","QCutOffNumBins",params.QCutOffNumBins,true);
    InitOK&=config.GetConfigByName("TrackMaper","QCutOffQuantile",params.QCutOffQuantile,true);
    InitOK&=config.GetConfigByName("TrackMaper","TrackerIterNum",params.TrackerIterNum,true);
    InitOK&=config.GetConfigByName("TrackMaper","TrackerInitIterNum",params.TrackerInitIterNum,true);
    InitOK&=config.GetConfigByName("TrackMaper","TrackerInitType",params.TrackerInitType,true);
    InitOK&=config.GetConfigByName("TrackMaper","TrackerMatchThresh",params.TrackerMatchThresh,true);
    InitOK&=config.GetConfigByName("TrackMaper","MatchThreshModule",params.MatchThreshModule,true);
    InitOK&=config.GetConfigByName("TrackMaper","MatchThreshAngle",params.MatchThreshAngle,true);
    InitOK&=config.GetConfigByName("TrackMaper","MatchNumThresh",params.MatchNumThresh,true);
    InitOK&=config.GetConfigByName("TrackMaper","RegularizeThresh",params.RegularizeThresh,true);
    InitOK&=config.GetConfigByName("TrackMaper","ReweigthDistance",params.ReweigthDistance,true);

    InitOK&=config.GetConfigByName("TrackMaper","ReshapeQAbsolute",params.ReshapeQAbsolute,true);
    InitOK&=config.GetConfigByName("TrackMaper","ReshapeQRelative",params.ReshapeQRelative,true);
    InitOK&=config.GetConfigByName("TrackMaper","LocationUncertainty",params.LocationUncertainty,true);
    InitOK&=config.GetConfigByName("TrackMaper","LocationUncertaintyMatch",params.LocationUncertaintyMatch,true);
    InitOK&=config.GetConfigByName("TrackMaper","DoReScaling",params.DoReScaling,true);

    InitOK&=config.GetConfigByName("TrackMaper","GlobalMatchThreshold",params.MatchThreshold,true);

    InitOK&=config.GetConfigByName("REBVO","SaveLog",params.SaveLog,true);
    InitOK&=config.GetConfigByName("REBVO","LogFile",params.LogFile,true);
    InitOK&=config.GetConfigByName("REBVO","TrayFile",params.TrayFile,true);

    InitOK&=config.GetConfigByName("REBVO","VideoSave",params.VideoSave,true);
    InitOK&=config.GetConfigByName("REBVO","VideoSaveFile",params.VideoSaveFile,true);
    InitOK&=config.GetConfigByName("REBVO","VideoSaveBuffersize",params.VideoSaveBuffersize,true);
    InitOK&=config.GetConfigByName("REBVO","EdgeMapDelay",params.EdgeMapDelay,true);

    InitOK&=config.GetConfigByName("IMU","ImuMode",params.ImuMode,true);
    if(params.ImuMode>0){

        InitOK&=config.GetConfigByName("IMU","GiroMeasStdDev",params.GiroMeasStdDev,true);
        InitOK&=config.GetConfigByName("IMU","GiroBiasStdDev",params.GiroBiasStdDev,true);
        InitOK&=config.GetConfigByName("IMU","InitBias",params.InitBias,true);
        InitOK&=config.GetConfigByName("IMU","InitBiasFrameNum",params.InitBiasFrameNum,true);
        InitOK&=config.GetConfigByName("IMU","BiasHintX",params.BiasInitGuess[0],true);
        InitOK&=config.GetConfigByName("IMU","BiasHintY",params.BiasInitGuess[1],true);
        InitOK&=config.GetConfigByName("IMU","BiasHintZ",params.BiasInitGuess[2],true);
        InitOK&=config.GetConfigByName("IMU","g_module",params.g_module,true);


        InitOK&=config.GetConfigByName("IMU","AcelMeasStdDev",params.AcelMeasStdDev,true);
        InitOK&=config.GetConfigByName("IMU","g_module_uncer",params.g_module_uncer,true);
        InitOK&=config.GetConfigByName("IMU","g_uncert",params.g_uncert,true);
        InitOK&=config.GetConfigByName("IMU","VBiasStdDev",params.VBiasStdDev,true);
        InitOK&=config.GetConfigByName("IMU","ScaleStdDevMult",params.ScaleStdDevMult,true);
        InitOK&=config.GetConfigByName("IMU","ScaleStdDevMax",params.ScaleStdDevMax,true);
        InitOK&=config.GetConfigByName("IMU","ScaleStdDevInit",params.ScaleStdDevInit,true);
        InitOK&=config.GetConfigByName("IMU","CamImuSE3File",params.SE3File,true);
        InitOK&=config.GetConfigByName("IMU","TimeDesinc",params.TimeDesinc,true);

    }

    if(params.ImuMode==2){
        InitOK&=config.GetConfigByName("IMU","ImuFile",params.ImuFile,true);
        InitOK&=config.GetConfigByName("IMU","TimeScale",params.ImuTimeScale,true);
    }else if(params.ImuMode==1){

        InitOK&=config.GetConfigByName("IMU","SampleTime",params.SampleTime,true);
        InitOK&=config.GetConfigByName("IMU","CircBufferSize",params.CircBufferSize,true);
        params.ImuTimeScale=1;
    }

    cam=cam_model({params.pp_x,params.pp_y},{params.z_f_x,params.z_f_y},params.kc,params.ImageSize);


    construct();
}

REBVO::REBVO(const REBVOParameters &parameters)
    :params(parameters),pipe(CBUFSIZE,4),cam_pipe(CCAMBUFSIZE,2),cam({params.pp_x,params.pp_y},{params.z_f_x,params.z_f_y},params.kc,params.ImageSize),outputFunc(nullptr)
{
    construct();
}



void REBVO::construct(){

    if(params.CameraType==1 && params.simu_time_on && !GlobalTimer.TurnSimuOn(params.simu_time_step,params.simu_time_sweep,params.simu_time_start)){
        InitOK=false;
        return;
    }

    //***** Imu grabber contruct (if necesary) *****//

    switch(params.ImuMode){
    case 1:                         //Load imu using custom architecture, just init the grabber, program is responsible of push

        imu=new ImuGrabber(params.CircBufferSize,params.SampleTime);


        if(!imu->LoadCamImuSE3(params.SE3File.data())){
            cout << "REBVO: Failed to load cam-imu transformation \n" <<endl;
            InitOK=false;
            return;
        }
        break;

    case 2:                         //Load imu from dataset file
    {
        bool error=false;
        imu=new ImuGrabber(ImuGrabber::LoadDataSet(params.ImuFile.data(),false,params.ImuTimeScale,error));
        if(error){
            cout << "REBVO: Failed to initialize the imu grabber" <<endl;
            InitOK=false;
            return;
        }
    }
        if(!imu->LoadCamImuSE3(params.SE3File.data())){
            cout << "Failed to load cam-imu transformation \n" <<endl;
            InitOK=false;
            return;
        }

        break;

    }

    for(customCam::CustomCamPipeBuffer &pb:cam_pipe)
        pb.img=std::shared_ptr<Image<RGB24Pixel> >(new Image<RGB24Pixel>(params.ImageSize));


    //***** PipeLine init ******

    //Initializes every object in the pipeline buffer
    //for exchange between threads

    for(PipeBuffer &pbuf : pipe){

        pbuf.ss=new sspace(params.Sigma0,params.Sigma1,cam.sz,3);
        pbuf.ef=new edge_tracker(cam,255*3);
        pbuf.gt=new global_tracker (pbuf.ef->GetCam());
        pbuf.img=new Image<float>(cam.sz);
        pbuf.imgc=new Image<RGB24Pixel>(cam.sz);
        pbuf.t=0;
    }
    return;
}


bool REBVO::Init(){

    if(!InitOK)
        return false;

    saveImg=false;
    start_record=false;
    system_reset=false;
    quit=false;
    Thr0=std::thread (FirstThr,this);

    return true;
}





bool REBVO::CleanUp(){

    quit=true;
    Thr0.join();
    return true;
}


REBVO::~REBVO(){

    if(!quit)
        CleanUp();
    if(imu)
        delete imu;

    for(PipeBuffer &pbuf : pipe){

        delete pbuf.ss;
        delete pbuf.ef;
        delete pbuf.gt;
        delete pbuf.img;
        delete pbuf.imgc;
    }
}

}
