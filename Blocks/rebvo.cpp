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


#include "rebvo.h"

#include <TooN/so3.h>

#include <iostream>
#include <iomanip>
#include "ttimer.h"
#include "datasetcam.h"
#include <TooN/Cholesky.h>
#include <scaleestimator.h>





REBVO::REBVO(Configurator &config)
    :pipe(CBUFSIZE,4)
{
    Size2D ImageSize;
    float z_f_x;
    float z_f_y;
    float pp_y;
    float pp_x;
    cam_model::rad_tan_distortion kc;

    int simu_time_on,simu_time_step;
    double simu_time_sweep,simu_time_start;


    InitOK&=config.GetConfigByName("Camera","CameraDevice",CameraDevice,true);
    InitOK&=config.GetConfigByName("REBVO","CameraType",CameraType,true);


    InitOK&=config.GetConfigByName("SimuCamera","SimVideoFile",SimFile,true);
    InitOK&=config.GetConfigByName("SimuCamera","SimVideoNFrames",sim_save_nframes,true);

    InitOK&=config.GetConfigByName("SimuCamera","SimuTimeOn",simu_time_on,true);
    InitOK&=config.GetConfigByName("SimuCamera","SimuTimeSweep",simu_time_sweep,true);
    InitOK&=config.GetConfigByName("SimuCamera","SimuTimeStep",simu_time_step,true);
    InitOK&=config.GetConfigByName("SimuCamera","SimuTimeStart",simu_time_start,true);


    InitOK&=config.GetConfigByName("DataSetCamera","DataSetDir",DataSetDir,true);
    InitOK&=config.GetConfigByName("DataSetCamera","DataSetFile",DataSetFile,true);
    InitOK&=config.GetConfigByName("DataSetCamera","TimeScale",CamTimeScale,true);


    InitOK&=config.GetConfigByName("REBVO","VideoNetHost",VideoNetHost,true);
    InitOK&=config.GetConfigByName("REBVO","VideoNetPort",VideoNetPort,true);
    InitOK&=config.GetConfigByName("REBVO","VideoNetEnabled",VideoNetEnabled,true);
    InitOK&=config.GetConfigByName("REBVO","BlockingUDP",BlockingUDP,true);

    InitOK&=config.GetConfigByName("Camera","ImageWidth",ImageSize.w,true);
    InitOK&=config.GetConfigByName("Camera","ImageHeight",ImageSize.h,true);

    InitOK&=config.GetConfigByName("Camera","ZfX",z_f_x,true);
    InitOK&=config.GetConfigByName("Camera","ZfY",z_f_y,true);

    InitOK&=config.GetConfigByName("Camera","PPx",pp_x,true);
    InitOK&=config.GetConfigByName("Camera","PPy",pp_y,true);

    InitOK&=config.GetConfigByName("Camera","KcR2",kc.Kc2,true);
    InitOK&=config.GetConfigByName("Camera","KcR4",kc.Kc4,true);
    InitOK&=config.GetConfigByName("Camera","KcR6",kc.Kc6,true);
    InitOK&=config.GetConfigByName("Camera","KcP1",kc.P1,true);
    InitOK&=config.GetConfigByName("Camera","KcP2",kc.P2,true);
    InitOK&=config.GetConfigByName("Camera","UseUndistort",useUndistort,true);
    InitOK&=config.GetConfigByName("Camera","Rotate180",rotatedCam,true);

    InitOK&=config.GetConfigByName("Camera","FPS",config_fps,true);
    InitOK&=config.GetConfigByName("ProcesorConfig","SetAffinity",cpuSetAffinity,true);
    InitOK&=config.GetConfigByName("ProcesorConfig","CamaraT1",cpu0,true);
    InitOK&=config.GetConfigByName("ProcesorConfig","CamaraT2",cpu1,true);
    InitOK&=config.GetConfigByName("ProcesorConfig","CamaraT3",cpu2,true);
    InitOK&=config.GetConfigByName("REBVO","EncoderType",encoder_type,true);
    InitOK&=config.GetConfigByName("REBVO","EncoderDevice",encoder_dev,true);

    InitOK&=config.GetConfigByName("Detector","Sigma0",Sigma0,true);
    InitOK&=config.GetConfigByName("Detector","Sigma1",Sigma1,true);
    InitOK&=config.GetConfigByName("Detector","ReferencePoints",ReferencePoints,true);
    InitOK&=config.GetConfigByName("Detector","MaxPoints",MaxPoints,true);
    InitOK&=config.GetConfigByName("Detector","DetectorThresh",DetectorThresh,true);
    InitOK&=config.GetConfigByName("Detector","DetectorAutoGain",DetectorAutoGain,true);

    InitOK&=config.GetConfigByName("Detector","DetectorMaxThresh",DetectorMaxThresh,true);
    InitOK&=config.GetConfigByName("Detector","DetectorMinThresh",DetectorMinThresh,true);

    InitOK&=config.GetConfigByName("Detector","DetectorPlaneFitSize",DetectorPlaneFitSize,true);
    InitOK&=config.GetConfigByName("Detector","DetectorPosNegThresh",DetectorPosNegThresh,true);
    InitOK&=config.GetConfigByName("Detector","DetectorDoGThresh",DetectorDoGThresh,true);

    InitOK&=config.GetConfigByName("TrackMaper","SearchRange",SearchRange,true);
    InitOK&=config.GetConfigByName("TrackMaper","QCutOffNumBins",QCutOffNumBins,true);
    InitOK&=config.GetConfigByName("TrackMaper","QCutOffQuantile",QCutOffQuantile,true);
    InitOK&=config.GetConfigByName("TrackMaper","TrackerIterNum",TrackerIterNum,true);
    InitOK&=config.GetConfigByName("TrackMaper","TrackerInitIterNum",TrackerInitIterNum,true);
    InitOK&=config.GetConfigByName("TrackMaper","TrackerInitType",TrackerInitType,true);
    InitOK&=config.GetConfigByName("TrackMaper","TrackerMatchThresh",TrackerMatchThresh,true);
    InitOK&=config.GetConfigByName("TrackMaper","MatchThreshModule",MatchThreshModule,true);
    InitOK&=config.GetConfigByName("TrackMaper","MatchThreshAngle",MatchThreshAngle,true);
    InitOK&=config.GetConfigByName("TrackMaper","MatchNumThresh",MatchNumThresh,true);
    InitOK&=config.GetConfigByName("TrackMaper","RegularizeThresh",RegularizeThresh,true);
    InitOK&=config.GetConfigByName("TrackMaper","ReweigthDistance",ReweigthDistance,true);

    InitOK&=config.GetConfigByName("TrackMaper","ReshapeQAbsolute",ReshapeQAbsolute,true);
    InitOK&=config.GetConfigByName("TrackMaper","ReshapeQRelative",ReshapeQRelative,true);
    InitOK&=config.GetConfigByName("TrackMaper","LocationUncertainty",LocationUncertainty,true);
    InitOK&=config.GetConfigByName("TrackMaper","LocationUncertaintyMatch",LocationUncertaintyMatch,true);
    InitOK&=config.GetConfigByName("TrackMaper","DoReScaling",DoReScaling,true);

    InitOK&=config.GetConfigByName("TrackMaper","GlobalMatchThreshold",MatchThreshold,true);

    InitOK&=config.GetConfigByName("REBVO","SaveLog",SaveLog,true);
    InitOK&=config.GetConfigByName("REBVO","LogFile",LogFile,true);
    InitOK&=config.GetConfigByName("REBVO","TrayFile",TrayFile,true);

    InitOK&=config.GetConfigByName("REBVO","VideoSave",VideoSave,true);
    InitOK&=config.GetConfigByName("REBVO","VideoSaveFile",VideoSaveFile,true);
    InitOK&=config.GetConfigByName("REBVO","VideoSaveBuffersize",VideoSaveBuffersize,true);
    InitOK&=config.GetConfigByName("REBVO","EdgeMapDelay",EdgeMapDelay,true);

    InitOK&=config.GetConfigByName("IMU","ImuMode",ImuMode,true);
    if(ImuMode>0){

        InitOK&=config.GetConfigByName("IMU","GiroMeasStdDev",GiroMeasStdDev,true);
        InitOK&=config.GetConfigByName("IMU","GiroBiasStdDev",GiroBiasStdDev,true);
        InitOK&=config.GetConfigByName("IMU","InitBias",InitBias,true);
        InitOK&=config.GetConfigByName("IMU","InitBiasFrameNum",InitBiasFrameNum,true);
        InitOK&=config.GetConfigByName("IMU","BiasHintX",BiasInitGuess[0],true);
        InitOK&=config.GetConfigByName("IMU","BiasHintY",BiasInitGuess[1],true);
        InitOK&=config.GetConfigByName("IMU","BiasHintZ",BiasInitGuess[2],true);
        InitOK&=config.GetConfigByName("IMU","g_module",g_module,true);


        InitOK&=config.GetConfigByName("IMU","AcelMeasStdDev",AcelMeasStdDev,true);
        InitOK&=config.GetConfigByName("IMU","g_module_uncer",g_module_uncer,true);
        InitOK&=config.GetConfigByName("IMU","g_uncert",g_uncert,true);
        InitOK&=config.GetConfigByName("IMU","VBiasStdDev",VBiasStdDev,true);
        InitOK&=config.GetConfigByName("IMU","ScaleStdDevMult",ScaleStdDevMult,true);
        InitOK&=config.GetConfigByName("IMU","ScaleStdDevMax",ScaleStdDevMax,true);
        InitOK&=config.GetConfigByName("IMU","ScaleStdDevInit",ScaleStdDevInit,true);
        InitOK&=config.GetConfigByName("IMU","CamImuSE3File",SE3File,true);
        InitOK&=config.GetConfigByName("IMU","TimeDesinc",TimeDesinc,true);

    }

    if(ImuMode==2){
        InitOK&=config.GetConfigByName("IMU","ImuFile",ImuFile,true);
        InitOK&=config.GetConfigByName("IMU","TimeScale",ImuTimeScale,true);
    }else if(ImuMode==1){

        InitOK&=config.GetConfigByName("IMU","SampleTime",SampleTime,true);
        InitOK&=config.GetConfigByName("IMU","CircBufferSize",CircBufferSize,true);
        InitOK&=config.GetConfigByName("IMU","DeviceName",ImuDevName,true);
        ImuTimeScale=1;
    }




    cam=new cam_model({pp_x,pp_y},{z_f_x,z_f_y},kc,ImageSize);

    if(CameraType==1 && simu_time_on && !GlobalTimer.TurnSimuOn(simu_time_step,simu_time_sweep,simu_time_start)){
            InitOK=false;
    }

}

REBVO::~REBVO(){
    delete cam;
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

