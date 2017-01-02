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
#include <image_undistort.h>


#include "v4lcam.h"
#include "simcam.h"



using namespace std;
//FirstThr() deals with the capture and detection of the edgemap and optionally with
//Auxiliary image building.

void REBVO::FirstThr(REBVO *cf){


    double t=0,t0=0,dtp;
    COND_TIME_DEBUG(double dt;)


    int l_kl_num=0;

    double tresh=cf->params.DetectorThresh;

    //Simple camera model

    cam_model cam(cf->cam);    //Start a fresh copy for tread safe use

    RGB24Pixel *data;

    image_undistort undistorter(cam);
    Image<RGB24Pixel> img_dist(cam.sz);

    //***** PipeLine init ******

    //Initializes every object in the pipeline buffer
    //for exchange between threads

    for(PipeBuffer &pbuf : cf->pipe){

        pbuf.ss=new sspace(cf->params.Sigma0,cf->params.Sigma1,cam.sz,3);
        pbuf.ef=new edge_tracker(cam,255*3);
        pbuf.gt=new global_tracker (pbuf.ef->GetCam());
        pbuf.img=new Image<float>(cam.sz);
        pbuf.imgc=new Image<RGB24Pixel>(cam.sz);
        pbuf.t=0;
    }


    //***** Cam init *****

    VideoCam *camara;

    switch(cf->params.CameraType){
    case 3:
        camara=new customCam(cf->cam_pipe,cam.sz,cf->params.SimFile.data());
        break;
    case 2:
        camara=new DataSetCam(cf->params.DataSetDir.data(),cf->params.DataSetFile.data(),cam.sz,cf->params.CamTimeScale,cf->params.SimFile.data());
        break;
    case 1:
        camara=new simcam(cf->params.SimFile.data(),cam.sz);
        break;
    case 0:
    default:
        camara=new v4lCam(cf->params.CameraDevice.data(),cam.sz,cf->params.config_fps,cf->params.SimFile.data());
        break;
    }

    if(camara->Error()){
        cout << "Failed to initialize the camera " <<endl;
        cf->quit=true;
        return;
    }




    //***** Call thread 1 & set cpu afiinity ******

    std::thread Thr1(SecondThread,cf);

    if(cf->params.cpuSetAffinity){
        cpu_set_t cpusetp;

        CPU_ZERO(&cpusetp);
        CPU_SET(cf->params.cpu0,&cpusetp);
        if(pthread_setaffinity_np(pthread_self(),sizeof(cpu_set_t),&cpusetp)!=0){
            printf("\nThread: Can't' set CPU affinity!\n");
            cf->quit=true;
        }
    }


    //******* Main loop ******

    if(camara->WaitFrame(false)<0){
        cout <<"Error en wait frame\n";
        return;
    }
    data=camara->GrabBuffer(t0,false);
    camara->ReleaseBuffer();

    util::timer tproc;

    while (!cf->quit){

        //Request free buffer on the pipeline
        PipeBuffer &pbuf=cf->pipe.RequestBuffer(0);

        //Grab frame
        if((data=camara->GrabBuffer(t,false))==NULL){
            pbuf.quit=true;                             //If no more frames, release the pipeline buffer
            cf->pipe.ReleaseBuffer(0);                  //with the quit flag on
            break;
        }


        if(cf->imu){
            pbuf.imu=cf->imu->GrabAndIntegrate(t0+cf->params.TimeDesinc,t+cf->params.TimeDesinc);
        }



        COND_TIME_DEBUG(dt=t-t0;)

        t0=t;

        int p_num=camara->PakNum();

        //Push frame for record (if necesary)
        camara->PushFrame(data);


        tproc.start();

        if(cf->params.useUndistort){

            if(cf->params.rotatedCam)
                img_dist.copyFromRotate180(data);   //copy buffer for fast release and rotate 180 deg
            else
                img_dist=data;              //copy buffer for fast release


            camara->ReleaseBuffer();

            undistorter.undistort<true>((*pbuf.imgc),img_dist); //Use radial-tangencial with bilin interp for undistortion

        }else{

            if(cf->params.rotatedCam)
                (*pbuf.imgc).copyFromRotate180(data);   //copy buffer for fast release and rotate 180 deg
            else
                (*pbuf.imgc)=data;              //copy buffer for fast release

            camara->ReleaseBuffer();
        }






        //Simple color conversion
        Image<float>::ConvertRGB2BW((*pbuf.img),*pbuf.imgc);


        //Build the scales and the DoG for edge detection
        pbuf.ss->build(*pbuf.img);

        //Detect Edges
        pbuf.ef->detect(pbuf.ss,cf->params.DetectorPlaneFitSize,cf->params.DetectorPosNegThresh,cf->params.DetectorDoGThresh
                        ,cf->params.MaxPoints,tresh,l_kl_num,cf->params.ReferencePoints,cf->params.DetectorAutoGain,cf->params.DetectorMaxThresh,cf->params.DetectorMinThresh);

        //Build auxiliary image on the GT
        pbuf.gt->build_field(*pbuf.ef,cf->params.SearchRange);


        dtp=tproc.stop();


        COND_TIME_DEBUG(printf("\nCamaraFrontal: dtp0 = %f, kln=%d thresh=%f dt=%f\n",dtp,l_kl_num,tresh,dt);)

        //Save time, processing time and frame Id
        pbuf.t=t;
        pbuf.dtp0=dtp;
        pbuf.p_id=p_num-1;
        pbuf.quit=false;


        if(cf->start_record){           //Start the recording of uncompressed video
            cf->start_record=false;
            camara->RecordNFrames(cf->params.sim_save_nframes);
        }
        cf->pipe.ReleaseBuffer(0);
    }



    Thr1.join();

    delete camara;
    return;
}


