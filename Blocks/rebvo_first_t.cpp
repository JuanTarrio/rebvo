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





using namespace std;
//FirstThr() deals with the capture and detection of the edgemap and optionally with
//Auxiliary image building.

void REBVO::FirstThr(REBVO *cf){


    double t=0,t0=0,dtp;
    COND_TIME_DEBUG(double dt;)


    int l_kl_num=0;

    double tresh=cf->DetectorThresh;

    //Simple camera model

    cam_model cam(*cf->cam);    //Start a fresh copy for tread safe use

    RGB24Pixel *data;



    //***** PipeLine init ******

    //Initializes every object in the pipeline buffer
    //for exchange between threads

    for(PipeBuffer &pbuf : cf->pipe){

        pbuf.ss=new sspace(cf->Sigma0,cf->Sigma1,cam.sz,3);
        pbuf.ef=new edge_tracker(cam,255*3);
        pbuf.gt=new global_tracker (pbuf.ef->GetCam());
        pbuf.img=new Image<float>(cam.sz);
        pbuf.imgc=new Image<RGB24Pixel>(cam.sz);
        pbuf.t=0;
    }


    //***** Cam init *****

    VideoCam *camara;

    switch(cf->CameraType){
    case 2:
        camara=new DataSetCam(cf->DataSetDir.data(),cf->DataSetFile.data(),cam.sz,cf->CamTimeScale,cf->SimFile.data());
        break;
    case 1:
        camara=new simcam(cf->SimFile.data(),cam.sz);
        break;
    case 0:
    default:
        camara=new v4lCam(cf->CameraDevice.data(),cam.sz,cf->config_fps,cf->SimFile.data());
        break;
    }

    if(camara->Error()){
        cout << "Failed to initialize the camera " <<endl;
        cf->quit=true;
        return;
    }

    //***** Imu init *****//

    switch(cf->ImuMode){
    case 1:                         //Load imu using specific architecture



        cf->imu=new ImuGrabber(cf->CircBufferSize,cf->SampleTime);
        cf->imu_dev=new archIMU(cf->ImuDevName.data(),*cf->imu);

        if(cf->imu_dev->error()){
            cout << "Failed to initialize the imu device" <<endl;
            cf->quit=true;
            return;
        }

        if(!cf->imu->LoadCamImuSE3(cf->SE3File.data())){
            cout << "Failed to load cam-imu transformation \n" <<endl;
            cf->quit=true;
            return;
        }
        break;

    case 2:                         //Load imu from dataset file
    {
        bool error=false;
        cf->imu=new ImuGrabber(ImuGrabber::LoadDataSet(cf->ImuFile.data(),false,cf->ImuTimeScale,error));
        if(error){
            cout << "Failed to initialize the imu " <<endl;
            cf->quit=true;
            return;
        }

        if(!cf->imu->LoadCamImuSE3(cf->SE3File.data())){
            cout << "Failed to load cam-imu transformation \n" <<endl;
            cf->quit=true;
            return;
        }
    }
        break;

    }


    //***** Call thread 1 & set cpu afiinity ******

    std::thread Thr1(SecondThread,cf);

    if(cf->cpuSetAffinity){
        cpu_set_t cpusetp;

        CPU_ZERO(&cpusetp);
        CPU_SET(cf->cpu0,&cpusetp);
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
            pbuf.imu=cf->imu->GrabAndIntegrate(t0+cf->TimeDesinc,t+cf->TimeDesinc);
        }



        COND_TIME_DEBUG(dt=t-t0;)

        t0=t;

        int p_num=camara->PakNum();

        //Push frame for record (if necesary)
        camara->PushFrame(data);


        tproc.start();

        //(*pbuf.imgc)=data;              //copy buffer for fast release

        for(int i=0,j=pbuf.imgc->bSize()-1;i<pbuf.imgc->bSize();i++,j--)
            (*pbuf.imgc)[i]=data[j];

        camara->ReleaseBuffer();


        //Simple color conversion
        Image<float>::ConvertRGB2BW(*pbuf.img,*pbuf.imgc);

        //Build the scales and the DoG for edge detection
        pbuf.ss->build(*pbuf.img);

        //Detect Edges
        pbuf.ef->detect(pbuf.ss,cf->DetectorPlaneFitSize,cf->DetectorPosNegThresh,cf->DetectorDoGThresh
                        ,cf->MaxPoints,tresh,l_kl_num,cf->ReferencePoints,cf->DetectorAutoGain,cf->DetectorMaxThresh,cf->DetectorMinThresh);

        //Build auxiliary image on the GT
        pbuf.gt->build_field(*pbuf.ef,cf->SearchRange);
        dtp=tproc.stop();


        COND_TIME_DEBUG(printf("\nCamaraFrontal: dtp0 = %f, kln=%d thresh=%f dt=%f\n",dtp,l_kl_num,tresh,dt);)

        //Save time, processing time and frame Id
        pbuf.t=t;
        pbuf.dtp0=dtp;
        pbuf.p_id=p_num-1;
        pbuf.quit=false;


        if(cf->start_record){           //Start the recording of uncompressed video
            cf->start_record=false;
            camara->RecordNFrames(cf->sim_save_nframes);
        }
        cf->pipe.ReleaseBuffer(0);
    }

    if(cf->imu_dev){
        cf->imu_dev->killIMU();
    }

    Thr1.join();

    delete camara;
    return;
}


