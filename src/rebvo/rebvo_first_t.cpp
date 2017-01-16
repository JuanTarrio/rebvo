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
#include "UtilLib/ttimer.h"
#include "VideoLib/datasetcam.h"
#include "mtracklib/scaleestimator.h"
#include "VideoLib/image_undistort.h"

#include "VideoLib/v4lcam.h"
#include "VideoLib/simcam.h"

using namespace std;

namespace rebvo {
//FirstThr() deals with the capture and detection of the edgemap and optionally with
//Auxiliary image building.

VideoCam * REBVO::initCamera(){
    switch (params.CameraType) {
    case 3:
        return new customCam(cam_pipe, cam.sz, params.SimFile.data());
        break;
    case 2:
        return new DataSetCam(params.DataSetDir.data(),
                params.DataSetFile.data(), cam.sz, params.CamTimeScale,
                params.SimFile.data());
        break;
    case 1:
        return new simcam(params.SimFile.data(), cam.sz);
        break;
    case 0:
    default:
        return new v4lCam(params.CameraDevice.data(), cam.sz,
                params.config_fps, params.SimFile.data());
        break;
    }
}

VideoCam * REBVO::initPairCamera(){
    switch (params.CameraType) {
    case 3:
        return new customCam(cam_pipe_stereo, cam.sz, params.SimFile.data());
        break;
    case 2:
        return new DataSetCam(params.DataSetDirStereo.data(),
                params.DataSetFileStereo.data(), cam.sz, params.CamTimeScale,
                params.SimFile.data());
        break;
    case 1:
        return nullptr;
        break;
    case 0:
    default:
        return nullptr;
        break;
    }
}


void REBVO::FirstThr(REBVO *cf) {

	double t = 0, t0 = 0, dtp;
	COND_TIME_DEBUG(double dt;)

	int l_kl_num = 0;

	double tresh = cf->params.DetectorThresh;

	//Simple camera model

	cam_model cam(cf->cam);    //Start a fresh copy for tread safe use

    RGB24Pixel *data=nullptr,*data_pair=nullptr;

	image_undistort undistorter(cam);
	Image<RGB24Pixel> img_dist(cam.sz);
    Image<RGB24Pixel> img_dist_pair(cam.sz);

	//***** Cam init *****

    VideoCam *camara=cf->initCamera();

	if (camara->Error()) {
        cout << "REBVO: Failed to initialize the main camera " << endl;
		cf->quit = true;
		return;
	}

    VideoCam *camara_pair=cf->initPairCamera();

    if (!camara_pair || camara->Error()) {
        cout << "REBVO: Failed to initialize the stereo camera " << endl;
        cf->quit = true;
        return;
    }
	//***** Call thread 1 & set cpu afiinity ******

	std::thread Thr1(SecondThread, cf);

	if (cf->params.cpuSetAffinity) {
        if(!REBVO::setAffinity(cf->params.cpu0)){
            std::cout <<"REBVO: Cannot set cpu affinity on the first thread";
            cf->quit=true;
        }
	}

	//******* Main loop ******

	util::timer tproc;

	while (!cf->quit) {

		//Request free buffer on the pipeline
		PipeBuffer &pbuf = cf->pipe.RequestBuffer(0);

		//Grab frame
		while ((data = camara->GrabBuffer(t, false)) == nullptr) {
            if (cf->quit || camara->Error()) {
				pbuf.quit = true; //If no more frames, release the pipeline buffer
				cf->pipe.ReleaseBuffer(0);               //with the quit flag on
				std::cout << "bye bye cruel world" << std::endl;
				goto exit_while;
			}
		}


        //********* Grab and sync stereo frame *****************//

        if(cf->params.StereoAvaiable){

            double t_stereo;
            while ((data_pair = camara_pair->GrabBuffer(t_stereo, false)) == nullptr) {
                if (cf->quit || camara_pair->Error()) {
                    pbuf.quit = true; //If no more frames, release the pipeline buffer
                    cf->pipe.ReleaseBuffer(0);               //with the quit flag on
                    std::cout << "bye bye cruel world on stereo" << std::endl;
                    goto exit_while;
                }
            }

            if(fabs(t-t_stereo)>(0.5/cf->params.config_fps))
                std::cout<<"REBVO Warning: cameras are unsync: "<<t-t_stereo<<"\n";

        }

        //********* Grab IMU datas *****************//

		if (cf->imu) {
            while(!cf->quit){
                pbuf.imu = cf->imu->GrabAndIntegrate(t0 + cf->params.TimeDesinc,
                                                     t + cf->params.TimeDesinc);
                if(pbuf.imu.n>0)
                    break;

                std::this_thread::sleep_for(std::chrono::duration<double>(cf->params.SampleTime));  //Sleep for IMU sampl time to wait for more data
            }
		}

		COND_TIME_DEBUG(dt=t-t0;)

		t0 = t;

		int p_num = camara->PakNum();

		//Push frame for record (if necesary)
		camara->PushFrame(data);

		tproc.start();

		if (cf->params.useUndistort) {

            if (cf->params.rotatedCam){
				img_dist.copyFromRotate180(data); //copy buffer for fast release and rotate 180 deg

                if(cf->params.StereoAvaiable)
                    img_dist_pair.copyFromRotate180(data_pair);
            }else{
				img_dist = data;              //copy buffer for fast release

                if(cf->params.StereoAvaiable)
                    img_dist_pair=data_pair;
            }

			camara->ReleaseBuffer();


            if(cf->params.StereoAvaiable)
                camara_pair->ReleaseBuffer();

			undistorter.undistort<true>((*pbuf.imgc), img_dist); //Use radial-tangencial with bilin interp for undistortion

            if(cf->params.StereoAvaiable)
                undistorter.undistort<true>((*pbuf.imgc_pair), img_dist_pair); //Use radial-tangencial with bilin interp for undistortion

		} else {

            if (cf->params.rotatedCam){
				(*pbuf.imgc).copyFromRotate180(data); //copy buffer for fast release and rotate 180 deg


                if(cf->params.StereoAvaiable)
                    (*pbuf.imgc_pair).copyFromRotate180(data_pair);

            }else{
				(*pbuf.imgc) = data;              //copy buffer for fast release

                if(cf->params.StereoAvaiable)
                    (*pbuf.imgc_pair)=data_pair;
            }

			camara->ReleaseBuffer();
            camara_pair->ReleaseBuffer();
		}

		//Simple color conversion
		Image<float>::ConvertRGB2BW((*pbuf.img), *pbuf.imgc);


		//Build the scales and the DoG for edge detection
		pbuf.ss->build(*pbuf.img);

		//Detect Edges
		pbuf.ef->detect(pbuf.ss, cf->params.DetectorPlaneFitSize,
				cf->params.DetectorPosNegThresh, cf->params.DetectorDoGThresh,
				cf->params.MaxPoints, tresh, l_kl_num,
				cf->params.ReferencePoints, cf->params.DetectorAutoGain,
				cf->params.DetectorMaxThresh, cf->params.DetectorMinThresh);


        if(cf->params.StereoAvaiable){
            Image<float>::ConvertRGB2BW((*pbuf.img_pair), *pbuf.imgc_pair);


            //Build the scales and the DoG for edge detection on pair img
            pbuf.ss_pair->build(*pbuf.img_pair);

            //Detect Edges on pair
            pbuf.ef_pair->detect(pbuf.ss_pair, cf->params.DetectorPlaneFitSize,
                    cf->params.DetectorPosNegThresh, cf->params.DetectorDoGThresh,
                    cf->params.MaxPoints, tresh, l_kl_num,
                    cf->params.ReferencePoints, cf->params.DetectorAutoGain,
                    cf->params.DetectorMaxThresh, cf->params.DetectorMinThresh);
        }


		//Build auxiliary image on the GT
		pbuf.gt->build_field(*pbuf.ef, cf->params.SearchRange);


		dtp = tproc.stop();

		COND_TIME_DEBUG(printf("\nCamaraFrontal: dtp0 = %f, kln=%d thresh=%f dt=%f\n",dtp,l_kl_num,tresh,dt);)

		//Save time, processing time and frame Id
		pbuf.t = t;
		pbuf.dtp0 = dtp;
		pbuf.p_id = p_num - 1;
		pbuf.quit = false;

		if (cf->start_record) {      //Start the recording of uncompressed video
			cf->start_record = false;
			camara->RecordNFrames(cf->params.sim_save_nframes);
		}
		cf->pipe.ReleaseBuffer(0);
	} exit_while :

	Thr1.join();

	delete camara;


	return;
}

}
