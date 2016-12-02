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


//#define TIME_DEBUG

#ifdef TIME_DEBUG
#define COND_TIME_DEBUG(arg) arg
#else
#define COND_TIME_DEBUG(arg)
#endif


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
        cout << "Failed to initialize the camera " << cf->CameraDevice.data() <<endl;
        cf->quit=true;
        return;
    }

    //***** Imu init *****//

    switch(cf->ImuMode){
    case 2:
    {
        bool error=false;
        cf->imu=new ImuGrabber(ImuGrabber::LoadDataSet(cf->ImuFile.data(),false,cf->ImuTimeScale,error));
        if(error){
            cout << "Failed to initialize the imu " <<endl;
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
            pbuf.imu=cf->imu->GrabAndIntegrate(t0,t);
        }

        COND_TIME_DEBUG(dt=t-t0;)

        t0=t;

        int p_num=camara->PakNum();

        //Push frame for record (if necesary)
        camara->PushFrame(data);


        tproc.start();

        (*pbuf.imgc)=data;              //copy buffer for fast release

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


    Thr1.join();

    delete camara;
    return;
}



void  REBVO::SecondThread(REBVO *cf){

    using namespace TooN;

    double dt_frame;

    COND_TIME_DEBUG(double dtp;)


    int n_frame=0;

    double Kp=1,K=1;

    double error_vel=0;                 //Estimated error for the velocity
    double error_score=0;               //Residual energy from the tracking

    Vector <3> V=Zeros,W=Zeros,Pos=Zeros;   //Estimated Velocity, Rotation and Position
    Matrix<3,3> R=Identity,Pose=Identity;   //Rotation Matrix and Global Rotation


    double P_Kp=5e-6;
    Matrix <3,3> P_V=Identity*1e50,P_W=Identity*1e-10;

    IMUState istate;

    istate.RGBias=Identity*cf->GiroBiasStdDev*cf->GiroBiasStdDev;
    istate.RGiro=Identity*cf->GiroMeasStdDev*cf->GiroMeasStdDev;
    istate.W_Bg=util::Matrix3x3Inv(istate.RGBias*100);

    istate.Rg=cf->g_module_uncer*cf->g_module_uncer;
    istate.Rs=Identity*cf->AcelMeasStdDev*cf->AcelMeasStdDev;
    istate.Qbias=Identity*cf->VBiasStdDev*cf->VBiasStdDev;

    istate.X=makeVector(M_PI/4,0,cf->g_module,0,0,0,0);
    istate.P=makeVector(M_PI/16*M_PI/16*1e-2,\
                        100,100,100,\
                        cf->VBiasStdDev*cf->VBiasStdDev*1e4,cf->VBiasStdDev*cf->VBiasStdDev*1e4,cf->VBiasStdDev*cf->VBiasStdDev*1e4).as_diagonal();

    int n_giro_init=0;
    Vector <3> giro_init=Zeros;

    //**** Set cpu Afinity for this thread *****

    if(cf->cpuSetAffinity){
        cpu_set_t cpusetp;

        CPU_ZERO(&cpusetp);
        CPU_SET(cf->cpu1,&cpusetp);
        if(pthread_setaffinity_np(pthread_self(),sizeof(cpu_set_t),&cpusetp)!=0){
            printf("\nSecond Thread: No puedo setear CPU affinity!\n");
            cf->quit=true;
            return;
        }
    }


    //***** Launch Thread 2 Pipeline thread ******

    std::thread Thr2(ThirdThread,cf);

    COND_TIME_DEBUG(util::interval_list tlist;)

    COND_TIME_DEBUG(util::timer t_loop;)

    //****** Dummy processing of the first frame ******


    cf->pipe.RequestBuffer(1);
    cf->pipe.ReleaseBuffer(1);

    //****** Main loop *******

    while(!cf->quit){

        bool EstimationOk=true;

        COND_TIME_DEBUG(dtp=t_loop.stop();)

        // 2 pipeline buffers are used in this thread
        PipeBuffer &new_buf=cf->pipe.RequestBuffer(1);  //This represent the newest edge-map
        PipeBuffer &old_buf=cf->pipe.RequestBuffer(2);  //This represents the old edgemap

        if(new_buf.quit){                               //If quit flag on the new buffer,
            old_buf.quit=true;                          //release quit on the old for thread 2
            cf->pipe.ReleaseBuffer(1);
            cf->pipe.ReleaseBuffer(2);
            break;
        }


        dt_frame=(new_buf.t-old_buf.t);

        if(dt_frame<0.001)
            dt_frame=1/cf->config_fps;

        COND_TIME_DEBUG(t_loop.start();)
        COND_TIME_DEBUG(tlist.clear();)

        int klm_num=0;

        COND_TIME_DEBUG(tlist.push_new();)
        //Estimate uncertainity cut-off for some quantile (90 percent typicaly)
        double s_rho_q=old_buf.ef->EstimateQuantile(RHO_MIN,RHO_MAX,cf->QCutOffQuantile,cf->QCutOffNumBins);


        COND_TIME_DEBUG(tlist.push_new();)

        //new_buf.gt->build_field(*new_buf.ef,cf->SearchRange);

        COND_TIME_DEBUG(tlist.push_new();)

        P_V=Identity*1e50;
        P_W=Identity*1e50;

        R=Identity;

        //***** Use the Tracker to obtain Velocity and Rotation  ****

        if(cf->ImuMode>0){      //Imu data avaiable?? Use it!

            if(!istate.init && cf->InitBias && n_frame>0){

                giro_init+=new_buf.imu.giro*new_buf.imu.dt;
                //giro_init+=istate.dWv;
                if(++n_giro_init>10){
                    istate.Bg=giro_init/n_giro_init;
                    istate.init=true;
                    istate.W_Bg=util::Matrix3x3Inv(istate.RGBias*1e2);
                }
            }


            R=new_buf.imu.Rot;  //Use imu rotation


            R.T()=TooN::SO3<>(istate.Bg)*R.T();

            old_buf.ef->rotate_keylines(R.T()); //Apply foward prerotation to the old key lines


            new_buf.gt->Minimizer_V<double>(istate.Vg,istate.P_Vg,*old_buf.ef,cf->TrackerMatchThresh,cf->TrackerIterNum,s_rho_q,cf->MatchNumThresh);   //Estimate translation only


            //***** Match from the old EdgeMap to the new one using the information from the minimization *****
            old_buf.ef->FordwardMatch(new_buf.ef);

            //***** Visual RotoTranslation estimation using forward matches
            Matrix <6,6> R_Xv,R_Xgv,W_Xv,W_Xgv;
            Vector <6> Xv,Xgv,Xgva;
            EstimationOk&=new_buf.ef->ExtRotVel(istate.Vg,W_Xv,R_Xv,Xv,cf->LocationUncertainty);


            istate.dVv=Xv.slice<0,3>();
            istate.dWv=Xv.slice<3,3>();

            Xgv=Xv;
            W_Xgv=W_Xv;

           // std::cout<<W_Xv<<"\n";

            Vector <3> dgbias=Zeros;
            edge_tracker::BiasCorrect(Xgv,W_Xgv,dgbias,istate.W_Bg,istate.RGiro,istate.RGBias);
            istate.Bg+=dgbias;

           // std::cout<<W_Xv<<W_Xgv<<istate.W_Bg;

            istate.dVgv=Xgv.slice<0,3>();
            istate.dWgv=Xgv.slice<3,3>();

            //***** extract rotation matrix *****
            SO3 <>R0(istate.dWgv);                      //R0 is a forward rotation
            R.T()=R0.get_matrix()*R.T();                //R is a backward rotation

            istate.Vgv=R0*istate.Vg+istate.dVgv;
            V=istate.Vgv;

            istate.Wgv=SO3<>(R).ln();

            R_Xgv=Cholesky<6>(W_Xgv).get_inverse();


            P_V=R_Xgv.slice<0,0,3,3>();
            P_W=R_Xgv.slice<3,3,3,3>();


            //Mix with accelerometer using bayesian filter


            ScaleEstimator::EstAcelLsq4(-istate.Vgv/dt_frame,istate.Av,R,dt_frame);
            ScaleEstimator::MeanAcel4(new_buf.imu.acel,istate.As,R);

            Xgva=Xgv;


            istate.Rv=(P_V/(dt_frame*dt_frame*dt_frame*dt_frame));
            istate.Qrot=P_W;
            istate.QKp=std::min((1/(1/istate.Rv(0,0)+1/istate.Rv(1,1)+1/istate.Rv(2,2)))*cf->ScaleStdDevMult*cf->ScaleStdDevMult,cf->ScaleStdDevMax);


            if(n_frame>4)
                K=ScaleEstimator::estKaGMEKBias(istate.As,istate.Av,1,R,istate.X,istate.P,istate.Qrot,istate.Qbias,istate.QKp,istate.Rg,istate.Rs,istate.Rv,istate.g_est,istate.b_est,W_Xgv,Xgva,cf->g_module);
            //***** Forward Rotate the old edge-map points *****
            old_buf.ef->rotate_keylines(R0.get_matrix());


        }else{                  //Regular procesing

            new_buf.gt->Minimizer_RV<double>(V,W,P_V,P_W,*old_buf.ef,cf->TrackerMatchThresh,cf->TrackerIterNum,cf->TrackerInitType,cf->ReweigthDistance,error_vel,error_score,s_rho_q,cf->MatchNumThresh,cf->TrackerInitIterNum);

            //***** Match from the old EdgeMap to the new one using the information from the minimization *****
            old_buf.ef->FordwardMatch(new_buf.ef);


            //***** extract rotation matrix *****
            SO3 <>R0(W);                    //R0 is a forward rotation
            R.T()=R0.get_matrix()*R.T();    //R is a backward rotation


            //***** Forward Rotate the old edge-map points *****
            old_buf.ef->rotate_keylines(R0.get_matrix());

        }
      if(util::isNaN(V) || util::isNaN(W))  //Check for minimization errors
        {
            P_V=Identity*1e50;
            V=Zeros;

            Kp=1;
            P_Kp=1e50;

            EstimationOk=false;

            printf("\nCamara Frontal: error in the estimation, not many KeyLines (%d)?\n",new_buf.ef->KNum());
        }else{






            COND_TIME_DEBUG(tlist.push_new();)



            COND_TIME_DEBUG(tlist.push_new();)

            COND_TIME_DEBUG(tlist.push_new();)

            //***** Match from the new EdgeMap to the old one searching on the stereo line *****

            //Because the old edge map mask is used, New keylines and Translation are back rotated
            klm_num=new_buf.ef->directed_matching(V,P_V,R,old_buf.ef,cf->MatchThreshModule,cf->MatchThreshAngle,cf->SearchRange,cf->LocationUncertaintyMatch);

            if(klm_num<cf->MatchThreshold){     //If matching keylines are below a certain threshold, reestart the estimation

                P_V=Identity*1e50;
                V=Zeros;

                Kp=1;
                P_Kp=10;

                EstimationOk=false;

                printf("\nCamara Frontal: restarting the estimation, match threshold low (%d,%d)?\n",new_buf.ef->KNum(),klm_num);
            }else{


                COND_TIME_DEBUG(tlist.push_new();)

                //****** Regularize the EdgeMap Depth ******

                for(int i=0;i<1;i++)
                        new_buf.ef->Regularize_1_iter(cf->RegularizeThresh);


                COND_TIME_DEBUG(tlist.push_new();)

                //****** Improve Depth using kalman filter ****

                new_buf.ef->UpdateInverseDepthKalman(V,P_V,P_W,cf->ReshapeQAbsolute,cf->ReshapeQRelative,cf->LocationUncertainty); //1e-5



                COND_TIME_DEBUG(tlist.push_new();)


                //****** Optionally reescale the EdgeMap's Depth
                Kp=new_buf.ef->EstimateReScaling(P_Kp,RHO_MAX,1,cf->DoReScaling>0);
               // cout << P_Kp<<"\n";


                COND_TIME_DEBUG(tlist.push_new();)


            }

        }



        COND_TIME_DEBUG(printf("\nr%f b%f v%f f%f w%f m%f r%f k%f t%f %f %d %f %s\n",tlist[0],tlist[1],tlist[2],tlist[3],\
               tlist[4],tlist[5],tlist[6],tlist[7],tlist.total(),dtp,new_buf.ef->KNum(),s_rho_q,EstimationOk?"OK":"NOK");)



        //Estimate position and pose incrementally
        Pose=Pose*R;
        Pos+=-Pose*V*K;

        P_V/=dt_frame*dt_frame;

        //Pass the buffer to the next thread

        old_buf.dt=dt_frame;
        old_buf.K=K;
        old_buf.Kp=Kp;
        old_buf.RKp=P_Kp;

        old_buf.Rot=R;
        old_buf.RotLie=SO3<>(R).ln();
        old_buf.Vel=-V*K/dt_frame;

        old_buf.Pose=Pose;
        old_buf.PoseLie=SO3<>(Pose).ln();
        old_buf.Pos=Pos;

        old_buf.s_rho_p=s_rho_q;

        old_buf.EstimationOK=EstimationOk;

        old_buf.imustate=istate;

      /*  if((old_buf.p_id%25)==0){
            cf->kf_list.push_back(keyframe(*old_buf.ef,*old_buf.gt,old_buf.t,old_buf.Rot,old_buf.RotLie,old_buf.Vel,old_buf.Pose,old_buf.PoseLie,old_buf.Pos));

            std::cout <<"\nadded keyframe\n";
        }*/

        if(cf->system_reset){   //Do a depth reset to the New Edgemap
            for (auto &kl: (*new_buf.ef)) {
                kl.rho=RhoInit;                     //Rho & srho init point
                kl.s_rho=RHO_MAX;
            }
            Pose=Identity;  //Reset trayectory
            Pos=Zeros;
            W=Zeros;
            V=Zeros;

            cf->system_reset=false;
        }

        cf->pipe.ReleaseBuffer(1);
        cf->pipe.ReleaseBuffer(2);

        n_frame++;



    }



    Thr2.join();

    return;
}

void REBVO::ThirdThread(REBVO *cf){
    using namespace TooN;

    bool quit=false;

    /*****   Set cpu Afinity of the thread   ******/

    if(cf->cpuSetAffinity){
        cpu_set_t cpusetp;

        CPU_ZERO(&cpusetp);
        CPU_SET(cf->cpu2,&cpusetp);
        if(pthread_setaffinity_np(pthread_self(),sizeof(cpu_set_t),&cpusetp)!=0){
            printf("\nThird Thread: No puedo setear CPU affinity!\n");
            cf->quit=true;
            return;
        }
    }



    /***** Init a pipe to buffer the NET packets *****
            This is nedded to compesate for the delay
            introduced by the MFC encoder, if used,
            and can be configured with the EdgeMapDelay
            tag */


    const int flen=MFC_MAX_STREAM_SIZE+sizeof(net_packet_hdr)+sizeof(net_keyline)*KEYLINE_MAX;
    int hdr_payload=0;


    util::CircListIndexer net_buf_inx(cf->EdgeMapDelay+1);
    unsigned char **net_pak= new unsigned char*[net_buf_inx.Size()];

    for(int i=0;i<net_buf_inx.Size();i++){
        net_pak[i]=new unsigned char[flen];
        memset(net_pak[i],0,sizeof(net_packet_hdr));
    }

    net_packet_hdr* net_hdr=(net_packet_hdr*)net_pak [0];


    /***** Start UDP comm port if needed******/

    udp_port *com_port=NULL;
    if(cf->VideoNetEnabled>0){
        com_port=new udp_port(cf->VideoNetHost.data(),cf->VideoNetPort);

        if(com_port->Error()){
            cf->quit=true;
            return;
        }
        printf("\nCamaraFrontal: Socket iniciado en %s:%d\n",cf->VideoNetHost.data(),cf->VideoNetPort);

    }

    /****** Init buffer for pla video save (no container) if needed ******/

    int VideoSaveIndex=0,VideoSaveBuffersize=0;
    char * VideoSaveBuffer=NULL;

    if(cf->VideoSave>0){
        VideoSaveBuffer=new char[cf->VideoSaveBuffersize];
        VideoSaveBuffersize=cf->VideoSaveBuffersize;
    }


    /****** Init Encoder if needed ******/

    const Size2D ImageSize(cf->cam->sz);

    VideoEncoder *encoder=NULL;

    if(cf->VideoNetEnabled>0 || cf->VideoSave>0){
        switch(cf->encoder_type){
        case VIDEO_ENCODER_TYPE_MJPEG:
            encoder=new MJPEGEncoder(ImageSize,90);
            break;
        case VIDEO_ENCODER_TYPE_MFC:
        {
            EncoderMFC *encoder_mfc=new EncoderMFC(cf->encoder_dev.data());
            int stb_sz=MFC_MAX_STREAM_SIZE;
            if(encoder_mfc->Initialize(ImageSize.w,ImageSize.h,V4L2_PIX_FMT_MPEG4,15,128000,2,2,stb_sz)<0){
                cf->quit=true;
                return;
            }
            encoder_mfc->Autoconfig();
            encoder=encoder_mfc;
        }
            break;
        case VIDEO_ENCODER_TYPE_RAW:
        default:

            encoder=new VideoEncoder(ImageSize);
            break;

        }
    }


    /****** Init log file if needed ******/

    ofstream a_log,t_log;
    int a_log_inx=0;

    if(cf->SaveLog){

        a_log.open(cf->LogFile.data());
        if(!a_log.is_open()){
            cout <<"\nREBVO: Cannot open log file\n";
            cf->quit=true;
            return;
        }

        a_log<< std::scientific<<std::setprecision(16);


        t_log.open(cf->TrayFile.data());
        if(!t_log.is_open()){
            cout <<"\nREBVO: Cannot open trayectory file\n";
            cf->quit=true;
            return;
        }

    }


    ofstream h_log;
    h_log.open("histo_log.txt");
    const uint h_num=500;
    uint histo[h_num];

    COND_TIME_DEBUG(util::timer t_proc;)

    /****** Main Loop does three optional things:
     * Encodes video and sends over UDP
     * Saves encoded video
     * Saves LOG in .m format
     * *******************************************/

    while(!cf->quit && !quit){

        PipeBuffer pbuf=cf->pipe.RequestBuffer(3);          //Request buffer for player 3

        if(pbuf.quit){
            cf->pipe.ReleaseBuffer(3);
            break;
        }

        COND_TIME_DEBUG(t_proc.start();)


        if(cf->VideoNetEnabled>0){

            //****** Prepare network buffer header *****/


            net_hdr= (net_packet_hdr*)net_pak [net_buf_inx-1];
            net_hdr->key_num=0;

            net_hdr->kline_num=copy_net_keyline(*pbuf.ef,\
                                    (net_keyline*)&net_pak [net_buf_inx-1][sizeof(net_packet_hdr)],KEYLINE_MAX,pbuf.K);


            copy_net_keyline_nextid(*pbuf.ef,\
                                    (net_keyline*)&net_pak [net_buf_inx-1][sizeof(net_packet_hdr)],KEYLINE_MAX);


            net_hdr->km_num=pbuf.ef->NumMatches();

            net_hdr->k=pbuf.K;
            net_hdr->max_rho=RHO_MAX;

            net_hdr= (net_packet_hdr*)net_pak [net_buf_inx];

            net_hdr->w=ImageSize.w;
            net_hdr->h=ImageSize.h;
            net_hdr->encoder_type=encoder->GetEncoderType();


            net_hdr->nav.dt=pbuf.dt;
            net_hdr->nav.Pos=pbuf.Pos;
            net_hdr->nav.Pose=pbuf.PoseLie;
            net_hdr->nav.Vel=pbuf.Vel;
            net_hdr->nav.Rot=pbuf.RotLie;


            hdr_payload=sizeof(net_packet_hdr)+sizeof(net_keyline)*net_hdr->kline_num;

            //****** Push frame on encoder *****/

            encoder->PushFrame(pbuf.imgc->Data());

            //****** Pop encoder frame *****/

            int n=encoder->PopFrame((char *)&net_pak[net_buf_inx][hdr_payload]\
                                    ,flen-hdr_payload);


            if(n>=0){                                   //If encoding OK transmit

                net_hdr->data_size=hdr_payload+n;
                net_hdr->jpeg_size=n;

                if (!com_port->SendFragmented(net_pak[net_buf_inx],net_hdr->data_size,65e3)){
                    printf("\nMTrack: Error enviando paquete de %d bytes a %s:%d! %d: %s\n",net_hdr->data_size,cf->VideoNetHost.data(),cf->VideoNetPort\
                           ,errno, strerror(errno));
                }

                if(cf->VideoSave){                      //Optionaly save on file buffer
                    memcpy(&VideoSaveBuffer[VideoSaveIndex],&net_pak[net_buf_inx][hdr_payload],std::min(n,VideoSaveBuffersize-VideoSaveIndex));
                }
            }


            ++net_buf_inx;


        }else if(cf->VideoSave && VideoSaveBuffer!=NULL && VideoSaveBuffersize>VideoSaveIndex){

            //****** If only video file buffer enabled encode and decode *****//

            encoder->PushFrame(pbuf.imgc->Data());
            int n=encoder->PopFrame(&VideoSaveBuffer[VideoSaveIndex],VideoSaveBuffersize-VideoSaveIndex);
            VideoSaveIndex+=n;

        }

        if(cf->SaveLog){

            //******* Save log data directly on file ******//

            a_log_inx++;

            a_log<<"Kp_cv("<<a_log_inx<<",:)="<<pbuf.Kp<<";\n";
            a_log<<"RKp_cv("<<a_log_inx<<",:)="<<pbuf.RKp<<";\n";
            a_log<<"Rot_cv("<<a_log_inx<<",:,:)=["<<pbuf.Rot(0,0)<<","<<pbuf.Rot(0,1)<<","<<pbuf.Rot(0,2)<<";"\
                <<pbuf.Rot(1,0)<<","<<pbuf.Rot(1,1)<<","<<pbuf.Rot(1,2)<<";"\
               <<pbuf.Rot(2,0)<<","<<pbuf.Rot(2,1)<<","<<pbuf.Rot(2,2)<<"];\n";
            a_log<<"Vel_cv("<<a_log_inx<<",:)=["<<pbuf.Vel[0]<<","<<pbuf.Vel[1]<<","<<pbuf.Vel[2]<<"];\n";
            a_log<<"t_cv("<<a_log_inx<<",:)="<<pbuf.t<<";\n";
            a_log<<"dt_cv("<<a_log_inx<<",:)="<<pbuf.dt<<";\n";
            a_log<<"i_cv("<<a_log_inx<<",:)="<<pbuf.p_id<<";\n";
            a_log<<"Pose_cv("<<a_log_inx<<",:,:)=["<<pbuf.Pose(0,0)<<","<<pbuf.Pose(0,1)<<","<<pbuf.Pose(0,2)<<";"\
                <<pbuf.Pose(1,0)<<","<<pbuf.Pose(1,1)<<","<<pbuf.Pose(1,2)<<";"\
               <<pbuf.Pose(2,0)<<","<<pbuf.Pose(2,1)<<","<<pbuf.Pose(2,2)<<"];\n";
            a_log<<"Pos_cv("<<a_log_inx<<",:)=["<<pbuf.Pos[0]<<","<<pbuf.Pos[1]<<","<<pbuf.Pos[2]<<"];\n";
            a_log<<"K_cv("<<a_log_inx<<",:)="<<pbuf.K<<";\n";
            a_log<<"KLN_cv("<<a_log_inx<<",:)="<<pbuf.ef->KNum()<<";\n";


            a_log<<"Giro_cv("<<a_log_inx<<",:)=["<<pbuf.imu.giro[0]<<","<<pbuf.imu.giro[1]<<","<<pbuf.imu.giro[2]<<"];\n";
            a_log<<"Acel_cv("<<a_log_inx<<",:)=["<<pbuf.imu.acel[0]<<","<<pbuf.imu.acel[1]<<","<<pbuf.imu.acel[2]<<"];\n";

            a_log<<"GBias_cv("<<a_log_inx<<",:)=["<<pbuf.imustate.Bg[0]<<","<<pbuf.imustate.Bg[1]<<","<<pbuf.imustate.Bg[2]<<"];\n";
            a_log<<"dWv_cv("<<a_log_inx<<",:)=["<<pbuf.imustate.dWv[0]<<","<<pbuf.imustate.dWv[1]<<","<<pbuf.imustate.dWv[2]<<"];\n";
            a_log<<"dWgv_cv("<<a_log_inx<<",:)=["<<pbuf.imustate.dWgv[0]<<","<<pbuf.imustate.dWgv[1]<<","<<pbuf.imustate.dWgv[2]<<"];\n";


            a_log<<"g_cv("<<a_log_inx<<",:)=["<<pbuf.imustate.g_est[0]<<","<<pbuf.imustate.g_est[1]<<","<<pbuf.imustate.g_est[2]<<"];\n";
            a_log<<"VBias_cv("<<a_log_inx<<",:)=["<<pbuf.imustate.b_est[0]<<","<<pbuf.imustate.b_est[1]<<","<<pbuf.imustate.b_est[2]<<"];\n";
            a_log<<"Av_cv("<<a_log_inx<<",:)=["<<pbuf.imustate.Av[0]<<","<<pbuf.imustate.Av[1]<<","<<pbuf.imustate.Av[2]<<"];\n";
            a_log<<"As_cv("<<a_log_inx<<",:)=["<<pbuf.imustate.As[0]<<","<<pbuf.imustate.As[1]<<","<<pbuf.imustate.As[2]<<"];\n";

            a_log.flush();

            //******* Save trayectory ************//

            t_log << std::scientific<<std::setprecision(16)<< pbuf.t << " " <<pbuf.Pos << " "<<util::LieRot2Quaternion(pbuf.PoseLie)<<"\n";
            t_log.flush();

        }


        COND_TIME_DEBUG(printf("\nCamara TT Dtp=%f\n",t_proc.stop());)


        pbuf.ef->DebugMatchHisto(500,h_num,histo);
        for(int i=0;i<h_num-1;i++)
            h_log<<histo[i]<<",";
        h_log<<histo[h_num-1]<<"\n";
        h_log.flush();
        //******** The system can optionaly take a snapshot (use for debuging purposes) ******//

        if(cf->saveImg){
            cf->saveImg=false;
            char name[128];
            snprintf(name,sizeof(name),"Snap%d.ppm",cf->snap_n);
            cout << "\nCamara Frontal: Tomando foto"<<cf->snap_n++<<"\n";
            SavePPM(name,pbuf.imgc->Data(),ImageSize.w,ImageSize.h);
        }

        cf->pipe.ReleaseBuffer(3);

    }

    // ****** All the video is saved in RAM and saved at the end, for avoiding delays ******//

    if(cf->VideoSave){

        printf("\nCamara Frontal: Saving video file...\n");

        ofstream video_of;
        video_of.open(cf->VideoSaveFile.data(),ios_base::trunc);

        if(video_of.is_open()){

            video_of.write(VideoSaveBuffer,VideoSaveIndex);
            video_of.close();

        }else{
            printf("\nCamara Frontal: Failed to open video save file\n");
        }
    }

    if(encoder)
        delete encoder;

    if(a_log.is_open())
        a_log.close();

    if(t_log.is_open())
        t_log.close();

    h_log.close();

    cf->quit=true;

    return;


}




REBVO::REBVO(Configurator &config)
    :pipe(CBUFSIZE,4)
{
    Size2D ImageSize;
    float z_f_x;
    float z_f_y;
    float pp_y;
    float pp_x;
    double kc[2];

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

    InitOK&=config.GetConfigByName("Camera","ImageWidth",ImageSize.w,true);
    InitOK&=config.GetConfigByName("Camera","ImageHeight",ImageSize.h,true);

    InitOK&=config.GetConfigByName("Camera","ZfX",z_f_x,true);
    InitOK&=config.GetConfigByName("Camera","ZfY",z_f_y,true);

    InitOK&=config.GetConfigByName("Camera","PPx",pp_x,true);
    InitOK&=config.GetConfigByName("Camera","PPy",pp_y,true);

    InitOK&=config.GetConfigByName("Camera","KcR2",kc[0],true);
    InitOK&=config.GetConfigByName("Camera","KcR4",kc[1],true);

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
    InitOK&=config.GetConfigByName("IMU","GiroMeasStdDev",GiroMeasStdDev,true);
    InitOK&=config.GetConfigByName("IMU","GiroBiasStdDev",GiroBiasStdDev,true);
    InitOK&=config.GetConfigByName("IMU","InitBias",InitBias,true);
    InitOK&=config.GetConfigByName("IMU","g_module",g_module,true);


    InitOK&=config.GetConfigByName("IMU","AcelMeasStdDev",AcelMeasStdDev,true);
    InitOK&=config.GetConfigByName("IMU","g_module_uncer",g_module_uncer,true);
    InitOK&=config.GetConfigByName("IMU","VBiasStdDev",VBiasStdDev,true);
    InitOK&=config.GetConfigByName("IMU","ScaleStdDevMult",ScaleStdDevMult,true);
    InitOK&=config.GetConfigByName("IMU","ScaleStdDevMax",ScaleStdDevMax,true);






    if(ImuMode==2){
        InitOK&=config.GetConfigByName("IMU","ImuFile",ImuFile,true);

        InitOK&=config.GetConfigByName("IMU","TimeScale",ImuTimeScale,true);
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

