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

        com_port->setBlock(cf->BlockingUDP>0);

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
            net_hdr->nav.Pos=pbuf.nav.Pos;
            net_hdr->nav.Pose=pbuf.nav.PoseLie;
            net_hdr->nav.Vel=pbuf.nav.Vel;
            net_hdr->nav.Rot=pbuf.nav.RotLie;


            hdr_payload=sizeof(net_packet_hdr)+sizeof(net_keyline)*net_hdr->kline_num;

            //****** Push frame on encoder *****/

            encoder->PushFrame(pbuf.imgc->Data());

            //****** Pop encoder frame *****/

            int n=encoder->PopFrame((char *)&net_pak[net_buf_inx][hdr_payload]\
                                    ,flen-hdr_payload);


            if(n>=0){                                   //If encoding OK transmit

                net_hdr->data_size=hdr_payload+n;
                net_hdr->jpeg_size=n;

                if (!com_port->SendFragmented(net_pak[net_buf_inx],net_hdr->data_size,32e3)){
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
            a_log<<"Rot_cv("<<a_log_inx<<",:,:)=["<<pbuf.nav.Rot(0,0)<<","<<pbuf.nav.Rot(0,1)<<","<<pbuf.nav.Rot(0,2)<<";"\
                <<pbuf.nav.Rot(1,0)<<","<<pbuf.nav.Rot(1,1)<<","<<pbuf.nav.Rot(1,2)<<";"\
               <<pbuf.nav.Rot(2,0)<<","<<pbuf.nav.Rot(2,1)<<","<<pbuf.nav.Rot(2,2)<<"];\n";
            a_log<<"Vel_cv("<<a_log_inx<<",:)=["<<pbuf.nav.Vel[0]<<","<<pbuf.nav.Vel[1]<<","<<pbuf.nav.Vel[2]<<"];\n";
            a_log<<"RotGiro_cv("<<a_log_inx<<",:)=["<<pbuf.nav.RotGiro[0]<<","<<pbuf.nav.RotGiro[1]<<","<<pbuf.nav.RotGiro[2]<<"];\n";
            a_log<<"t_cv("<<a_log_inx<<",:)="<<pbuf.t<<";\n";
            a_log<<"dt_cv("<<a_log_inx<<",:)="<<pbuf.dt<<";\n";
            a_log<<"i_cv("<<a_log_inx<<",:)="<<pbuf.p_id<<";\n";
            a_log<<"Pose_cv("<<a_log_inx<<",:,:)=["<<pbuf.nav.Pose(0,0)<<","<<pbuf.nav.Pose(0,1)<<","<<pbuf.nav.Pose(0,2)<<";"\
                <<pbuf.nav.Pose(1,0)<<","<<pbuf.nav.Pose(1,1)<<","<<pbuf.nav.Pose(1,2)<<";"\
               <<pbuf.nav.Pose(2,0)<<","<<pbuf.nav.Pose(2,1)<<","<<pbuf.nav.Pose(2,2)<<"];\n";
            a_log<<"Pos_cv("<<a_log_inx<<",:)=["<<pbuf.nav.Pos[0]<<","<<pbuf.nav.Pos[1]<<","<<pbuf.nav.Pos[2]<<"];\n";
            a_log<<"K_cv("<<a_log_inx<<",:)="<<pbuf.K<<";\n";
            a_log<<"KLN_cv("<<a_log_inx<<",:)="<<pbuf.ef->KNum()<<";\n";


            a_log<<"Giro_cv("<<a_log_inx<<",:)=["<<pbuf.imu.giro[0]<<","<<pbuf.imu.giro[1]<<","<<pbuf.imu.giro[2]<<"];\n";
            a_log<<"Acel_cv("<<a_log_inx<<",:)=["<<pbuf.imu.acel[0]<<","<<pbuf.imu.acel[1]<<","<<pbuf.imu.acel[2]<<"];\n";
            a_log<<"CAcel_cv("<<a_log_inx<<",:)=["<<pbuf.imu.cacel[0]<<","<<pbuf.imu.cacel[1]<<","<<pbuf.imu.cacel[2]<<"];\n";
            a_log<<"DGiro_cv("<<a_log_inx<<",:)=["<<pbuf.imu.dgiro[0]<<","<<pbuf.imu.dgiro[1]<<","<<pbuf.imu.dgiro[2]<<"];\n";

            a_log<<"GBias_cv("<<a_log_inx<<",:)=["<<pbuf.imustate.Bg[0]<<","<<pbuf.imustate.Bg[1]<<","<<pbuf.imustate.Bg[2]<<"];\n";
            a_log<<"dWv_cv("<<a_log_inx<<",:)=["<<pbuf.imustate.dWv[0]<<","<<pbuf.imustate.dWv[1]<<","<<pbuf.imustate.dWv[2]<<"];\n";
            a_log<<"dWgv_cv("<<a_log_inx<<",:)=["<<pbuf.imustate.dWgv[0]<<","<<pbuf.imustate.dWgv[1]<<","<<pbuf.imustate.dWgv[2]<<"];\n";


            a_log<<"g_cv("<<a_log_inx<<",:)=["<<pbuf.imustate.g_est[0]<<","<<pbuf.imustate.g_est[1]<<","<<pbuf.imustate.g_est[2]<<"];\n";
            a_log<<"VBias_cv("<<a_log_inx<<",:)=["<<pbuf.imustate.b_est[0]<<","<<pbuf.imustate.b_est[1]<<","<<pbuf.imustate.b_est[2]<<"];\n";
            a_log<<"Av_cv("<<a_log_inx<<",:)=["<<pbuf.imustate.Av[0]<<","<<pbuf.imustate.Av[1]<<","<<pbuf.imustate.Av[2]<<"];\n";
            a_log<<"As_cv("<<a_log_inx<<",:)=["<<pbuf.imustate.As[0]<<","<<pbuf.imustate.As[1]<<","<<pbuf.imustate.As[2]<<"];\n";

            a_log<<"Posgv_cv("<<a_log_inx<<",:)=["<<pbuf.imustate.Posgv[0]<<","<<pbuf.imustate.Posgv[1]<<","<<pbuf.imustate.Posgv[2]<<"];\n";

            a_log.flush();

            //******* Save trayectory ************//

            t_log << std::scientific<<std::setprecision(18)<< pbuf.t/cf->ImuTimeScale << " " <<pbuf.nav.Pos << " "<<util::LieRot2Quaternion(pbuf.nav.PoseLie)<<"\n";
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


