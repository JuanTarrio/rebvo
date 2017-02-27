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


#include "visualizer/visualizer.h"


#include <stdint.h>

#include "VideoLib/video_encoder.h"
#include "VideoLib/videodecoder.h"

#include "visualizer/gl_viewer.h"
#include "visualizer/depth_filler.h"
#include "CommLib/udp_port.h"

#include <TooN/so3.h>

namespace rebvo {

#define MFC_MAX_STREAM_SIZE (2*1024*1024)


int visualizer::OnPaint(XVideoContext *xvc, void *param){

    visualizer *fc=(visualizer *)param;
    XSetLineAttributes(xvc->display, xvc->gc, 1, LineSolid, CapRound, JoinRound);

    char    msg[256];

    if(fc->ShowImg<1){

        XSetForeground(xvc->display,xvc->gc,0);
        XFillRectangle(xvc->display,xvc->pixmap,xvc->gc,0,0,xvc->width,xvc->height);
    }

    for(int i=0;i<fc->net_kln && fc->ShowImg<2;i++){

        int c;

        c=(1/pow((float)fc->net_kl[i].s_rho/NET_RHO_SCALING,2))*10;

        util::keep_min(c,255);
        c=(c<<16)+(255-c);
        XSetForeground(xvc->display,xvc->gc,c);

        XDrawPoint(xvc->display, xvc->pixmap, xvc->gc,fc->net_kl[i].qx,fc->net_kl[i].qy);

        if(fc->ShowTrails){
            XSetForeground(xvc->display,xvc->gc,0x00ff00);
            XDrawLine(xvc->display, xvc->pixmap, xvc->gc,fc->net_kl[i].qx,fc->net_kl[i].qy\
                      ,fc->net_kl[i].qx+(fc->net_kl[i].extra.flow.x-127.0)/10.0*10,fc->net_kl[i].qy+(fc->net_kl[i].extra.flow.y-127.0)/10.0*10);
        }

    }




    XSetForeground(xvc->display,xvc->gc,0xFFFFF);


    float scale=520/fc->depth_show_max;
    Point2DI cuadp={(int)xvc->width/2,(int)xvc->height/2};



    XSetForeground(xvc->display,xvc->gc,0xFF00FF);
    XDrawLine(xvc->display, xvc->pixmap, xvc->gc,cuadp.x,cuadp.y-2,cuadp.x,cuadp.y+2);
    XDrawLine(xvc->display, xvc->pixmap, xvc->gc,cuadp.x-2,cuadp.y,cuadp.x+2,cuadp.y);

    XSetForeground(xvc->display,xvc->gc,0xFFFF00);

    float px=cuadp.x/scale,py=cuadp.y/scale;


    float px1,py1;
    if(fc->pos_tray.size()>1){
        TooN::Vector <3> pf=fc->pos_tray[fc->pos_tray.size()-1];
        for(int i=0;i<(int)fc->pos_tray.size()-1;i++){

            TooN::Vector <3> p0=fc->Pose.T()*(pf-fc->pos_tray[i]);
            TooN::Vector <3> p1=fc->Pose.T()*(pf-fc->pos_tray[i+1]);

            px=cuadp.x-p0[0]*scale;py=cuadp.y+p0[2]*scale;
            px1=cuadp.x-p1[0]*scale;py1=cuadp.y+p1[2]*scale;


            XDrawLine(xvc->display,xvc->pixmap,xvc->gc,px,py,\
                      px1,py1);
        }
    }

    XSetForeground(xvc->display,xvc->gc,0x00FFFF);

    snprintf(msg,sizeof(msg),"FPS= %2.1f dFPS= %2.1f T2I= %.2f KL= %d KM= %d",fc->pFPS,fc->dFPS,fc->time2impact,\
             fc->net_kln,fc->net_hdr->km_num);
    XDrawString(xvc->display, xvc->pixmap, xvc->gc, 10, 10,msg, strlen(msg));



    return 0;
}


int visualizer::OnPaintDepth(XVideoContext *xvc, void *param){

    visualizer *fc=(visualizer *)param;

    char msg[128];


    XSetForeground(xvc->display,xvc->gc,0x00FF00);

    XSetLineAttributes(xvc->display, xvc->gc, 1, LineSolid, CapRound, JoinRound);

    float scale=xvc->height/fc->depth_show_max;
    Point2DF cuadp={(float)(xvc->width/2.0),(float)(xvc->height-scale*1.0)};


    for(int i=0;i<fc->net_kln;i++){


        bool connected=false;
        float x1,x2,y1,y2,z;

        int c=(1/util::norm2((float)fc->net_kl[i].s_rho/NET_RHO_SCALING))*10;

        z=NET_RHO_SCALING/fc->net_kl[i].rho;

        x1=((float)fc->net_kl[i].qx-(float)fc->ImageSize.w/2)*z/fc->ZfX;
        y1=z;


        if(fc->net_kl[i].n_kl>=0 && c>20){

            int nkl=fc->net_kl[i].n_kl;


            if(nkl>=KEYLINE_MAX){
                nkl-=KEYLINE_MAX;


                z=NET_RHO_SCALING/fc->net_kl[nkl].rho;
                x2=((float)fc->net_kl[nkl].qx+256.0-(float)fc->ImageSize.w/2)*z/fc->ZfX;
                y2=z;


                if(1/util::norm2((float)fc->net_kl[nkl].s_rho/NET_RHO_SCALING)>2)
                    connected=true;


            }else{

                z=NET_RHO_SCALING/fc->net_kl[nkl].rho;
                x2=((float)fc->net_kl[nkl].qx+0.0-(float)fc->ImageSize.w/2)*z/fc->ZfX;
                y2=z;

                if(1/util::norm2((float)fc->net_kl[nkl].s_rho/NET_RHO_SCALING)>2)
                    connected=true;

            }

        }

        util::keep_min(c,255);
        c=(c<<16)+(255-c);
        XSetForeground(xvc->display,xvc->gc,c);


        if(connected)
            XDrawLine(xvc->display, xvc->pixmap, xvc->gc,cuadp.x+x1*scale,cuadp.y-y1*scale,cuadp.x+x2*scale,cuadp.y-y2*scale);
        else
            XDrawPoint(xvc->display, xvc->pixmap, xvc->gc,cuadp.x+x1*scale,cuadp.y-y1*scale);


    }


    XSetForeground(xvc->display,xvc->gc,0xFF00FF);

    XDrawLine(xvc->display,xvc->pixmap,xvc->gc,cuadp.x-0.5*scale,cuadp.y,\
              cuadp.x+0.5*scale,cuadp.y);

    XDrawLine(xvc->display,xvc->pixmap,xvc->gc,cuadp.x,cuadp.y-0.5*scale,\
              cuadp.x,cuadp.y+0.5*scale);


    XSetForeground(xvc->display,xvc->gc,0xFFFF00);

    float px=cuadp.x/scale,pz=cuadp.y/scale;

    float px1,pz1;
    if(fc->pos_tray.size()>1){
        TooN::Vector <3> pf=fc->pos_tray[fc->pos_tray.size()-1];
        for(int i=0;i<(int)fc->pos_tray.size()-1;i++){

            TooN::Vector <3> p0=fc->Pose.T()*(pf-fc->pos_tray[i]);
            TooN::Vector <3> p1=fc->Pose.T()*(pf-fc->pos_tray[i+1]);

            px=cuadp.x-p0[0]*scale;pz=cuadp.y+p0[2]*scale;
            px1=cuadp.x-p1[0]*scale;pz1=cuadp.y+p1[2]*scale;


            XDrawLine(xvc->display,xvc->pixmap,xvc->gc,px,pz,\
                      px1,pz1);
        }
    }

    double max_z=fc->net_hdr->k/fc->net_hdr->max_rho;

    XSetForeground(xvc->display,xvc->gc,0xFF00FF);
    XDrawLine(xvc->display, xvc->pixmap, xvc->gc,cuadp.x-1*scale,cuadp.y-max_z*scale,cuadp.x+1*scale,cuadp.y-max_z*scale);

    XSetForeground(xvc->display,xvc->gc,0xFF00FF);
    XSetForeground(xvc->display,xvc->gc,0x00FFFF);
    snprintf(msg,sizeof(msg),"Rango = %.1fm KFix = %.2f",fc->depth_show_max,fc->KFix);
    XSetForeground(xvc->display,xvc->gc,0xFFFFFF);
    XDrawString(xvc->display,xvc->pixmap,xvc->gc,10,10,msg,strlen(msg));

    return 0;
}


int visualizer::Run(){

    if(!carga)
        return -1;

    using namespace TooN;


    Size2D frameSize=ImageSize;


    timeval t_pfps0,t_pfps1,t_joy0;

    XVideoContext 	xvc;
    XVideoContext 	xvc_d;


    int br;
    char k;


    int frame_count=0,draw_frame_count=0;

    bool new_frame;


    cam_model::rad_tan_distortion Kc={0,0,0,0,0};
    cam_model cam({(float)(ImageSize.w/2.0),(float)(ImageSize.h/2.0)},{(float)ZfX,(float)ZfX},Kc,frameSize);


    VideoDecoder decoderMPEG4(AV_CODEC_ID_MPEG4,frameSize.w,frameSize.h);
    VideoDecoder decoderMJPEG(AV_CODEC_ID_MJPEG,frameSize.w,frameSize.h);

    RGB24Pixel *data_depth=new RGB24Pixel[640*520];
    memset(data_depth,0,640*520*sizeof(RGB24Pixel));

    RGB24Pixel *data=new RGB24Pixel[frameSize.w*frameSize.h];

    const int net_k_max=KEYLINE_MAX;
    const int flen=MFC_MAX_STREAM_SIZE+sizeof(net_packet_hdr)+sizeof(net_keyline)*net_k_max+FF_INPUT_BUFFER_PADDING_SIZE;
    unsigned char *net_pak= new unsigned char[flen];
    net_hdr= (net_packet_hdr*)net_pak;


    udp_port com_port(Host.data(),Port,true,1e6);

    if(com_port.Error()){
        return 0;
    }

    com_port.setBlock(false);


    printf("\nVisualizer: Socket iniciado en %s:%d\n",Host.data(),Port);


    const int EMSaveNum=EdgeMapSaveNumber<1?1:EdgeMapSaveNumber;

    std::vector<depth_filler> d_filler(EdgeMapSaveNumber,depth_filler(cam,{10,10},depth_filler::BOUND_NONE));

    int NextEMIndex=0;

    if(XLibView>0){

        if(IniciarVideoOutput(&xvc,frameSize.w,frameSize.h)){
            printf("\nVisualizer: No puedo iniciar Video\n");
            return 0;
        }

    }if(XLibView>1){

        if(IniciarVideoOutput(&xvc_d,640,520)){
            printf("\nVisualizer: No puedo iniciar Video\n");
            return 0;
        }
    }

    RenderParams rp;

    rp.net_kl=&net_kl;
    rp.net_kln=&net_kln;
    rp.net_klistn=1;
    rp.zf=ZfX;
    rp.pp.x=ImageSize.w/2;
    rp.pp.y=ImageSize.h/2;

    rp.net_kpn=0;
    rp.d_filler=&d_filler;
    rp.pos_tray=&pos_tray;
    rp.Pose=&Pose;
    rp.current_em=NextEMIndex;
    rp.total_em=EMSaveNum;
    rp.ref_err=Zeros;


    rp.img_data=new Image<RGB24Pixel>(data,frameSize);

    const int glv_num=ViewNumber;

    gl_viewer * glv[glv_num];

    for(int i=0;i<glv_num;i++)
        glv[i]=new gl_viewer(ImageSize.w,ImageSize.h,"GL View",atan(ImageSize.h/2/ZfX)*360/M_PI);

    rp.draw_crash_cuad=true;


    gettimeofday(&t_pfps1,NULL);
    t_pfps0=t_pfps1;
    t_joy0=t_pfps1;

    bool run=true;
    while(run){

        new_frame=false;

        while((br=com_port.RecvFragmented((unsigned char*)net_hdr,flen))>0){


            if(br!=net_hdr->data_size){
                printf("\nDisplay Frontal: Wrong Recv Size! %d!=%d\n",br,net_hdr->data_size);
                goto recv_frame_err;
            }
            if(net_hdr->w!=frameSize.w || net_hdr->h!=frameSize.h){
                printf("\nDisplayfrontal:  Wrong Frame Wrong Recv Size Size\n");
                goto recv_frame_err;
            }

            frame_count++;
            new_frame=true;


        }

        if(br<0){
            printf("\nDisplay Frontal: No OK!\n");fflush(stdout);
        }


recv_frame_err:
        //



        if(new_frame){

            draw_frame_count++;

            if(net_hdr->jpeg_size>0){

                switch(net_hdr->encoder_type){

                case VIDEO_ENCODER_TYPE_RAW:
                    memcpy(data,\
                           &net_pak[sizeof(net_packet_hdr)\
                            +sizeof(net_keyline)*(net_hdr->kline_num)],\
                            net_hdr->jpeg_size);
                    break;

                case VIDEO_ENCODER_TYPE_MFC:
                    decoderMPEG4.DecodeFrame(&net_pak[sizeof(net_packet_hdr)\
                            +sizeof(net_keyline)*net_hdr->kline_num],net_hdr->jpeg_size,data);

                    break;


                case VIDEO_ENCODER_TYPE_MJPEG:
                    decoderMJPEG.DecodeFrame(&net_pak[sizeof(net_packet_hdr)\
                            +sizeof(net_keyline)*net_hdr->kline_num],net_hdr->jpeg_size,data);

                    break;

                default:

                    break;

                }


            }


            net_kl=(net_keyline*)&net_pak[sizeof(net_packet_hdr)];
            net_kln=net_hdr->kline_num;

            pos_tray.push_back(net_hdr->nav.Pos);

            Pose=SO3<>((TooN::Vector<3>)net_hdr->nav.Pose).get_matrix();

            rp.nav=net_hdr->nav;

            d_filler[0].ResetData();
            d_filler[0].FillEdgeData(net_kl,net_kln,{0,0},DF_ThreshRelRho,DF_ThreshMatchNum);
            d_filler[0].InitCoarseFine();
            d_filler[0].Integrate(DF_IterNum);
            d_filler[0].computeDistance(Zeros);


            rp.net_kpn=net_hdr->key_num;





        }else{

            usleep(100);

        }




        if(XLibView>0){

            if(MostrarVideo(&xvc,data,&OnPaint,this)==-1){
                printf("\nDisplayfrontal: error en mostrar video \n");
                break;
            }

        }


        if(XLibView>1){


            if(MostrarVideo(&xvc_d,data_depth,&OnPaintDepth,this)==-1){
                printf("\nDisplayfrontal: error en mostrar video \n");
                break;
            }
        }


        if(XLibView>0){

            k=GetKey(&xvc)|GetKey(&xvc_d);

            switch(k){
            case 'z':
                depth_show_max*=2;
                break;
            case 'x':
                depth_show_max/=2;
                break;
            case 'b':
                pos_tray.clear();
                break;

            case 'i':
                ShowImg=(ShowImg+1)%3;
                break;

            case 't':
                ShowTrails=!ShowTrails;
                break;


            case 'q':
                run=false;
                break;


            default:
                break;

            }
        }

        for(int i=0;i<glv_num;i++)
            run&=glv[i]->glDrawLoop(rp,new_frame);


        if(frame_count>=30){

            gettimeofday(&t_pfps1,NULL);

            pFPS=frame_count/util::dift(t_pfps1,t_pfps0);
            dFPS=draw_frame_count/util::dift(t_pfps1,t_pfps0);
            t_pfps0=t_pfps1;
            frame_count=0;
            draw_frame_count=0;
        }


    }

    delete []net_hdr;
    return 0;

}

visualizer::visualizer(Configurator &config)
{


    carga&=config.GetConfigByName("Visualizer","Host",Host,true);
    carga&=config.GetConfigByName("Visualizer","Port",Port,true);

    carga&=config.GetConfigByName("Visualizer","ImageWidth",ImageSize.w,true);

    carga&=config.GetConfigByName("Visualizer","ImageHeight",ImageSize.h,true);

    carga&=config.GetConfigByName("Visualizer","DepthShow",depth_show_max,true);


    carga&=config.GetConfigByName("Visualizer","TraySource",TraySource,true);

    carga&=config.GetConfigByName("Visualizer","NumberOfViews",ViewNumber,true);

    carga&=config.GetConfigByName("Camera","ZfX",ZfX,true);

    carga&=config.GetConfigByName("Visualizer","EdgeMapSaveNumber",EdgeMapSaveNumber,true);
    carga&=config.GetConfigByName("Visualizer","XLibView",XLibView,true);

    carga&=config.GetConfigByName("DepthFiller","ThreshRelRho",DF_ThreshRelRho,true);
    carga&=config.GetConfigByName("DepthFiller","ThreshMatchNum",DF_ThreshMatchNum,true);
            carga&=config.GetConfigByName("DepthFiller","IterNum",DF_IterNum,true);



}


}





