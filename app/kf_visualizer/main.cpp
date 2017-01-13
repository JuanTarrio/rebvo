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



#ifndef _REENTRANT
#define _REENTRANT
#endif


#include <iostream>
#include "rebvo/rebvo.h"
#include "visualizer/gl_viewer.h"
#include "UtilLib/configurator.h"
#include "mtracklib/keyframe.h"
#include "visualizer/surface_integrator.h"

using namespace rebvo;
using namespace TooN;


struct kfv_params{
    int ViewNumber=1;
    int DF_IterNum;
    u_int DF_BlockSize;
    double DF_ThreshRelRho;
    double DF_ThreshMatchNum;

    bool load(const char *cname){

        Configurator config;
        if(!config.ParseConfigFile(cname,false))
            return false;

        bool carga=config.GetConfigByName("KFVisualizer","NumberOfViews",ViewNumber,true);
        carga&=config.GetConfigByName("DepthFiller","PixelBlockSize",DF_BlockSize,true);
        carga&=config.GetConfigByName("DepthFiller","ThreshRelRho",DF_ThreshRelRho,true);
        carga&=config.GetConfigByName("DepthFiller","ThreshMatchNum",DF_ThreshMatchNum,true);
        carga&=config.GetConfigByName("DepthFiller","IterNum",DF_IterNum,true);
        return carga;
    }
};

int main(int argn,char ** argv)
{


    if(argn!=3){
        std::cout <<"Use kf_visualizer KeyFrameFile ConfigFile\n";
        return -1;
    }





    kfv_params param;

    if(!param.load(argv[2]))
        return -1;

    std::vector<keyframe> kf_list;

    if(!keyframe::loadKeyframesFromFile(argv[1],kf_list)){
        std::cout<<"\nError loading KFFile\n";
        return -1;
    }

    if(kf_list.size()==0){
        std::cout<<"\nKFList empty\n";
        return -1;
    }

    std::cout<<"\nLoaded "<<kf_list.size()<<"Keyframes\n";

    cam_model cam=kf_list[0].camera;

    Matrix<3,3> Pose=Identity;
    std::vector<Vector<3>> pos;
    pos.push_back(Zeros);

    std::vector<bool> kfshow;

    for(int i=0;i<kf_list.size();i++){
        kfshow.push_back(false);
        kf_list[i].initDepthFiller({param.DF_BlockSize,param.DF_BlockSize},param.DF_IterNum,param.DF_ThreshRelRho,param.DF_ThreshMatchNum);
    }

    TooN::Vector <3> oGridSize,oGridOrig;
    oGridSize=SurfaceInt::analizeSpaceSize(kf_list,&oGridOrig);
    std::cout << oGridSize<<" "<<oGridOrig<<"\n";

    OcGrid ocgrid(oGridOrig,oGridSize,{500,500,500});

    std::cout <<"Start filling...\n";
    std::cout <<"Finish: "<< ocgrid.fillKFList(kf_list) <<" blocks filled\n";

    RenderParams rp;

    rp.net_kl=nullptr;
    rp.net_kln=0;
    rp.net_klistn=0;
    rp.zf=cam.zfm;
    rp.pp=cam.pp;
    rp.net_kpn=0;
    rp.ref_err=Zeros;
    rp.d_filler=nullptr;
    rp.pos_tray=&pos;
    rp.Pose=&Pose;
    rp.current_em=0;
    rp.total_em=0;
    rp.ref_err=Zeros;

    rp.img_data=new Image<RGB24Pixel>(cam.sz);
    rp.kflist=&kf_list;
    rp.kf_show_mask=&kfshow;
    rp.render_match=0;


    gl_viewer * glv[param.ViewNumber];

    for(int i=0;i<param.ViewNumber;i++){
        glv[i]=new gl_viewer(cam.sz.w,cam.sz.h,"GL View",atan((double)cam.sz.h/2.0/cam.zfm)*360.0/M_PI);
        glv[i]->ToggleFixView(2);
        glv[i]->ToggleCameraView(0);
    }

    bool new_frame=false;
    bool run=true;



    int curr_kf=0;
    int num_kf=1;

    while(run){


        for(int i=0;i<param.ViewNumber;i++){
            KeySym k=0;
            run&=glv[i]->glDrawLoop(rp,new_frame,&k);

            new_frame=false;

            switch(k){
            case XK_o:
                curr_kf=(curr_kf-1+kf_list.size())%kf_list.size();
                new_frame=true;
                break;
            case XK_p:
                curr_kf=(curr_kf+1)%kf_list.size();
                new_frame=true;
                break;
            case XK_ntilde:
                util::keep_min(++num_kf,kf_list.size());
                std::cout <<"Showing "<<num_kf<<" Keylines\n";
                new_frame=true;
                break;
            case XK_l:
                util::keep_max(--num_kf,0);
                std::cout <<"Showing "<<num_kf<<" Keylines\n";
                new_frame=true;
                break;
            case XK_k:
                rp.render_match=!rp.render_match;
                new_frame=true;
                break;

            case XK_plus:

                std::cout <<"Start ray cut...";
                ocgrid.rayCutSurface(kf_list);
                std::cout <<"Finished\n";

                new_frame=true;
                break;

            case XK_apostrophe:

                std::cout <<"Start ray cut...";
                ocgrid.rayCutSurface(kf_list[curr_kf]);
                std::cout <<"Finished\n";

                new_frame=true;
                break;

            case XK_minus:

                for(keyframe &kf:kf_list){
                    if(kf.depthFillerAval())
                        kf.depthFill().ResetVisibility();

                }
                new_frame=true;
                break;


            }

            if(new_frame){

                for(int i=0;i<kf_list.size();i++)
                    kfshow[i]=false;
                for(int i=0;i<num_kf;i++)
                    kfshow[(curr_kf-i+kf_list.size())%kf_list.size()]=true;

                pos.clear();
                pos.push_back(-kf_list[curr_kf].Pose.T()*kf_list[(curr_kf)].Pos);
                Pose=kf_list[curr_kf].Pose.T();
            }
        }
        usleep(10e3);

    }




    return 0;
}
