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


#include "visualizer/depth_filler.h"
#include <TooN/so3.h>

namespace rebvo {
depth_filler::depth_filler(cam_model &cam,const Size2D &blockSize)
    :cam_mod(cam),im_size(cam.sz),bl_size(blockSize),size{im_size.w/bl_size.w,im_size.h/bl_size.h},
      current_min_dist(1e20),pos_data_d(size),pos_data_v(size),data(size)
{



}



void depth_filler::ResetData(){

    for(int i=0;i<size.w*size.h;i++){
        data[i].fixed=false;

        data[i].depth=1;
        data[i].v_depth=1e6;
        data[i].rho=1;
        data[i].v_rho=1e6;

        data[i].visibility=true;
    }

}

void depth_filler::FillEdgeData(net_keyline *kl,int kn,Point2DF p_off,double v_thresh){


    for (int ikl=0;ikl<kn;ikl++){


        double rho=kl[ikl].rho/NET_RHO_SCALING;
        double s_rho=kl[ikl].s_rho/NET_RHO_SCALING;


        if(1/(s_rho*s_rho)<20)
            continue;

  //      if(s_depth*rho>v_thresh)
  //          continue;

     //   if(s_rho/rho>v_thresh)
     //       continue;

        Point2DF p;
        p.x=(kl[ikl].qx+p_off.x)/bl_size.w;
        p.y=(kl[ikl].qy+p_off.y)/bl_size.h;
        int inx=((int)p.y)*size.w+(int)p.x;

        data[inx].pi.x=kl[ikl].qx+p_off.x;
        data[inx].pi.y=kl[ikl].qy+p_off.y;


        data[inx].rho=rho;
        data[inx].s_rho=s_rho;

        data[inx].fixed=true;





    }


}



void depth_filler::ComputeColor(TooN::Vector<3> rel_pos){

    current_min_dist=1e20;
    for(int x=0;x<size.w;x++)
        for(int y=0;y<size.h;y++){
            TooN::Vector <3> P=Get3DPos(x,y)-rel_pos;

            data[y*size.w+x].dist=TooN::norm(P);

            util::keep_min(current_min_dist,data[y*size.w+x].dist);

        }
}


void depth_filler::Integrate(int iter_num,bool init_cf){

    if(init_cf){
        InitCoarseFine();
    }else{

        for(int inx=0;inx<size.w*size.h;inx++){
            if(data[inx].fixed)
                continue;
            data[inx].rho=1;
            data[inx].s_rho=1e3;
        }
    }


    for(int iter=0;iter<iter_num;iter++){

        Integrate1StepRho();

    }

    for(int inx=0;inx<size.w*size.h;inx++){
        data[inx].v_rho=data[inx].s_rho*data[inx].s_rho;
        double s_depth=data[inx].v_rho/(data[inx].rho*sqrt(3*(data[inx].rho*data[inx].rho+data[inx].v_rho*data[inx].v_rho)));
        data[inx].depth=1/data[inx].rho;
        data[inx].v_depth=s_depth*s_depth;


    }

}

void depth_filler::InitCoarseFine(){


    for(int size_x=size.w,size_y=size.h;size_x>1 && size_y>1;size_x/=2,size_y/=2){

        for(int x=0;x<=size.w-size_x;x+=size_x)
            for(int y=0;y<=size.h-size_y;y+=size_y){

                double mean_rho=0;
                int n=0;

                for(int dx=0;dx<size_x;dx++)
                    for(int dy=0;dy<size_y;dy++){
                        int inx=(y+dy)*size.w+x+dx;
                        if(data[inx].fixed){
                            mean_rho+=data[inx].rho;
                            n++;
                        }
                    }
                if(n>0){
                    mean_rho/=n;
                    for(int dx=0;dx<size_x;dx++)
                        for(int dy=0;dy<size_y;dy++){
                            int inx=(y+dy)*size.w+x+dx;
                            if(!data[inx].fixed){
                                data[inx].rho=mean_rho;
                            }
                        }
                }
            }

    }


}



void depth_filler::Integrate1Step(){

    for(int y=0;y<size.h;y++){
        for(int x=0;x<size.w;x++){

            double z=0;
            int n=0;

            int inx=y*size.w+x;

            if(data[inx].fixed){
                pos_data_d[inx]=(data[inx]).depth;
                pos_data_v[inx]=data[inx].v_depth;
                continue;
            }

            for(int dy=-1;dy<=1;dy++){
                for(int dx=-1;dx<=1;dx++){

                    if(dx==0&&dy==0)
                        continue;

                    int p_x=x+dx;
                    int p_y=y+dy;

                    if(p_x<0 || p_x>=size.w || p_y<0 || p_y >= size.h)
                        continue;

                    z+=data[p_y*size.w+p_x].depth;
                    n++;

                }
            }

            pos_data_d[inx]=data[inx].depth=z/n;
            pos_data_v[inx]=1e6;
        }
    }


    for(int inx=0;inx<size.w*size.h;inx++){
        data[inx].depth=pos_data_d[inx];
        data[inx].v_depth=pos_data_v[inx];
    }
}


void depth_filler::Integrate1StepRho(){

    double w=1.8;

    for(int y=0;y<size.h;y++){
        for(int x=0;x<size.w;x++){

            int inx=y*size.w+x;

            if(data[inx].fixed){
                //pos_data_d[inx]=data[inx].rho;
                //pos_data_v[inx]=data[inx].s_rho;

            }else{

                double r=0,sr=0,isr=0;
                int n=0;


                for(int dy=-1;dy<=1;dy++){
                    for(int dx=-1;dx<=1;dx++){

                        if(dx==0&&dy==0)
                            continue;

                        int p_x=x+dx;
                        int p_y=y+dy;

                        if(p_x<0 || p_x>=size.w || p_y<0 || p_y >= size.h)
                            continue;

                        r+=data[p_y*size.w+p_x].rho;//data[p_y*s.w+p_x].s_rho;
                        sr+=data[p_y*size.w+p_x].s_rho;
                        isr+=1/data[p_y*size.w+p_x].s_rho;
                        n++;

                    }
                }

                data[inx].rho=(1-w)*data[inx].rho+w*r/n;
                data[inx].s_rho=sr/n;

                if(data[inx].rho){
                    data[inx].rho=r/n;
                    //printf("r%f %f ",data[inx].rho,r/n);
                }

            }
        }
    }



}

}
