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


depth_filler::depth_filler(cam_model &cam,const Size2D &blockSize,const bound_modes &bound_mode)
    :cam_mod(cam),im_size(cam.sz),bl_size(blockSize),size{im_size.w/bl_size.w,im_size.h/bl_size.h},
      current_min_dist(1e20),pos_data_d(size),pos_data_v(size),imgc(nullptr),bound_m(bound_mode),data(size)
{

    ResetData();

}



void depth_filler::ResetData(){

    for(int i=0;i<size.w*size.h;i++){
        data[i].fixed=false;

        data[i].depth=1;
        data[i].rho=1;
        data[i].I_rho=1.0/util::square(RHO_MAX*2);
        data[i].s_rho=RHO_MAX*2;
        data[i].visibility=true;
        data[i].father=this;
    }

}



//Fill edge data using net_keyline structure
void depth_filler::FillEdgeData(net_keyline *kl, int kn, Point2DF p_off, double v_thresh, int m_num_t,bool discart){


    for (int ikl=0;ikl<kn;ikl++){

        net_keyline &nkl=kl[ikl];

        double kl_rho=nkl.rho/NET_RHO_SCALING;
        double kl_s_rho=nkl.s_rho/NET_RHO_SCALING;


        if(kl_s_rho/kl_rho>v_thresh)
            continue;

        if(nkl.m_num<m_num_t){
            if(discart)
                continue;
            else
                kl_s_rho=RHO_MAX;
        }

  //      if(s_depth*rho>v_thresh)
  //          continue;

     //   if(s_rho/rho>v_thresh)
     //       continue;

        int inx=data.GetIndex((nkl.qx+p_off.x)/bl_size.w,(nkl.qy+p_off.y)/bl_size.h);



        double i_rho=data[inx].I_rho*data[inx].rho;
        double kl_I_rho=1/(kl_s_rho*kl_s_rho);
        i_rho+=kl_rho*kl_I_rho;
        data[inx].I_rho+=kl_I_rho;
        double v_rho=data[inx].I_rho>0?1.0/data[inx].I_rho:1e20;
        data[inx].rho=i_rho*v_rho;
        data[inx].s_rho=sqrt(v_rho);

        data[inx].fixed=true;


    }


}

//fill edge data directly from edge tracker list
/**
 * @brief depth_filler::FillEdgeData
 * @param et keyline list
 * @param v_thresh threshold on relative error
 * @param m_num_t threshold on matched num
 */
void depth_filler::FillEdgeData(edge_tracker&et,
                                double v_thresh,
                                int m_num_t,
                                bool discart)
{


    for ( KeyLine &kl:et){


        if(kl.s_rho/kl.rho>v_thresh )
            continue;




        int inx=data.GetIndex((kl.c_p.x)/bl_size.w,(kl.c_p.y)/bl_size.h);


        double i_rho=data[inx].I_rho*data[inx].rho;     //block information vector

        double kl_I_rho=1/(kl.s_rho*kl.s_rho);          //KL information matrix

        if(kl.m_num<m_num_t){
            if(discart)
                continue;
            else
                kl_I_rho=1.0/(RHO_MAX*RHO_MAX);
        }

        i_rho+=kl.rho*kl_I_rho;                         //Inforamtion update
        data[inx].I_rho+=kl_I_rho;

        double v_rho=data[inx].I_rho>0?1.0/data[inx].I_rho:1e20;
        data[inx].rho=i_rho*v_rho;
        data[inx].s_rho=sqrt(v_rho);

        data[inx].fixed=true;


    }


}



/**
 * @brief depth_filler::ComputeColor
 * @param rel_pos
 */

void depth_filler::computeDistance(TooN::Vector<3> rel_pos){

    current_min_dist=1e20;
    for(int x=0;x<size.w;x++)
        for(int y=0;y<size.h;y++){
            TooN::Vector <3> P=get3DPos(x,y)-rel_pos;

            data[y*size.w+x].dist=TooN::norm(P);

            util::keep_min(current_min_dist,data[y*size.w+x].dist);

        }
}

/**
 * @brief depth_filler::ResetVisibility reset visibility on every block

*/


void depth_filler::ResetVisibility()
{
    for(int i=0;i<size.w*size.h;i++){
        data[i].visibility=true;
    }
}

/**
 * @brief depth_filler::Integrate
 * @param iter_num
 * @param init_cf
 */

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

        Integrate1Step();

    }

    for(int inx=0;inx<size.w*size.h;inx++){
        data[inx].I_rho=1/(data[inx].s_rho*data[inx].s_rho);
        data[inx].depth=1/data[inx].rho;


    }

}

void depth_filler::InitCoarseFine(){


    for(int size_x=size.w,size_y=size.h;size_x>1 && size_y>1;size_x/=2,size_y/=2){

        for(int x=0;x<=size.w-size_x;x+=size_x)
            for(int y=0;y<=size.h-size_y;y+=size_y){

                double mean_rho=0;
                double mean_srho=0;
                int n=0,nr=0;

                for(int dx=0;dx<size_x;dx++)
                    for(int dy=0;dy<size_y;dy++){
                        int inx=(y+dy)*size.w+x+dx;
                        if(data[inx].fixed){
                            mean_rho+=data[inx].rho;
                            mean_srho+=data[inx].s_rho;
                            //util::keep_max(mean_srho,data[inx].s_rho);
                            n++;
                            nr++;
                        }else if(inboundary(x+dx,y+dy)){
                            //util::keep_max(mean_srho,data[inx].s_rho);
                            mean_srho+=data[inx].s_rho;
                            nr++;
                        }
                    }
                if(n>0){
                    mean_rho/=n;
                    mean_srho/=nr;
                    for(int dx=0;dx<size_x;dx++)
                        for(int dy=0;dy<size_y;dy++){
                            int inx=(y+dy)*size.w+x+dx;
                            if(!data[inx].fixed){
                                data[inx].rho=mean_rho;
                                if(!inboundary(x+dx,y+dy))
                                    data[inx].s_rho=mean_srho;
                            }
                        }
                }
            }

    }


}

bool depth_filler::inboundary(int x,int y){

    switch(bound_m){
    case BOUND_NONE:
        return false;
    case BOUND_CORNERS:
        return (x==0 && (y==0 || y==size.h-1)) || (x==size.w-1 && (y==0 || y==size.h-1)) ;
    case BOUND_FULL:
        return x==0 || x==size.w-1 || y==0 || y==size.h-1;
    }


    return false;
}


/**
 * @brief depth_filler::Integrate1Step
 *
 */

void depth_filler::Integrate1Step(){

    double w=1;//1.8;

    for(int y=0;y<size.h;y++){
        for(int x=0;x<size.w;x++){

            int inx=data.GetIndex(x,y);

            if(data[inx].fixed){
                //pos_data_d[inx]=data[inx].rho;
                //pos_data_v[inx]=data[inx].s_rho;

            }else{

                double r=0,sr=0;
                int n=0;


                for(int dy=-1;dy<=1;dy++){
                    for(int dx=-1;dx<=1;dx++){

                        if(dx==0&&dy==0)
                            continue;

                        int p_x=x+dx;
                        int p_y=y+dy;

                        if(p_x<0 || p_x>=size.w || p_y<0 || p_y >= size.h)
                            continue;

                        r+=data(p_x,p_y).rho;//data[p_y*s.w+p_x].s_rho;
                        sr+=data(p_x,p_y).s_rho;
                        n++;

                    }
                }

                data[inx].rho=(1-w)*data[inx].rho+w*r/n;
                if(!inboundary(x,y))
                    data[inx].s_rho=sr/n;



            }
        }
    }



}


void depth_filler::calcSurfNormals(){

    for(int x=0;x<size.w-1;x++)
        for(int y=0;y<size.h-1;y++){

            TooN::Vector <3> P00=get3DPos(x,y);
            TooN::Vector <3> P01=get3DPos(x+1,y);
            TooN::Vector <3> P10=get3DPos(x,y+1);
            TooN::Vector <3> P11=get3DPos(x+1,y+1);


            data(x,y).normal=(-TooN::unit((P01-P00)^(P10-P00))+TooN::unit((P01-P11)^(P10-P11)))/2;
            data(x+1,y+1).normal=data(x,y).normal;
        }

}

void depth_filler::calcSurfArea(){

    for(int x=0;x<size.w-1;x++)
        for(int y=0;y<size.h-1;y++){

            TooN::Vector <3> P00=get3DPos(x,y);
            TooN::Vector <3> P01=get3DPos(x+1,y);
            TooN::Vector <3> P10=get3DPos(x,y+1);
            TooN::Vector <3> P11=get3DPos(x+1,y+1);

            data(x,y).area=(TooN::norm((P01-P00)^(P10-P00))+TooN::norm((P01-P11)^(P10-P11)))/2;
        }

}

Image<RGB24Pixel> *depth_filler::getImgc() const
{
    return imgc;
}

void depth_filler::setImgc(Image<RGB24Pixel> *value)
{
    imgc = value;
}

}
