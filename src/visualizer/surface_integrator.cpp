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
#include "visualizer/surface_integrator.h"

using namespace TooN;
namespace rebvo {
namespace SurfaceInt{

//analizeSpaceSize(): search on the kf_list depth_fillers
//maximun and minimun points for space size determination

TooN::Vector<3> analizeSpaceSize(std::vector<keyframe> &kf_list, //List of keframes
                                 TooN::Vector<3> *origin)       //Return origin point (minimun pos)
{

    double min_x=1e+20,max_x=1e-20;
    double min_y=1e+20,max_y=1e-20;
    double min_z=1e+20,max_z=1e-20;

    for(keyframe &kf:kf_list){

        if(kf.depthFillerAval()){
            depth_filler &df=kf.depthFill();

            for(int df_y=0;df_y<df.gridSize().h;df_y++)
                for(int df_x=0;df_x<df.gridSize().w;df_x++){

                    TooN::Vector<3> P=kf.Local2WorldScaled(df.get3DPos(df_x,df_y));

                    util::keep_min(min_x,P[0]);
                    util::keep_min(min_y,P[1]);
                    util::keep_min(min_z,P[2]);
                    util::keep_max(max_x,P[0]);
                    util::keep_max(max_y,P[1]);
                    util::keep_max(max_z,P[2]);


                }
        }

    }

    if(origin)
        *origin=makeVector(min_x,min_y,min_z);

    return TooN::makeVector(max_x-min_x,max_y-min_y,max_z-min_z);

}

void checkDFRayCrossExaustive(keyframe &target,keyframe &hidder){


    if(!target.depthFillerAval() || !hidder.depthFillerAval())
        return;

    depth_filler &t_df=target.depthFill();
    depth_filler &h_df=hidder.depthFill();

    double norm_size=util::norm(t_df.blockSize().w,t_df.blockSize().h)/target.camera.zfm*target.K;

    for(int t_y=0;t_y<t_df.gridSize().h;t_y++)
        for(int t_x=0;t_x<t_df.gridSize().w;t_x++){

            bool crossed=false;

            for(int h_y=0;h_y<h_df.gridSize().h && !crossed;h_y++)
                for(int h_x=0;h_x<h_df.gridSize().w && !crossed;h_x++){

                    Vector <3> ray_orig=hidder.transformTo(target,Zeros);
                    Vector <3> ray_pass=hidder.transformTo(target,h_df.get3DPos(h_x,h_y)*hidder.K);
                    Vector <3> ray_versor=unit((ray_pass-ray_orig));

                    Vector <3> point=t_df.get3DPos(t_x,t_y)*target.K;

                    double buble_size=norm_size/t_df.data(t_x,t_y).rho;


                    double dist_p=norm((point-ray_orig)^ray_versor);

                    if(dist_p<buble_size){
                        double dist_orig=((point-ray_orig)*ray_versor);
                        if(dist_orig>0 && dist_orig<(h_df.data(h_x,h_y).dist))

                            crossed=true;
                    }

                }

            if(crossed)
                t_df.data(t_x,t_y).visibility=false;


        }


}

}

OcGrid::OcGrid(TooN::Vector<3> orig_point,TooN::Vector<3> space_size,Point3D<u_int> grid_size)
    :size(grid_size),wordl_size(space_size),
      lin_size(grid_size.x*grid_size.y*grid_size.z),origin(orig_point),
      block_size(TooN::makeVector(space_size[0]/size.x,space_size[1]/size.y,space_size[2]/size.z)),
      min_block_size(TooN::min_value(block_size)),
      data(new OGridPoint*[lin_size])
{

    for(int i=0;i<lin_size;i++)
        data[i]=nullptr;


}

OcGrid::~OcGrid(){
    delete [] data;
}

//Adds a depth filler point to an OcGrid point list
bool OcGrid::addDFPoint(OGridPoint *&pos,df_point *point)
{
    if(!pos)
        pos=new OGridPoint; //If OcGrid empty, create...

    for(df_point* &df_ptr: pos->surf_p){    //Check in the OGridPoint if the point is already added
        if(df_ptr==point)
            return false;
    }
    pos->surf_p.push_back(point);   //If not.. add
    return true;
}


void OcGrid::hideAll(OGridPoint *&pos,df_point *point)
{
    if(!pos)
        return;

    for(df_point* &df_ptr: pos->surf_p){
        if(df_ptr->father!=point->father )//&& !df_ptr->fixed)
            df_ptr->visibility=false;
    }
    return;
}

//Fill OcGrid with references to the surface in the passing blocks

u_int OcGrid::fillKFList(std::vector<keyframe> &kf_list)
{

    u_int counter=0;
    for(keyframe &kf:kf_list){

        if(!kf.depthFillerAval())
            continue;
        depth_filler &df=kf.depthFill();

        for(u_int g_y=0;g_y<df.gridSize().h;g_y++)
            for(u_int g_x=0;g_x<df.gridSize().w;g_x++){ //For each point in the Depth Filler grid


                //double min_rho=std::min(std::min(df.data(g_x,g_y).rho,df.data(g_x+1,g_y).rho),std::min(df.data(g_x,g_y+1).rho,df.data(g_x+1,g_y+1).rho));

                double min_rho=df.data(g_x,g_y).rho/kf.K;
                double rect_size_x=df.blockSize().w/kf.camera.zfm/min_rho;  //Determine the size of the projected rectangle on the current grid
                double rect_size_y=df.blockSize().h/kf.camera.zfm/min_rho;

                double step_num_x=std::max(rect_size_x/min_block_size,1.0); //Determine the step size num
                double step_num_y=std::max(rect_size_y/min_block_size,1.0);


                if(step_num_x>100 || step_num_y >100){
                    std::cout<<step_num_x<<" "<<step_num_y<<"\n";
                }

                double step_size_x=df.blockSize().w/step_num_x;
                double step_size_y=df.blockSize().h/step_num_y;


                if(step_size_x==0||step_size_y==0)
                    std::cout<<min_rho<<" "<<min_block_size<<"\n";

                for(float i_y=g_y*df.blockSize().h;i_y<(g_y+1)*df.blockSize().h;i_y+=step_size_y)
                    for(float i_x=g_x*df.blockSize().w;i_x<(g_x+1)*df.blockSize().w;i_x+=step_size_x){
                        TooN::Vector <3> p=kf.Local2WorldScaled(df.getImg3DPos(i_x,i_y));   //Surface point in 3D space

                        TooN::Vector <3> po=p-origin;
                        if(po[0]>wordl_size[0]  || po[1]>wordl_size[1]  ||po[2]>wordl_size[2]  ){
                            std::cout <<"\nOcGrid, Out of size!\n";
                            continue;
                        }

                        if(wordl2Index(p)>lin_size){
                            std::cout <<"\nOcGrid, Out of lin size!"<<g_x<<" "<<g_y<<" "<<i_x<<" "<<i_y<<" "<<p-origin<<" "<<min_rho<<"\n";
                            continue;
                        }



                        if(addDFPoint(data[wordl2Index(p)],&df.data(g_x,g_y)))  //addpoint
                            counter++;

                    }

            }
    }

    return counter;

}


//Seach along ray direction hiding each surface that encounters


u_int OcGrid::rayCutSurface(keyframe &kf)
{
    if(!kf.depthFillerAval())
        return 0;
    depth_filler &df=kf.depthFill();

    for(u_int g_y=0;g_y<df.gridSize().h;g_y++)
        for(u_int g_x=0;g_x<df.gridSize().w;g_x++){

            Vector <3> ray_orig=kf.Local2WorldScaled(Zeros);
            Vector <3> ray_pass=kf.Local2WorldScaled(df.get3DPosShiftRho(g_x,g_y,df.data(g_x,g_y).s_rho));
            Vector <3> ray_step=unit((ray_pass-ray_orig))*min_block_size;
            int step_num=norm(ray_pass-ray_orig)/min_block_size;

            Vector <3> pos=ray_orig;
            for(int i=0;i<step_num;i++,pos+=ray_step){
                hideAll(data[wordl2Index(pos)],&df.data(g_x,g_y));
            }

        }

    return 0;
}

u_int OcGrid::rayCutSurface(std::vector<keyframe> &kf_list)
{
    u_int counter=0;
    for(keyframe &kf:kf_list){
        rayCutSurface(kf);
    }
    return counter;
}


}
