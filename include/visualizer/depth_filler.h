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


#ifndef DEPTH_FILLER_H
#define DEPTH_FILLER_H

#include "mtracklib/edge_tracker.h"
#include "CommLib/net_keypoint.h"

namespace rebvo {

class df_point{

public:
    bool fixed;

    double depth;

    double rho;
    double s_rho;
    double I_rho;

    Point2DF pi;

    bool intersected;

    bool visibility;

    double dist;

};

class depth_filler
{


    cam_model &cam_mod;
    Size2D im_size;
    Size2D bl_size;
    Size2D size;

    double current_min_dist;

    Image <double> pos_data_d;
    Image <double> pos_data_v;


public:


    Image <df_point> data;

    depth_filler(cam_model &cam,const Size2D &blockSize);

    void ResetData();
    void FillEdgeData(net_keyline *kl, int kn, Point2DF p_off, double v_thresh,int m_num_t);
    void Integrate1Step();
    void Integrate(int iter_num, bool init_cf=true);

    void InitCoarseFine();

    void ComputeColor(TooN::Vector <3> rel_pos);

    const Size2D & imageSize(){
        return im_size;
    }
    const Size2D & blockSize(){
        return bl_size;
    }
    const Size2D & gridSize(){
        return size;
    }

    TooN::Vector <3> Get3DPos(int histo_x,int histo_y,bool transform=true){
        return Get3DPos(histo_x,histo_y,cam_mod,transform);
    }

    TooN::Vector <3> Get3DPos(int histo_x,int histo_y,cam_model &cam,bool transform=true){
        if(histo_x>size.w || histo_y>size.h)
            return TooN::Zeros;
        float x,y;
        cam.Img2Hom<float>(x,y,histo_x*bl_size.w,histo_y*bl_size.h);
        TooN::Vector <3> P0=TooN::makeVector(x/cam.zfm,y/cam.zfm,1)/data(histo_x,histo_y).rho;
        return P0;
    }

    TooN::Vector <3> GetImg3DPos(float x,float y,bool transform=true){
        return GetImg3DPos(x,y,cam_mod,transform);
    }

    TooN::Vector <3> GetImg3DPos(float x,float y,cam_model &cam,bool transform=true){

        float x_histo=x/(float)bl_size.w-0.5;
        int xf=std::min(std::max(floor(x_histo),0.0),size.w-1.0);
        int xc=std::min(std::max(ceil(x_histo),0.0),size.w-1.0);

        float y_histo=y/(float)bl_size.h-0.5;
        int yf=std::min(std::max(floor(y_histo),0.0),size.h-1.0);
        int yc=std::min(std::max(ceil(y_histo),0.0),size.h-1.0);

        float dx=x_histo-xf;
        float dy=y_histo-yf;

        float rho00=data(xf,yf).rho;
        float rho10=data(xc,yf).rho;
        float rho01=data(xf,yc).rho;
        float rho11=data(xc,yc).rho;

        float rho=rho00*(1-dx)*(1-dy)+rho10*dx*(1-dy)+rho01*(1-dx)*dy+rho11*dx*dy;


        cam.Img2Hom<float>(x,y,x,y);
        TooN::Vector <3> P0=TooN::makeVector(x/cam.zfm,y/cam.zfm,1)/rho;

        return P0;
    }

    double GetDist(int x,int y){
        return data[y*size.w+x].dist;
    }


    bool IsImgVisible(float x,float y){

        float x_histo=x/bl_size.w-0.5;
        int xf=std::min(std::max(floor(x_histo),0.0),size.w-1.0);
        int xc=std::min(std::max(ceil(x_histo),0.0),size.w-1.0);

        float y_histo=y/bl_size.h-0.5;
        int yf=std::min(std::max(floor(y_histo),0.0),size.h-1.0);
        int yc=std::min(std::max(ceil(y_histo),0.0),size.h-1.0);


        return data[yf*size.w+xf].visibility || data[yf*size.w+xc].visibility || data[yc*size.w+xf].visibility || data[yc*size.w+xc].visibility;
    }

    double GetImgDist(float x,float y){

        float x_histo=x/bl_size.w-0.5;
        int xf=std::min(std::max(floor(x_histo),0.0),size.w-1.0);
        int xc=std::min(std::max(ceil(x_histo),0.0),size.w-1.0);

        float y_histo=y/bl_size.h-0.5;
        int yf=std::min(std::max(floor(y_histo),0.0),size.h-1.0);
        int yc=std::min(std::max(ceil(y_histo),0.0),size.h-1.0);

        float dx=x_histo-xf;
        float dy=y_histo-yf;

        double d00=data[yf*size.w+xf].dist;
        double d10=data[yf*size.w+xc].dist;
        double d01=data[yc*size.w+xf].dist;
        double d11=data[yc*size.w+xc].dist;

        return d00*(1-dx)*(1-dy)+d10*dx*(1-dy)+d01*(1-dx)*dy+d11*dx*dy;

    }

    double GetMinDist(){return current_min_dist;}
};
}
#endif // DEPTH_FILLER_H
