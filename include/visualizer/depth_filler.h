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

class keyframe;

class depth_filler;

struct df_point{

    bool fixed;             //Flag to determine if there are KeyLines passing through
    double depth;           //Estimated depth
    double rho;             //Estimated Inv-Depth
    double s_rho;           //Estimated Inv-Depth Uncertainty

    double rho0;
    double s_rho0;

    double I_rho;           //Information on Inverse Depth 1/s_rho^2
    bool visibility;        //Visibility on the plot flag
    double dist;            //Euclidean distance from the point to center of the camera
    depth_filler* father;   //pointer to the depth_filler calss
    TooN::Vector <3> normal;
    float area;
    bool visil_opt;
    TooN::Vector <3> optim_point;

};

class depth_filler
{
public:

    enum bound_modes{BOUND_NONE, BOUND_CORNERS,BOUND_FULL} ;
private:
    cam_model &cam_mod; //Reference to cam model
    Size2D im_size;     //Image size
    Size2D bl_size;     //Block (discretization) size
    Size2D size;        //Grid Size=im_size/bl_size

    double current_min_dist;

    Image <double> pos_data_d;
    Image <double> pos_data_v;
    Image <RGB24Pixel> *imgc;

    bound_modes bound_m;

public:

    keyframe *kf_ptr=nullptr;

    Image <df_point> data;

    depth_filler(cam_model &cam, const Size2D &blockSize, const bound_modes &bound_mode);

    void ResetData();
    void FillEdgeData(net_keyline *kl, int kn, Point2DF p_off, double v_thresh, int m_num_t, bool discart=true);
    void FillEdgeData(edge_tracker&et, double v_thresh, int m_num_t, bool discart=true);
    void Integrate1Step();
    void Integrate(int iter_num, bool init_cf=true);

    void InitCoarseFine();

    void computeDistance(TooN::Vector <3> rel_pos);

    void ResetVisibility();

    const Size2D & imageSize(){
        return im_size;
    }
    const Size2D & blockSize(){
        return bl_size;
    }
    const Size2D & gridSize(){
        return size;
    }

    TooN::Vector <3> get3DPos(int histo_x,int histo_y){
        return get3DPos(histo_x,histo_y,cam_mod);
    }

    TooN::Vector <3> get3DPosShiftRho(int histo_x,int histo_y,double s_rho){
        return get3DPosShiftRho(histo_x,histo_y,s_rho,cam_mod);
    }

    TooN::Vector <3> get3DPos(int histo_x,int histo_y,cam_model &cam){
        if(histo_x>size.w || histo_y>size.h)
            return TooN::Zeros;
        float x,y;
        cam.Img2Hom<float>(x,y,((float)histo_x+0.5)*bl_size.w,((float)histo_y+0.5)*bl_size.h);    //shift point by 0.5*bl_size and substract principal point
        TooN::Vector <3> P0=TooN::makeVector(x/cam.zfm,y/cam.zfm,1)/data(histo_x,histo_y).rho;
        return P0;
    }

    TooN::Vector <3> get3DPosShiftRho(int histo_x,int histo_y,double s_rho,cam_model &cam){
        if(histo_x>size.w || histo_y>size.h)
            return TooN::Zeros;
        float x,y;
        cam.Img2Hom<float>(x,y,((float)histo_x+0.5)*bl_size.w,((float)histo_y+0.5)*bl_size.h);    //shift point by 0.5*bl_size and substract principal point
        TooN::Vector <3> P0=TooN::makeVector(x/cam.zfm,y/cam.zfm,1)/(data(histo_x,histo_y).rho+s_rho);
        return P0;
    }

    TooN::Vector <3> getImg3DPos(float x,float y){
        return getImg3DPos(x,y,cam_mod);
    }

    //Converts image position coordinates to a 3D point in space using bilin interpolation
    TooN::Vector <3> getImg3DPos(float x,float y,cam_model &cam){

        float x_histo=x/(float)bl_size.w-0.5;   //Points in grid are shifted 0.5*bl_size due to discretization effect
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

    //Converts image position coordinates to a 3D point in space using bilin interpolation
    TooN::Vector <3> getImg3DPosTriInterp(float x,float y,cam_model &cam){

        float x_histo=x/(float)bl_size.w-0.5;   //Points in grid are shifted 0.5*bl_size due to discretization effect
        int xf=std::min(std::max(floor(x_histo),0.0),size.w-1.0);
        int xc=std::min(std::max(ceil(x_histo),0.0),size.w-1.0);

        float y_histo=y/(float)bl_size.h-0.5;
        int yf=std::min(std::max(floor(y_histo),0.0),size.h-1.0);
        int yc=std::min(std::max(ceil(y_histo),0.0),size.h-1.0);

        float dx=x_histo-xf;
        float dy=y_histo-yf;

        float rho00=data(xf,yf).rho; //(0,0)
        float rho10=data(xc,yf).rho; //(1,0)
        float rho01=data(xf,yc).rho; //(0,1)
        float rho11=data(xc,yc).rho; //(1,1)

        float rho;
        if(dx>dy)
            rho=rho00+dx*(rho10-rho00)+dy*(rho11-rho10);
        else
            rho=rho00+dy*(rho01-rho00)+dx*(rho11-rho01);


        cam.Img2Hom<float>(x,y,x,y);
        TooN::Vector <3> P0=TooN::makeVector(x/cam.zfm,y/cam.zfm,1)/rho;

        return P0;
    }

    df_point& getImgPoint(int x,int y){
        int x_histo=util::Constrain<int>(x/bl_size.w,0,size.w-1);   //Points in grid are shifted 0.5*bl_size due to discretization effec
        int y_histo=util::Constrain<int>(y/bl_size.h,0,size.h-1);
        return data(x_histo,y_histo);
    }

    double getImgRhoTriInterp(float x,float y,double *s_rho=nullptr){

        float x_histo=x/(float)bl_size.w-0.5;
        int xf=std::min(std::max(floor(x_histo),0.0),size.w-1.0);
        int xc=std::min(std::max(ceil(x_histo),0.0),size.w-1.0);

        float y_histo=y/(float)bl_size.h-0.5;
        int yf=std::min(std::max(floor(y_histo),0.0),size.h-1.0);
        int yc=std::min(std::max(ceil(y_histo),0.0),size.h-1.0);

        float dx=x_histo-xf;
        float dy=y_histo-yf;


        float rho00=data(xf,yf).rho; //(0,0)
        float rho10=data(xc,yf).rho; //(1,0)
        float rho01=data(xf,yc).rho; //(0,1)
        float rho11=data(xc,yc).rho; //(1,1)

        float rho;
        if(dx>dy)
            rho=rho00+dx*(rho10-rho00)+dy*(rho11-rho10);
        else
            rho=rho00+dy*(rho01-rho00)+dx*(rho11-rho01);

        if(s_rho){

            float srho00=data(xf,yf).s_rho;
            float srho10=data(xc,yf).s_rho;
            float srho01=data(xf,yc).s_rho;
            float srho11=data(xc,yc).s_rho;


            if(dx>dy)
                (*s_rho)=srho00+dx*(srho10-srho11)+dy*(srho11-srho10);
            else
                (*s_rho)=srho00+dy*(srho01-srho11)+dx*(srho11-srho01);

        }

        return rho;
    }

    double getImgRho(float x,float y,double *s_rho=nullptr,bool *fixed=nullptr){

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

        if(s_rho){

            float srho00=data(xf,yf).s_rho;
            float srho10=data(xc,yf).s_rho;
            float srho01=data(xf,yc).s_rho;
            float srho11=data(xc,yc).s_rho;

            (*s_rho)=srho00*(1-dx)*(1-dy)+srho10*dx*(1-dy)+srho01*(1-dx)*dy+srho11*dx*dy;
        }

        if(fixed)
            *fixed=data(xf,yf).fixed;

        return rho;
    }

    double interplShiftRhoScaled(float x,float y,double scale,double mult){

        float x_histo=x/(float)bl_size.w-0.5;
        int xf=std::min(std::max(floor(x_histo),0.0),size.w-1.0);
        int xc=std::min(std::max(ceil(x_histo),0.0),size.w-1.0);

        float y_histo=y/(float)bl_size.h-0.5;
        int yf=std::min(std::max(floor(y_histo),0.0),size.h-1.0);
        int yc=std::min(std::max(ceil(y_histo),0.0),size.h-1.0);

        float dx=x_histo-xf;
        float dy=y_histo-yf;

        float rho00=util::Constrain(data(xf,yf).rho+mult*data(xf,yf).s_rho,RHO_MIN,RHO_MAX);
        float rho10=util::Constrain(data(xc,yf).rho+mult*data(xc,yf).s_rho,RHO_MIN,RHO_MAX);
        float rho01=util::Constrain(data(xf,yc).rho+mult*data(xf,yc).s_rho,RHO_MIN,RHO_MAX);
        float rho11=util::Constrain(data(xc,yc).rho+mult*data(xc,yc).s_rho,RHO_MIN,RHO_MAX);

        float rho=rho00*(1-dx)*(1-dy)+rho10*dx*(1-dy)+rho01*(1-dx)*dy+rho11*dx*dy;

        return rho/scale;
    }

    void calcSurfNormals();



    double GetDist(int x,int y){
        return data[y*size.w+x].dist;
    }


    bool IsImgVisible(float x,float y,bool use_and=true){

        float x_histo=x/bl_size.w-0.5;
        int xf=std::min(std::max(floor(x_histo),0.0),size.w-1.0);
        int xc=std::min(std::max(ceil(x_histo),0.0),size.w-1.0);

        float y_histo=y/bl_size.h-0.5;
        int yf=std::min(std::max(floor(y_histo),0.0),size.h-1.0);
        int yc=std::min(std::max(ceil(y_histo),0.0),size.h-1.0);


        if(use_and)
            return data[yf*size.w+xf].visibility && data[yf*size.w+xc].visibility && data[yc*size.w+xf].visibility && data[yc*size.w+xc].visibility;

        return data[yf*size.w+xf].visibility || data[yf*size.w+xc].visibility || data[yc*size.w+xf].visibility || data[yc*size.w+xc].visibility;
    }

    bool IsImgVisibleSimple(float x,float y){

        int xf=util::Constrain<int>(x/bl_size.w,0,size.w-1);

        int yf=util::Constrain<int>(y/bl_size.h,0,size.h-1);

        return data(xf,yf).visibility;
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

    Image<RGB24Pixel> *getImgc() const;
    void setImgc(Image<RGB24Pixel> *value);
    void calcSurfArea();
private:
    bool inboundary(int x, int y);
};
}
#endif // DEPTH_FILLER_H
