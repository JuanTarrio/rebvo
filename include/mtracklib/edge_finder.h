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


 #ifndef EDGE_FINDER_H
#define EDGE_FINDER_H

#include "sspace.h"
#include "VideoLib/video_io.h"
#include "UtilLib/CircList.h"
#include "UtilLib/cam_model.h"
#include "VideoLib/image.h"
#include <fstream>

namespace  rebvo{

//Goal limits on Inverse Depth and Initialization
constexpr double  RHO_MAX=20;
constexpr double  RHO_MIN=1e-3;
constexpr double  RhoInit=1;

//Maximun number of KeyLines to track for (in memory size)
constexpr int KEYLINE_MAX=50000;

struct KeyLine{

        int  p_inx;         //KeyLine linear index of image position


        Point2DF m_m;       //KeyLine's gradient vector
        Point2DF u_m;       //Normalized m_m
        float n_m;          //Norm of m_m

        float score;        //Final score given to the keyline

        Point2DF c_p;       //KeyLine's image position

        double rho;         //Estimated Inverse Depth
        double s_rho;       //Estimated Inverse Depth Uncertainty

        double rho_nr;         //Estimated Inverse Depth Non-Regularized
        double s_rho_nr;       //Estimated Inverse Depth Non-Regularized


        double rho0;        //Predicted Inverse Depth in EKF (use only if reescaling)
        double s_rho0;      //Predicted Inverse Depth Uncertainty


        Point2DF p_m;       //KL position in homogeneous coordinates (plane on focal length zf)

        Point2DF p_m_0;     //matched KL position in homogeneous coordinates

        int m_id;           //Id of the matching keyline
        int m_id_f;         //Id of the matching keyline by fordward matching
        int m_id_kf;        //Id of the matching keyline in the last keyframe

        int m_num;          //number of consecutive matches

        Point2DF m_m0;      //Grandient of matched KeyLine
        double n_m0;        //Norm of m_m0

        int p_id;           //Id of previous consecutive KeyLine
        int n_id;           //Id of next consecutive KeyLine
        int net_id;         //Network ID of the KeyLine


        int stereo_m_id;
        double stereo_rho;         //Estimated Inverse Depth from stereo
        double stereo_s_rho;       //Estimated Inverse Depth Uncertainty from stereo

};




class edge_finder
{

protected:

    cam_model cam_mod;             //Pinhole Model of the Camera

    const Size2D fsz;              //Wrapper for image size

    int max_img_value;              //Valor maximo de intensidad en las imagenes

    //**** Image Size Masks ****
    Image<int>img_mask_kl;          //pointer to Keyline Id in the image




    //**** KeyLine list ****
    int kl_size;
    KeyLine *kl;
    int kn;         //Number of KLs

    float reTunedThresh;

public:




    edge_finder(cam_model &cam, float max_i_value,int kl_num_max=KEYLINE_MAX);
    edge_finder(const edge_finder &ef);
    ~edge_finder();



    void detect(sspace *ss ,int plane_fit_size,double pos_neg_thresh,double dog_thresh,
                int kl_max, double &tresh, int &l_kl_num, int kl_ref=0, double gain=0, double thresh_max=1e10, double thresh_min=1-10);
    void build_mask(sspace *ss, int kl_max, int win_s, float per_hist, float grad_thesh, float dog_thesh);


    void join_edges();

    int reEstimateThresh(int  knum,int n);


    //***** Quick returns*****

    cam_model & GetCam(){return cam_mod;}

    int KNum()const {return kn;}

    KeyLine & operator [](uint inx){return kl[inx];}    //Index operator to access keylines directly

    float  getThresh(){return reTunedThresh;}
    //Some iterators

    typedef KeyLine * iterator;
    typedef const KeyLine * const_iterator;

    iterator begin(){return &kl[0];}
    iterator end(){return &kl[kn];}

    void annotate_keyline(int inx);

    //File save/load

    void dumpToBinaryFile(std::ofstream &file);
    void readFromBinaryFile(std::ifstream &file);


};

}
#endif // EDGE_FINDER_H
