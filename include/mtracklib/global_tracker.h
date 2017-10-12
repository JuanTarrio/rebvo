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

#ifndef GLOBAL_TRACKER_H
#define GLOBAL_TRACKER_H


#include "edge_tracker.h"

#include <UtilLib/ne10wrapper.h>

namespace  rebvo{
struct gt_field_data{
    int dist;           //Integer Distance to keyline
    int ikl;            //Keyline ID
};

class global_tracker
{

    Image<gt_field_data> field;     //Auxilary match image
    cam_model cam_mod;             //Camera model

    double max_r;                   //Max search radius

    edge_tracker *klist_f;          //Kline list used to build field

    uint FrameCount;                //Number of frame minimizations made

public:
    global_tracker(cam_model &cam);
    global_tracker(const global_tracker&gt);
    ~global_tracker();

    void build_field(edge_tracker &klist, int radius, float min_mod=-1);


    template <class T,bool ReWeight, bool ProcJF,bool UsePriors>
    double TryVelRot(TooN::Matrix<6,6,T> &JtJ,TooN::Vector<6,T> &JtF, const TooN::Vector<6,T> &VelRot, const TooN::Vector<3> &V0p, const TooN::Matrix<3,3> &PV0, const TooN::Vector<3> &W0p, const TooN::Matrix<3,3> &PW0, edge_tracker &klist,T *P0m,int pnum,double match_thresh,double s_rho_min, uint MatchNumThresh,T k_huber,T* DResidual,T* DResidualNew);

    template <class T,bool UsePriors=false>
    double Minimizer_RV(TooN::Vector<3> &Vel,TooN::Vector<3> &W0,TooN::Matrix<3,3> &RVel,TooN::Matrix<3,3> &RW0, edge_tracker &klist,double match_thresh,int iter_max,int init_type,double reweigth_distance,double &rel_error,double &rel_error_score,const double &max_s_rho,const uint& MatchNumThresh,const double& init_iter,TooN::Matrix<6,6,T> &W_X);

    template <class T>
    inline T Calc_f_J(int f_inx,T &df_dx,T &df_dy,KeyLine &kl,const Point2D<T> &p,const T &max_r,const T &simil_t,int &mnum,T &fi);
    template <class T>
    inline T Calc_f_J2(int f_inx,T &df_dx,T &df_dy,KeyLine &kl,const Point2D<T> &p,const T &max_r,const T &simil_t,int &mnum,T &fi);

    template <class T>
    inline T Calc_f_J_Complete(int f_inx,T &df_dx,T &df_dy,KeyLine &kl,const Point2D<T> &p,const T &max_r,const T &simil_mod,const T &simil_ang,int &mnum,const T rho,const T rho_tol,T &fi);

    void SetEdgeTracker(edge_tracker *et){
        klist_f=et;
    }

    const double& getMaxSRadius(){return max_r;}


    template <class T>
    double TryVel(TooN::Matrix<3,3> &JtJ, TooN::Vector<3> &JtF, const TooN::Vector<3> &Vel, edge_tracker &klist, double match_thresh, double s_rho_min,uint MatchNumThresh,T *Residuals,double reweigth_distance,float min_mod);
    template <class T>
    double TryVel_vect(TooN::Matrix<3,3> &JtJ,TooN::Vector<3> &JtF, const TooN::Vector<3> &Vel,  edge_tracker &klist, T match_thresh, double s_rho_min,uint MatchNumThresh,double reweigth_distance);
    template <class T>
    double Minimizer_V(TooN::Vector<3> &Vel, TooN::Matrix<3,3> &RVel, edge_tracker &klist,T match_thresh, int iter_max, T s_rho_min,uint MatchNumThresh,double reweigth_distance,float min_mod);




    template <class T>
    static inline bool Test_f_k(const  Point2DF &f_m   //KL1 gradient m
                         ,const KeyLine &kl     //KL2
                         ,const T &simil_t)     //Similutude threshold
    {


        T p_n2=(kl.n_m*kl.n_m);                 //norm2 of m2
        T p_esc=kl.m_m.x*f_m.x+kl.m_m.y*f_m.y;  //scalar product of m1,m2

        if(fabs(p_esc-p_n2)>simil_t*p_n2){      //| m1.m2 - m2.m2|/m2.m2 > theshold?
            return false;
        }

        return true;
    }


};



}
#endif // GLOBAL_TRACKER_H
