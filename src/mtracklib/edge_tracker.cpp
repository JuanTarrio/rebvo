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



#include "mtracklib/edge_tracker.h"

#include <iostream>

#include <stdio.h>
#include <TooN/SVD.h>
#include <TooN/Cholesky.h>

using namespace TooN;
namespace  rebvo{

//********************************************************************
// rotate_keylines(): Rotates the keylines acording to matrix RotF
//********************************************************************
void edge_tracker::rotate_keylines(TooN::Matrix <3,3> RotF){

    auto &zf=cam_mod.zfm;

    for(int i=0;i<kn;i++){

        Vector <3> q=RotF*makeVector(kl[i].p_m.x/zf,kl[i].p_m.y/zf,1);

        if(fabs(q[2])>0){

            kl[i].p_m.x=q[0]/q[2]*zf;
            kl[i].p_m.y=q[1]/q[2]*zf;


            kl[i].rho/=q[2];

            kl[i].s_rho=kl[i].s_rho/q[2];

        }

        //Nasty rotation of the keyline tangent

        q=RotF*makeVector(kl[i].m_m.x,kl[i].m_m.y,0);
        kl[i].m_m.x=q[0];
        kl[i].m_m.y=q[1];


    }

}




//********************************************************************
// Regularize_1_iter() perform regularization using thresh as angular
// threshold and the estimated uncertainty as cut-off
//********************************************************************


int edge_tracker::Regularize_1_iter(double thresh)
{

    int r_num=0;

    double r[kn],s[kn];
    bool set[kn];
    for(int ikl=0;ikl<kn;ikl++){

        set[ikl]=false;

        if(kl[ikl].n_id <0 || kl[ikl].p_id<0)       //If no neigbours, skip...
            continue;

        KeyLine &k=kl[ikl];                         //current keyline
        KeyLine &kn=kl[kl[ikl].n_id];               //next keyline
        KeyLine &kp=kl[kl[ikl].p_id];               //previus keyline


        if(util::square(kn.rho-kp.rho)>util::norm2(kn.s_rho,kp.s_rho)) //If the uncertainty doest'n fit, skip...
            continue;


        double alpha=(kn.m_m.x*kp.m_m.x+kn.m_m.y*kp.m_m.y)/(kn.n_m*kp.n_m); //

        if(alpha-thresh<0)
            continue;

        alpha=(alpha-thresh)/(1-thresh);    //alpha is a weigthing factor, goes from 0 if cos(angle_bethen_gradients)<thresh
                                            //to 1 if the angles coincide

        double wr=1/(k.s_rho*k.s_rho);          //the regularization is also sigma-weighted
        double wrn=alpha/(kn.s_rho*kn.s_rho);
        double wrp=alpha/(kp.s_rho*kp.s_rho);


        r[ikl]=(k.rho*wr+kn.rho*wrn+kp.rho*wrp)/(wr+wrn+wrp);       //regularized rho is a linear combination

        s[ikl]=(k.s_rho*wr+kn.s_rho*wrn+kp.s_rho*wrp)/(wr+wrn+wrp); //also for the uncertainty, it is mixed using a linear
                                                                    //correlation model

        set[ikl]=true;

        r_num++;

    }
    for(int ikl=0;ikl<kn;ikl++){    //Save the regularization in the edge-map
        if(set[ikl]){
            kl[ikl].rho=r[ikl];
            kl[ikl].s_rho=s[ikl];
        }
    }
    return r_num;

}



//********************************************************************
// search_match() search for a matchng KeyLine in the direction of the stereo 
// lines, using the ImgMask for this object
//********************************************************************


int edge_tracker::search_match(
        KeyLine &k,                     //KeyLine to search for
        TooN::Vector <3> Vel,           //Estimated translation
        TooN::Matrix <3,3> RVel,        //Translation Uncertainty
        TooN::Matrix <3,3> BackRot,     //Estimated BackRotation
        double min_thr_mod,             //Gradient matching threshold on the module
        double min_thr_ang,             //Gradient matching threshold on the angle (degrees)
        double max_radius,              //Max Pixel Distance to search for
        double loc_uncertainty          //Location Uncertainty
        )
{

    const double cang_min_edge=cos(min_thr_ang*M_PI/180.0);

    double dq_min=0,dq_max=0,t_x=0,t_y=0,dq_rho=0;
    int t_steps=0;

    Point2DF p_m;
    double k_rho;


    Vector <3> p_m3=BackRot*makeVector(k.p_m.x,k.p_m.y,cam_mod.zfm);    //Back-Rotate the NewKeypoint
                                                                        //to look on the old EdgeMap's mask
    
    p_m.x=p_m3[0]*cam_mod.zfm/p_m3[2];                              //Proyect to image plain
    p_m.y=p_m3[1]*cam_mod.zfm/p_m3[2];    
    k_rho=k.rho*cam_mod.zfm/p_m3[2];
    

    Point2DF pi0=cam_mod.Hom2Img(p_m);                               //Transform to image coordinates


    t_x=-(Vel[0]*cam_mod.zfm-Vel[2]*p_m.x);                             //Displacement vector t such that
    t_y=-(Vel[1]*cam_mod.zfm-Vel[2]*p_m.y);                             //t*rho=pixel_displacement

    double norm_t=util::norm(t_x,t_y);

    Vector <3> DrDv=makeVector(cam_mod.zfm,cam_mod.zfm,-p_m.x-p_m.y);

    double sigma2_t=(DrDv.as_row()*RVel*DrDv.as_col())(0,0);             //Estimated uncertainty in the displacement


    if(norm_t>1e-6){            //If displacement (|Vel|>0)
        t_x/=norm_t;            //Normalize t, keep the mod in norm_t
        t_y/=norm_t;

        dq_rho=norm_t*k_rho;    //Prior for the translation distance, only usefull if forward matching exist

        dq_min=std::max(0.0,norm_t*(k_rho-k.s_rho))-loc_uncertainty;         //Minimun distance to seach for, constrained by -ERR_DQ
        dq_max=std::min(max_radius,norm_t*(k_rho+k.s_rho))+loc_uncertainty;  //Maximun distance to seach for, constrained by max_radius+ERR_DQ

        if(dq_rho>dq_max){                      //If stating point grater than max
            dq_rho=(dq_max+dq_min)/2;           //Start from the midle in both direction
            t_steps=util::round2int_positive(dq_rho);    //Round the number of iterations to perfom
        }else{
            t_steps=util::round2int_positive(std::max(dq_max-dq_rho,dq_rho-dq_min)); //Seach start from dq_rho to both dicerctions, the
        }                                                                   //number of iterations is the maximun
    }else{  //If no displacemt (not common) search in perpendicular direction of the edge
        t_x=k.m_m.x;
        t_y=k.m_m.y;
        norm_t=k.n_m;
        t_x/=norm_t;
        t_y/=norm_t;
        norm_t=1;
        dq_min=-max_radius-loc_uncertainty;
        dq_max=max_radius+loc_uncertainty;

        dq_rho=0;
        t_steps=dq_max;


    }


    const double& norm_m=k.n_m;

    double tn=dq_rho,tp=dq_rho+1; //Displacement counter

    for(int t_i=0;t_i<t_steps;t_i++,tp+=1,tn-=1){


        for(int i_inx=0;i_inx<2;i_inx++){   //For both directions

            double t;

            if(i_inx){          //Select direction
                t=tp;
                if(t>dq_max)
                    continue;
            }else{
                t=tn;
                if(t<dq_min)
                    continue;
            }




            int inx,j;

            if((inx=img_mask_kl.GetIndexRC(t_x*t+pi0.x,t_y*t+pi0.y))<0)  //Image position to seach for,
                continue;                                               // -1 means out of border


            if((j=img_mask_kl[inx])>=0){        //If there is an edge at this position

                const double &norm_m0=kl[j].n_m;

                double cang=(kl[j].m_m.x*k.m_m.x+kl[j].m_m.y*k.m_m.y)/(norm_m0*norm_m);

                if(cang<cang_min_edge || fabs(norm_m0/norm_m-1)>min_thr_mod)    //check angle and modulus thresholds
                    continue;                                                   //if no match keep searching


                double &s_rho=kl[j].s_rho;
                double &rho=kl[j].rho;

                double v_rho_dr=(                           //Estimated uncertainty on the displacement
                            loc_uncertainty*loc_uncertainty //Edge location error
                            +s_rho*s_rho*norm_t*norm_t      //rho uncertainty error
                            +sigma2_t*rho*rho);             //velocity uncertainty error
                
                if(util::square(t-norm_t*rho)>v_rho_dr)     //test of model consistency
                    continue;

                return j;                                   //Al test passed, match OK!


            }

        }

    }

    return -1;


}

//***********************************************************************
// directed_matching() match the KeyLines of this object in the direction
// of the stereo lines, using the ImgMask of et0
//***********************************************************************

int edge_tracker::directed_matching(
        TooN::Vector <3> Vel,           //Estimated velocity
        TooN::Matrix <3,3> RVel,        //Estimated uncertainty on the velocity
        TooN::Matrix <3,3> BackRot,     //Estimated back-rotation
        edge_tracker *et0,              //EdgeMap were to look for matches
        double min_thr_mod,             //Threshold on the module
        double min_thr_ang,             //Threshold on the angle
        double max_radius,              //Maximun search distance
        double loc_uncertainty,         //Location Uncertainty
        bool clear)                     //Clear the matches? default false
{


    nmatch=0;


    int i_mch;

    Vel=BackRot*Vel;                //Back rotate translation
    RVel=BackRot*RVel*BackRot.T();  //and uncertainty

    for(int i_kn=0;i_kn<kn;i_kn++){


        if(clear){                  //optionally clear previous match
            kl[i_kn].m_id=-1;
            kl[i_kn].m_num=0;
        }

        //Search for a match

        if((i_mch=et0->search_match(kl[i_kn],Vel,RVel,BackRot,min_thr_mod,min_thr_ang,max_radius,loc_uncertainty))<0)
            continue;


        //If a match is found clone data from et0

        kl[i_kn].rho=et0->kl[i_mch].rho;
        kl[i_kn].s_rho=et0->kl[i_mch].s_rho;

        kl[i_kn].m_id=i_mch;
        kl[i_kn].m_num=et0->kl[i_mch].m_num+1;

        kl[i_kn].p_m_0=et0->kl[i_mch].p_m;
        kl[i_kn].m_m0=et0->kl[i_mch].m_m;
        kl[i_kn].n_m0=et0->kl[i_mch].n_m;

        nmatch++;


    }


    return nmatch;

}

//***********************************************************************
// FordwardMatch() use the information from tracking to do a pre-match
//***********************************************************************

int edge_tracker::FordwardMatch(edge_tracker *et    //Edgemap to match to
                                ,bool clear)
{

    double nmatch=0;


    if(clear){  //optionally clear match
        for(int ikl=0;ikl<et->kn;ikl++){

            et->kl[ikl].m_id=-1;
            et->kl[ikl].m_num=0;
        }
    }

    for(int ikl=0;ikl<kn;ikl++){

        KeyLine &k=kl[ikl];

        int &ikl_f=k.m_id_f;    //Forward match avaiable? (this is assign by GlobalTracker::TryVelRot)
        if(ikl_f<0)
            continue;           //No... continue
        if(ikl_f>=et->kn){      //This should never happen
            printf("\nEdgeTracker::Forward Match Error %d %d!\n",ikl_f,et->kn);
            continue;
        }

        //If match, clone to et
        et->kl[ikl_f].rho=k.rho;
        et->kl[ikl_f].s_rho=k.s_rho;

        et->kl[ikl_f].m_num=k.m_num+1;

        et->kl[ikl_f].m_id=ikl;

        et->kl[ikl_f].p_m_0=k.p_m;

        et->kl[ikl_f].m_m0=k.m_m;
        et->kl[ikl_f].n_m0=k.n_m;
        nmatch++;

    }

    et->nmatch=nmatch;

    return nmatch;


}

//***********************************************************************
// UpdateInverseDepthKalman() use EKF to update for depth estimates
//***********************************************************************


void edge_tracker::UpdateInverseDepthKalman(Vector <3> vel,             //Estimated tranlation
                                            Matrix <3,3> RVel,          //Uncertainty on translation
                                            TooN::Matrix<3, 3> RW0,     //Uncertainty on rotation
                                            double ReshapeQAbsolute,    //Constant uncertainty added to depth error
                                            double ReshapeQRelative,    //Relative uncertainty added to depth error
                                            double LocationUncertainty) //Location error
{

    for(int i=0;i<kn;i++){

#ifdef USE_ONLY_EDGES
        if(!kl[i].is_edge)
            continue;
#endif

        if(kl[i].m_id>=0){      //For each keyline if a match is avaiable
            UpdateInverseDepthKalmanSimple(kl[i],vel,RVel,RW0,ReshapeQAbsolute,ReshapeQRelative,LocationUncertainty);
        }


    }

}

//************************************************************************
// UpdateInverseDepthKalman() use EKF to update for depth estimates of kli
// parameters are the same
//************************************************************************


double edge_tracker::UpdateInverseDepthKalman(KeyLine &kli,
                                              Vector <3> vel,
                                              Matrix <3,3> RVel,
                                              Matrix <3,3> RW0,
                                              double ReshapeQAbsolute,
                                              double ReshapeQRelative,
                                              double LocationUncertainty){



    const double &zf=cam_mod.zfm;       //camera focal length

    if(kli.s_rho<0 || kli.rho<0){       //Debug check
        std::cout << "\nUIDK panic! "<<kli.s_rho<<" "<<kli.rho<<"\n";
    }

    kli.s_rho0=kli.s_rho;

    double qx,qy,q0x,q0y;
    qx=kli.p_m.x;       //KeyLine new homogeneus coordinates
    qy=kli.p_m.y;


    q0x=kli.p_m_0.x;    //Keyline old homogeneus coordinates
    q0y=kli.p_m_0.y;


    double v_rho=kli.s_rho*kli.s_rho;   //IDepth variance

    double u_x=kli.m_m0.x/kli.n_m0;     //Shortcut for edge perpendicular direction
    double u_y=kli.m_m0.y/kli.n_m0;



    double Y = u_x*(qx-q0x)+u_y*(qy-q0y); //Pixel displacement proyected on u
    double H= u_x*(vel[0]*zf-vel[2]*q0x)+u_y*(vel[1]*zf-vel[2]*q0y);



    double rho_p=1/(1.0/kli.rho+vel[2]);    //Predicted Inverse Depth

    kli.rho0=rho_p;                         //BackUp for the predicted ID

    double F=1/(1+kli.rho*vel[2]);          //Jacobian of the
    F=F*F;                                  //prediction ecuation

    double p_p=F*v_rho*F+                               //Uncertainty propagation
            util::square(kli.rho*ReshapeQRelative)+     //Relative uncertainty model
            rho_p*rho_p*RVel(2,2)*rho_p*rho_p+          //Uncertainty on the Z-velocity
            util::square(ReshapeQAbsolute);             //Absolute uncertainty model



    double e=Y-H*rho_p;                     //error correction

    Matrix <1,6> Mk;

    Mk[0]=makeVector(-1,u_x*rho_p*zf,u_y*rho_p*zf,-rho_p*(u_x*q0x+u_y*q0y),u_x*(rho_p*vel[2]),u_y*(rho_p*vel[2]));  //parcial derivative of
                                                                                                                    //the correction ecuation
                                                                                                                    //with respecto to uncertainty sources
    Matrix <6,6> R = Zeros;

    R(0,0)=util::square(LocationUncertainty);
    R.slice <1,1,3,3> ()=RVel;
    R(4,4)=util::square(LocationUncertainty);
    R(5,5)=util::square(LocationUncertainty);


    //*** Kalman update ecuations ****
    double S=H*p_p*H+(Mk*R*Mk.T())(0,0);

    double K=p_p*H*(1/S);

    kli.rho=rho_p+(K*e);

    v_rho=(1-K*H)*p_p;

    kli.s_rho=sqrt(v_rho);


    //*** if inverse depth goes beyond limits apply correction ****

    if(kli.rho<RHO_MIN){
        kli.s_rho+=RHO_MIN-kli.rho;
        kli.rho=RHO_MIN;
    }else if(kli.rho>RHO_MAX){
        kli.rho=RHO_MAX;
        //kli.s_rho+=RHO_MAX;
    }else if(std::isnan(kli.rho) || std::isnan(kli.s_rho) || std::isinf(kli.rho) || std::isinf(kli.s_rho)){     //This checks should never happen
        std::cout<<"\nKL EKF Nan Rho!"<<rho_p<< " " << p_p<< " "<<vel[2]<<"\n";
        kli.rho=RhoInit;
        kli.s_rho=RHO_MAX;
    }else if(kli.s_rho<0){                                                                  //only for debug
        std::cout<<"\nKL EKF Panic! Neg VRho!"<<vel<<RVel <<kli.rho << kli.s_rho <<"\n";
        kli.rho=RhoInit;
        kli.s_rho=RHO_MAX;

    }

    return kli.rho;


}




//************************************************************************
// UpdateInverseDepthKalmanSimple() use EKF to update for depth estimates of kli
// parameters are the same
//************************************************************************


double edge_tracker::UpdateInverseDepthKalmanSimple(KeyLine &kli,
                                              Vector <3> vel,
                                              Matrix <3,3> RVel,
                                              Matrix <3,3> RW0,
                                              double ReshapeQAbsolute,
                                              double ReshapeQRelative,
                                              double LocationUncertainty){



    const double &zf=cam_mod.zfm;       //camera focal length

    if(kli.s_rho<0 || kli.rho<0){       //Debug check
        std::cout << "\nUIDK panic! "<<kli.s_rho<<" "<<kli.rho<<"\n";
    }

    kli.s_rho0=kli.s_rho;

    double qx,qy,q0x,q0y;
    qx=kli.p_m.x;       //KeyLine new homogeneus coordinates
    qy=kli.p_m.y;


    q0x=kli.p_m_0.x;    //Keyline old homogeneus coordinates
    q0y=kli.p_m_0.y;


    double v_rho=kli.s_rho*kli.s_rho;   //IDepth variance

    double u_x=kli.m_m0.x/kli.n_m0;     //Shortcut for edge perpendicular direction
    double u_y=kli.m_m0.y/kli.n_m0;



    double Y = u_x*(qx-q0x)+u_y*(qy-q0y); //Pixel displacement proyected on u
    double H= u_x*(vel[0]*zf-vel[2]*q0x)+u_y*(vel[1]*zf-vel[2]*q0y);



    double rho_p=1/(1.0/kli.rho+vel[2]);    //Predicted Inverse Depth

    kli.rho0=rho_p;                         //BackUp for the predicted ID

    double F=1/(1+kli.rho*vel[2]);          //Jacobian of the
    F=F*F;                                  //prediction ecuation

    double p_p=F*v_rho*F+                               //Uncertainty propagation
            util::square(ReshapeQAbsolute);             //Absolute uncertainty model



    double e=Y-H*rho_p;                     //error correction
/*
    Matrix <1,6> Mk;

    Mk[0]=makeVector(-1,u_x*rho_p*zf,u_y*rho_p*zf,-rho_p*(u_x*q0x+u_y*q0y),u_x*(rho_p*vel[2]),u_y*(rho_p*vel[2]));  //parcial derivative of
                                                                                                                    //the correction ecuation
                                                                                                                    //with respecto to uncertainty sources
    Matrix <6,6> R = Zeros;

    R(0,0)=util::square(LocationUncertainty);
    R.slice <1,1,3,3> ()=RVel;
    R(4,4)=util::square(LocationUncertainty);
    R(5,5)=util::square(LocationUncertainty);
*/

    //*** Kalman update ecuations ****
    double S=H*p_p*H+LocationUncertainty*LocationUncertainty;

    double K=p_p*H*(1/S);

    kli.rho=rho_p+(K*e);

    v_rho=(1-K*H)*p_p;

    kli.s_rho=sqrt(v_rho);


    //*** if inverse depth goes beyond limits apply correction ****

    if(kli.rho<RHO_MIN){
        kli.s_rho+=RHO_MIN-kli.rho;
        kli.rho=RHO_MIN;
    }else if(kli.rho>RHO_MAX){
        kli.rho=RHO_MAX;
        //kli.s_rho+=RHO_MAX;
    }else if(std::isnan(kli.rho) || std::isnan(kli.s_rho) || std::isinf(kli.rho) || std::isinf(kli.s_rho)){     //This checks should never happen
        std::cout<<"\nKL EKF Nan Rho!"<<rho_p<< " " << p_p<< " "<<vel[2]<<"\n";
        kli.rho=RhoInit;
        kli.s_rho=RHO_MAX;
    }else if(kli.s_rho<0){                                                                  //only for debug
        std::cout<<"\nKL EKF Panic! Neg VRho!"<<vel<<RVel <<kli.rho << kli.s_rho <<"\n";
        kli.rho=RhoInit;
        kli.s_rho=RHO_MAX;

    }

    return kli.rho;


}



//********************************************************************************
// EstimateReScaling() calculates the relation between the predicted and updated
// inverse depth and optionally apllyes a global reescaling
//********************************************************************************

double edge_tracker::EstimateReScaling(double &RKp,             //Estimated uncertainty on global depth
                                       const double &s_rho_min, //uncertainty threshold
                                       const uint &MatchNumMin, //match number threshold
                                       bool re_escale)
{

    if(kn<=0)
        return 1;

    double rTr=0,rTr0=0;
    for(int ikl=0;ikl<kn;ikl++){

        if(kl[ikl].m_num<MatchNumMin || kl[ikl].s_rho0<=0 || kl[ikl].s_rho>s_rho_min)
            continue;
        rTr+=kl[ikl].rho*kl[ikl].rho/(kl[ikl].s_rho0*kl[ikl].s_rho0);       //Weighted mean of corrected IDepth
        rTr0+=kl[ikl].rho0*kl[ikl].rho0/(kl[ikl].s_rho0*kl[ikl].s_rho0);    //Weighted mean of predicted IDepth
        if(std::isnan(kl[ikl].rho0 || kl[ikl].rho)){
           printf("\n NAN: %f %f %f",kl[ikl].s_rho0,kl[ikl].rho0,kl[ikl].rho);
        }
    }

    double Kp=rTr0>0?sqrt(rTr/rTr0):1;  //Estimated Ratio
    RKp=1/rTr;

    if(re_escale){                      //Optionally reescale
        for(int ikl=0;ikl<kn;ikl++){
           kl[ikl].rho/=Kp;
           kl[ikl].s_rho/=Kp;
        }
    }
    return Kp;

}


//********************************************************************************
// EstimateQuantile() uses histograms to estimate an uncertainty threshold
// for a certain quatile
//********************************************************************************

double edge_tracker::EstimateQuantile(double s_rho_min,     //Starting uncertainty on the histo
                                      double s_rho_max,     //Final uncertainty on the histo
                                      double percentile,    //Quantile threshold
                                      int n){               //Number of bins on the histo


    int histo[n];


    for(int i=0;i<n;i++){
        histo[i]=0;
    }

    for(int ikl=0;ikl<kn;ikl++){
        int i=n*(kl[ikl].s_rho-s_rho_min)/(s_rho_max-s_rho_min);    //histogram position

        i=i>n-1?n-1:i;
        i=i<0?0:i;

        histo[i]++;     //count
    }

    double s_rho=1e3;

    for(int i=0,a=0;i<n;i++){

        if(a>percentile*kn){    //count in the histogram until a certain percentile
                                //of the total number of keylines
            s_rho=(double)i*(s_rho_max-s_rho_min)/(double)n+s_rho_min;
            break;

        }

        a+=histo[i];
    }

    return s_rho;

}


void edge_tracker::DebugMatchHisto(uint m_num_max,uint bin_num,uint *histo){

    for(int i=0;i<bin_num;i++)
        histo[i]=0;

    for(KeyLine &kl:*this){

        int i=(kl.m_num*bin_num)/m_num_max;
        histo[util::Constrain<int>(i,0,bin_num-1)]++;
    }

}


//***********************************************************************************
// ExtRotVel() uses forward match to estimate a RotoTranslation using linear aprox
//***********************************************************************************

bool edge_tracker::ExtRotVel(const Vector <3> &vel,     //Prevously estimated translation
                             Matrix <6,6> &Wx,          //Output information matrix
                             Matrix <6,6> &Rx,          //Wx^-1
                             Vector <6> &X,             //Output incremental state
                             const double &LocUncert,   //Location uncertainty
                             double HubReweigth
                             )
{


    int nmatch1D=0;
    for(int i=0;i<kn;i++){
        if(kl[i].m_id>=0)
            nmatch1D++;
    }


    Matrix<> Phi=Zeros(nmatch1D,6);
    Vector<> Y=Zeros(nmatch1D);

    int j=0;

    for(int i=0;i<kn;i++){

        if(kl[i].m_id<0)
            continue;

        float &u_x=kl[i].u_m.x,&u_y=kl[i].u_m.y;
        double &zf=cam_mod.zfm;

        Point2DF &p_m=kl[i].p_m,&p_m_0=kl[i].p_m_0;


        double rho_t=1/(1/kl[i].rho+vel[2]);
        float qt_x=p_m_0.x+rho_t*(vel[0]*zf-vel[2]*p_m_0.x);
        float qt_y=p_m_0.y+rho_t*(vel[1]*zf-vel[2]*p_m_0.y);

        double &s_rho=kl[i].s_rho;

        float q_x=p_m.x;
        float q_y=p_m.y;


        Phi(j,0) = u_x*rho_t*zf;
        Phi(j,1) = u_y*rho_t*zf;
        Phi(j,2) = u_x*(-rho_t*q_x)+u_y*(-rho_t*q_y);

        Phi(j,3) =-u_x*q_x*q_y/zf-u_y*(zf+q_y*q_y/zf);
        Phi(j,4) =+u_y*q_x*q_y/zf+u_x*(zf+q_x*q_x/zf);
        Phi(j,5) =-u_x*q_y+u_y*q_x;

        Y[j]=u_x*(p_m.x-qt_x)+u_y*(p_m.y-qt_y);

        float dqvel=u_x*(vel[0]*zf-vel[2]*p_m_0.x)+u_y*(vel[1]*zf-vel[2]*p_m_0.y);
        float s_y=sqrt(s_rho*s_rho*dqvel*dqvel+LocUncert*LocUncert);

        double weigth=1;
        if(fabs(Y[j])>HubReweigth)
            weigth=fabs(Y[j])/HubReweigth;

        Phi.slice(j,0,1,6)/=s_y*weigth;
        Y[j]/=s_y*weigth;

        j++;


    }

    if(j!=nmatch1D){
        printf("\nExtRotVel j err %d %d!!!\n",j,nmatch1D);
        return false;
    }





    Matrix <6,6> JtJ=Phi.T()*Phi;
    Vector <6> JtF=Phi.T()*Y;

    SVD <>SVDpTp(JtJ);

    X=SVDpTp.backsub(JtF);
    Rx=SVDpTp.get_pinv();
    Wx=JtJ;

    if(util::isNaN(Rx) || util::isNaN(X)){
        printf("\nEdge Tracker::ExtRotVel estimation Fail!\n");
        return false;
    }

    return true;

}


//***********************************************************************************
// BiasCorrect() correct rototranlation vector using giroscope prior
//***********************************************************************************

void edge_tracker::BiasCorrect(TooN::Vector <6> &X,     //Previous estimated state and output state
                               TooN::Matrix<6,6> &Wx,   //Previous estimated and output state information matrix
                               TooN::Vector <3> &Gb,    //Previous estimated and output bias
                               TooN::Matrix<3,3> &Wb,   //Previous estimated and output state information matrix
                               const TooN::Matrix<3,3> &Rg,     //Giroscope meassurement covariance
                               const TooN::Matrix<3,3> &Rb      //Giroscope bias covariance
                               )
{

    using namespace TooN;

    const Matrix<3,3> &Wg=util::Matrix3x3Inv(Rg);       //Giroscope meassurement information

    Wb=util::Matrix3x3Inv(util::Matrix3x3Inv(Wb)+Rb);   //Bias uncertainty update

    Matrix<6,6> Wxb=Wx;

    Matrix<3,3> iWgWb=util::Matrix3x3Inv(Wg+Wb);

    Wxb.slice<3,3,3,3>()+=Wg*(Identity - iWgWb*Wg);

    Vector <6> X1=Wx*X;
    X1.slice<3,3>()+=Wg*iWgWb*Wb*Gb;

    X=Cholesky<6>(Wxb).get_inverse()*X1;

    Gb=iWgWb*(Wg*X.slice<3,3>()+Wb*Gb);
    Wb=Wg+Wb;

    Wx.slice<3,3,3,3>()+=Wg;
}

}
