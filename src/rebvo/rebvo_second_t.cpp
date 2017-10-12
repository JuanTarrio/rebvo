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


#include <iostream>
#include <iomanip>
#include <TooN/so3.h>
#include <TooN/Cholesky.h>

#include "rebvo/rebvo.h"
#include "UtilLib/ttimer.h"
#include "VideoLib/datasetcam.h"
#include "mtracklib/scaleestimator.h"
#include "mtracklib/kfvo.h"
#include <TooN/SVD.h>


using namespace std;

namespace  rebvo{


void  REBVO::SecondThread(REBVO *cf){

    using namespace TooN;



    double dt_frame;

    COND_TIME_DEBUG(double dtp;)


    int n_frame=0;
    double Kp=1,K=1;

    double error_vel=0;                 //Estimated error for the velocity
    double error_score=0;               //Residual energy from the tracking

    Vector <3> V=Zeros,W=Zeros,Pos=Zeros;   //Estimated Velocity, Rotation and Position
    Matrix<3,3> R=Identity,Pose=Identity;   //Rotation Matrix and Global Rotation

    Matrix<3,3> Rgva=Identity;              //Rotation Matrix with filter correction

    double P_Kp=5e-6;
    Matrix <3,3> P_V=Identity*1e50,P_W=Identity*1e-10;

    IMUState istate;


    istate.W_Bg=util::Matrix3x3Inv(istate.RGBias*100);

    istate.Qg=Identity*cf->params.g_uncert*cf->params.g_uncert;
    istate.Rg=cf->params.g_module_uncer*cf->params.g_module_uncer;
    istate.Rs=Identity*cf->params.AcelMeasStdDev*cf->params.AcelMeasStdDev;
    istate.Qbias=Identity*cf->params.VBiasStdDev*cf->params.VBiasStdDev;

    istate.X=makeVector(M_PI/4,0,cf->params.g_module,0,0,0,0);
    istate.P=makeVector(cf->params.ScaleStdDevInit*cf->params.ScaleStdDevInit,\
                        100,100,100,\
                        cf->params.VBiasStdDev*cf->params.VBiasStdDev*1e1,cf->params.VBiasStdDev*cf->params.VBiasStdDev*1e1,cf->params.VBiasStdDev*cf->params.VBiasStdDev*1e1).as_diagonal();

    istate.u_est=makeVector(1,0,0);

    int n_giro_init=0;
    Vector <3> giro_init=Zeros;
    Vector <3> g_init=Zeros;


    //**** Set cpu Afinity for this thread *****

    if(cf->params.cpuSetAffinity){
        if(!REBVO::setAffinity(cf->params.cpu1)){
            std::cout <<"REBVO: Cannot set cpu affinity on the second thread";
            cf->quit=true;
        }
    }


    //***** Launch Thread 2 Pipeline thread ******

    std::thread Thr2(ThirdThread,cf);

    COND_TIME_DEBUG(util::interval_list tlist;)

    COND_TIME_DEBUG(util::timer t_loop;)

    //****** Dummy processing of the first frame ******

	{
    	PipeBuffer &new_buf=cf->pipe.RequestBuffer(1);
    	if(new_buf.quit){
        	cf->pipe.ReleaseBuffer(1);
        	PipeBuffer &old_buf=cf->pipe.RequestBuffer(2);
        	old_buf.quit=false;
        	cf->pipe.ReleaseBuffer(2);
        	cf->quit=true;

    	}else{
    		cf->pipe.ReleaseBuffer(1);
    	}
	}
    //****** Main loop *******


    util::timer tproc;

    while(!cf->quit){

        bool EstimationOk=true;

        COND_TIME_DEBUG(dtp=t_loop.stop();std::cout << "1st exit" << std::endl;)

        // 2 pipeline buffers are used in this thread
        PipeBuffer &new_buf=cf->pipe.RequestBuffer(1);  //This represent the newest edge-map
        PipeBuffer &old_buf=cf->pipe.RequestBuffer(2);  //This represents the old edgemap

        if(new_buf.quit){                               //If quit flag on the new buffer,
            old_buf.quit=true;                          //release quit on the old for thread 2
            cf->pipe.ReleaseBuffer(1);
            cf->pipe.ReleaseBuffer(2);
            break;
        }

        dt_frame=(new_buf.t-old_buf.t);

        if(dt_frame<0.001)
            dt_frame=1/cf->params.config_fps;

        tproc.start();
        COND_TIME_DEBUG(t_loop.start();)
        COND_TIME_DEBUG(tlist.clear();)



        if(cf->params.TrackKeyFrames && cf->kf_list.size()==0){
            old_buf.K=1;
            cf->kf_list.push_back(keyframe(*old_buf.ef,*old_buf.gt,old_buf.t,old_buf.K,old_buf.nav.Rot,old_buf.nav.RotLie,old_buf.nav.Vel,old_buf.nav.Pose,old_buf.nav.PoseLie,old_buf.nav.Pos));
            std::cout <<"\nadded keyframe\n";
            kfvo::resetForwardMatch(cf->kf_list.back());
            kfvo::resetKFMatch(*old_buf.ef);
        }


        int klm_num=0,num_kf_fow_m=0,num_kf_back_m=0;
        P_V=Identity*1e50;
        P_W=Identity*1e50;
        R=Identity;

        COND_TIME_DEBUG(tlist.push_new();)
        //Estimate uncertainity cut-off for some quantile (90 percent typicaly)
        double s_rho_q=old_buf.ef->EstimateQuantile(RHO_MIN,RHO_MAX,cf->params.QCutOffQuantile,cf->params.QCutOffNumBins);


        COND_TIME_DEBUG(tlist.push_new();)

        new_buf.gt->build_field(*new_buf.ef,cf->params.SearchRange,new_buf.ef->getThresh());


        //***** Use the Tracker to obtain Velocity and Rotation  ****

        if(cf->params.ImuMode>0){      //Imu data avaiable?? Use it!

            if(!istate.init  && n_frame>0){

                if(cf->params.InitBias>0){
                    giro_init+=new_buf.imu.giro*new_buf.imu.dt;

                    g_init-=new_buf.imu.cacel;
                    //giro_init+=istate.dWv;
                    if(++n_giro_init>cf->params.InitBiasFrameNum){
                        istate.Bg=giro_init/n_giro_init;
                        istate.init=true;
                        istate.W_Bg=util::Matrix3x3Inv(istate.RGBias*1e2);
                        istate.X.slice<1,3>()=g_init/n_giro_init;
                    }
                }else{
                    istate.init=true;
                    istate.Bg=cf->params.BiasInitGuess*new_buf.imu.dt;
                }
            }




            R=new_buf.imu.Rot;  //Use imu rotation


            R.T()=TooN::SO3<>(istate.Bg)*R.T();

            old_buf.ef->rotate_keylines(R.T()); //Apply foward prerotation to the old key lines


            COND_TIME_DEBUG(tlist.push_new();)

            if(cf->params.TrackerInitType==0){
                istate.Vg=Zeros;
            }

            #ifdef USE_NE10
            new_buf.gt->Minimizer_V<float>(istate.Vg,istate.P_Vg,*old_buf.ef,cf->params.TrackerMatchThresh,cf->params.TrackerIterNum,s_rho_q,cf->params.MatchNumThresh,cf->params.ReweigthDistance,old_buf.ef->getThresh());   //Estimate translation only
            #else
            new_buf.gt->Minimizer_V<double>(istate.Vg,istate.P_Vg,*old_buf.ef,cf->params.TrackerMatchThresh,cf->params.TrackerIterNum,s_rho_q,cf->params.MatchNumThresh,cf->params.ReweigthDistance,old_buf.ef->getThresh());   //Estimate translation only
            #endif

            COND_TIME_DEBUG(tlist.push_new();)

            //***** Match from the old EdgeMap to the new one using the information from the minimization *****
            old_buf.ef->FordwardMatch(new_buf.ef);


            COND_TIME_DEBUG(tlist.push_new();)

            //***** Visual RotoTranslation estimation using forward matches
            Matrix <6,6> R_Xv,R_Xgv,W_Xv,W_Xgv;
            Vector <6> Xv,Xgv,Xgva;
            EstimationOk&=new_buf.ef->ExtRotVel(istate.Vg,W_Xv,R_Xv,Xv,cf->params.LocationUncertainty,cf->params.ReweigthDistance);



            COND_TIME_DEBUG(tlist.push_new();)

            istate.dVv=Xv.slice<0,3>();
            istate.dWv=Xv.slice<3,3>();

            Xgv=Xv;
            W_Xgv=W_Xv;


            istate.RGBias=Identity*cf->params.GiroBiasStdDev*cf->params.GiroBiasStdDev*dt_frame*dt_frame;
            istate.RGiro=Identity*cf->params.GiroMeasStdDev*cf->params.GiroMeasStdDev*dt_frame*dt_frame;

            Vector <3> dgbias=Zeros;
            edge_tracker::BiasCorrect(Xgv,W_Xgv,dgbias,istate.W_Bg,istate.RGiro,istate.RGBias);
            istate.Bg+=dgbias;


            istate.dVgv=Xgv.slice<0,3>();
            istate.dWgv=Xgv.slice<3,3>();


            //***** extract rotation matrix *****
            Rgva=R;                                     //Save previous matrix
            SO3 <>R0(istate.dWgv);                      //R0 is a forward rotation
            R.T()=R0.get_matrix()*R.T();                //R is a backward rotation


            istate.Vgv=R0*istate.Vg+istate.dVgv;
            V=istate.Vgv;

            istate.Wgv=SO3<>(R).ln();
            R_Xgv=Cholesky<6>(W_Xgv).get_inverse();


            P_V=R_Xgv.slice<0,0,3,3>();
            P_W=R_Xgv.slice<3,3,3,3>();


            COND_TIME_DEBUG(tlist.push_new();)

            //Mix with accelerometer using bayesian filter


            ScaleEstimator::EstAcelLsq4(-istate.Vgv/dt_frame,istate.Av,R,dt_frame);
            ScaleEstimator::MeanAcel4(new_buf.imu.cacel,istate.As,R);

            Xgva=Xgv;


            istate.Rv=(P_V/(dt_frame*dt_frame*dt_frame*dt_frame));
            istate.Qrot=P_W;
            istate.QKp=P_Kp;// std::min((1/(1/istate.Rv(0,0)+1/istate.Rv(1,1)+1/istate.Rv(2,2)))*cf->params.ScaleStdDevMult*cf->params.ScaleStdDevMult,cf->params.ScaleStdDevMax);


            if(n_frame>4+cf->params.InitBiasFrameNum){
                K=ScaleEstimator::estKaGMEKBias(istate.As,istate.Av,1,R,istate.X,istate.P,istate.Qg,
                                                istate.Qrot,istate.Qbias,istate.QKp,istate.Rg,istate.Rs,istate.Rv,
                                                istate.g_est,istate.b_est,W_Xgv,Xgva,cf->params.g_module);

                istate.dVgva=Xgva.slice<0,3>();
                istate.dWgva=Xgva.slice<3,3>();

                SO3<>R0gva(istate.dWgva);
                Rgva.T()=R0gva.get_matrix()*Rgva.T();
                istate.Vgva=R0gva*istate.Vg+istate.dVgva;

            }else{
                istate.dVgva=istate.dVgv;
                istate.dWgva=istate.dWgv;
                Rgva=R;
                istate.Vgva=istate.Vgv;
            }

            //K=1;


            COND_TIME_DEBUG(tlist.push_new();)

            //***** Forward Rotate the old edge-map points *****
            old_buf.ef->rotate_keylines(R0.get_matrix());


            //***** Push into pose graph *****//

            //Push relative motion into pose log
            Vector <6> relPose;
            relPose.slice<3,3>()=SO3<>(Rgva).ln();
            relPose.slice<0,3>()=-Rgva*istate.Vgva;
            Matrix <6,6> J_rp_x=Zeros;
            J_rp_x.slice<3,3,3,3>()=-Identity;
            J_rp_x.slice<0,0,3,3>()=-R;
            J_rp_x.slice<0,3,3,3>()=util::crossMatrix(relPose.slice<0,3>());
            Matrix <6,6> J_x_rp=TooN::SVD<>(J_rp_x).get_pinv();
            cf->poses.addFrameMeas(OdometryMeas(relPose,J_x_rp.T()*W_Xgv*J_x_rp,istate.As,istate.Av,istate.g_est,istate.Rs,istate.Rv,istate.P.slice<1,1,3,3>(),K,1/(istate.P(0,0)),istate.QKp,0));



        }else{                  //Regular procesing
            COND_TIME_DEBUG(tlist.push_new();)

            #ifdef USE_NE10
            TooN::Matrix<6,6,float> W_X;
            new_buf.gt->Minimizer_RV<float>(V,W,P_V,P_W,*old_buf.ef,cf->params.TrackerMatchThresh,cf->params.TrackerIterNum,cf->params.TrackerInitType,cf->params.ReweigthDistance,error_vel,error_score,s_rho_q,cf->params.MatchNumThresh,cf->params.TrackerInitIterNum,W_X);
            #else        
            TooN::Matrix<6,6,double> W_X;
            new_buf.gt->Minimizer_RV<double>(V,W,P_V,P_W,*old_buf.ef,cf->params.TrackerMatchThresh,cf->params.TrackerIterNum,cf->params.TrackerInitType,cf->params.ReweigthDistance,error_vel,error_score,s_rho_q,cf->params.MatchNumThresh,cf->params.TrackerInitIterNum,W_X);
            #endif



            COND_TIME_DEBUG(tlist.push_new();)

            //***** Match from the old EdgeMap to the new one using the information from the minimization *****
            klm_num= old_buf.ef->FordwardMatch(new_buf.ef);


            COND_TIME_DEBUG(tlist.push_new();)

            //***** extract rotation matrix *****
            SO3 <>R0(W);                    //R0 is a forward rotation
            R.T()=R0.get_matrix()*R.T();    //R is a backward rotation


            COND_TIME_DEBUG(tlist.push_new();)
            COND_TIME_DEBUG(tlist.push_new();)
            COND_TIME_DEBUG(tlist.push_new();)

            //***** Forward Rotate the old edge-map points *****
            old_buf.ef->rotate_keylines(R0.get_matrix());

            //***** Push into pose graph *****//

            //Push relative motion into pose log
            Vector <6> relPose;
            relPose.slice<3,3>()=SO3<>(R).ln();
            relPose.slice<0,3>()=-R*V;
            Matrix <6,6> J_rp_x=Zeros;
            J_rp_x.slice<3,3,3,3>()=-Identity;
            J_rp_x.slice<0,0,3,3>()=-R;
            J_rp_x.slice<0,3,3,3>()=util::crossMatrix(relPose.slice<0,3>());
            Matrix <6,6> J_x_rp=TooN::SVD<>(J_rp_x).get_pinv();
            cf->poses.addFrameMeas(OdometryMeas(relPose,J_x_rp.T()*W_X*J_x_rp,new_buf.imu.cacel,Zeros,Zeros,Zeros,Zeros,Zeros,1,0,P_Kp,0));



        }
      if(util::isNaN(V) || util::isNaN(W))  //Check for minimization errors
        {
            P_V=Identity*1e50;
            V=Zeros;

            Kp=1;
            P_Kp=1e50;

            EstimationOk=false;

            printf("\nCamara Frontal: error in the estimation, not many KeyLines (%d)?\n",new_buf.ef->KNum());
        }else{




            COND_TIME_DEBUG(tlist.push_new();)



            //***** Match from the new EdgeMap to the old one searching on the stereo line *****

            //Because the old edge map mask is used, New keylines and Translation are back rotated
            klm_num=new_buf.ef->directed_matching(V,P_V,R,old_buf.ef,num_kf_back_m,cf->params.MatchThreshModule,cf->params.MatchThreshAngle,cf->params.SearchRange,cf->params.LocationUncertaintyMatch,cf->params.StereoAvaiable);

            if(klm_num<cf->params.MatchThreshold){     //If matching keylines are below a certain threshold, reestart the estimation

                P_V=Identity*1e50;
                V=Zeros;

                Kp=1;
                P_Kp=10;

                EstimationOk=false;

                printf("\nCamara Frontal: restarting the estimation, match threshold low (%d,%d)?\n",new_buf.ef->KNum(),klm_num);
            }else{


                //***** Rebuild forward matchs *****


                if(cf->params.TrackKeyFrames){
                    keyframe &current_kf=cf->kf_list.back();

                    num_kf_fow_m=kfvo::buildForwardMatch(current_kf, *new_buf.ef,*old_buf.ef);
                    std::cout <<"Fow M0: "<<num_kf_fow_m;

                    Matrix <3> localPose=Pose*R;
                    Vector <3> localPos=Pos-localPose*V*K;

                    num_kf_fow_m=kfvo::forwardCorrectAugmentate(current_kf,*new_buf.ef,localPose,localPos,10,0,true);

                    std::cout <<" Fow M: "<<num_kf_fow_m<<" Back M0: "<<num_kf_back_m;

                    num_kf_back_m=kfvo::correctAugmentate(current_kf,*new_buf.ef,localPose,localPos,10,0,true);
                    std::cout <<" Back M: "<<num_kf_back_m<<"\n";
                }



                COND_TIME_DEBUG(tlist.push_new();)

                //****** Regularize the EdgeMap Depth ******

                for(int i=0;i<1;i++)
                        new_buf.ef->Regularize_1_iter(cf->params.RegularizeThresh);


                COND_TIME_DEBUG(tlist.push_new();)

                //****** Improve Depth using kalman filter ****

                new_buf.ef->UpdateInverseDepthKalman(V,P_V,P_W,cf->params.ReshapeQAbsolute,cf->params.ReshapeQRelative,cf->params.LocationUncertainty); //1e-5


                //****** If stereo information, use it ****

                if(cf->params.StereoAvaiable){

                    Matrix <3,3> RCam2Pair=Data(  0.999997256477450,   0.002312067192420,   0.000376008102351,
                                                 -0.002317135723285,   0.999898048506528,   0.014089835846697,
                                                 -0.000343393120589,  -0.014090668452670,   0.999900662638179);
                    Vector <3> TCam2Pair=makeVector(-0.110073808127139,0.000399121547014,-0.000853702503351);
                    new_buf.stereo_match_num=new_buf.ef->directed_matching_stereo(TCam2Pair,RCam2Pair,new_buf.ef_pair,
                                                                                  cf->params.MatchThreshModule,cf->params.MatchThreshAngle,100,
                                                                                  cf->params.LocationUncertaintyMatch,cf->params.ReshapeQAbsolute,
                                                                                  cf->params.ReshapeQRelative,cf->params.LocationUncertainty);


                }

                COND_TIME_DEBUG(tlist.push_new();)


                //****** Optionally reescale the EdgeMap's Depth
                if(cf->params.StereoAvaiable){
                    new_buf.ef->fuseStereoDepth();
                    Kp=1;
                }else{
                    Kp=new_buf.ef->EstimateReScalingOpt(P_Kp,RHO_MAX,1,cf->params.DoReScaling>0);
                }
               // cout << P_Kp<<"\n";


                COND_TIME_DEBUG(tlist.push_new();)


            }

        }


      /* 0:Quantile
       * 1:Build Field + Rotation
       * 2:Minimizer V
       * 3:Forward Match
       * 4:Ext RotVel
       * 5:Bias Correct
       * 6:Scale Filter
       * 7:Rotate KL
       * 8:Direc MAtch
       * 9:Regularize
       * 10:EKF
       * 11:Re Scalling
       * */
/*
        COND_TIME_DEBUG(printf("\nQ %f R0 %f M0 %f FM %f M2 %f BC %f BF %f R1 %f DM %f KR %f KM %f SR %f TT %f DTP %f \n",tlist[0],tlist[1],tlist[2],tlist[3],\
               tlist[4],tlist[5],tlist[6],tlist[7],tlist[8],tlist[9],tlist[10],tlist[11],tlist.total(),dtp);)
*/



        COND_TIME_DEBUG(printf("\n V %f RV %f Fl %f MP %f TT %f DTP %f KN %d\n",tlist(3,0),tlist(6,3),tlist(7,6),tlist(12,7),tlist.total(),dtp,new_buf.ef->KNum());)


        //Estimate position and pose incrementally

        if(cf->params.ImuMode>0){  //If imu present use the filtered version...


            if(n_frame>4+cf->params.InitBiasFrameNum){


                istate.u_est=Rgva.T()*istate.u_est;

                istate.u_est=istate.u_est-(istate.u_est*istate.g_est)/(istate.g_est*istate.g_est)*istate.g_est;
                TooN::normalize(istate.u_est);

                Matrix<3> PoseP1=TooN::SO3<>(istate.g_est,makeVector(0,1,0)).get_matrix();
                Matrix<3> PoseP2=TooN::SO3<>(PoseP1*istate.u_est,makeVector(1,0,0)).get_matrix();

                Pose=PoseP2*PoseP1;

                //Pose=Pose*Rgva;
                Pos+=-Pose*istate.Vgva*K;

                istate.Posgva=Pos;
                istate.Posgv+=-Pose*istate.Vgv*K;
            }

        }else{

            Pose=Pose*R;
            Pos+=-Pose*V*K;

        }




        P_V/=dt_frame*dt_frame;

        //Pass the buffer to the next thread

        new_buf.dt=dt_frame;
        new_buf.K=K;
        new_buf.Kp=Kp;
        new_buf.RKp=P_Kp;

        new_buf.nav.dt=dt_frame;
        new_buf.nav.t=new_buf.t;

        new_buf.nav.Rot=R;
        new_buf.nav.RotLie=SO3<>(R).ln();
        new_buf.nav.RotGiro=SO3<>(Rgva).ln()/dt_frame;
        new_buf.nav.Vel=-V*K/dt_frame;

        new_buf.nav.Pose=Pose;
        new_buf.nav.PoseLie=SO3<>(Pose).ln();
        new_buf.nav.Pos=Pos;
        new_buf.nav.g=istate.g_est;
        new_buf.nav.scale=K;

        new_buf.s_rho_p=s_rho_q;

        new_buf.EstimationOK=EstimationOk;

        new_buf.imustate=istate;

        //******** Push KEYFRAME ***************************//



        if(cf->saveKeyframes && cf->params.TrackKeyFrames && num_kf_back_m<std::min(cf->params.TrackPoints,new_buf.ef->KNum())*cf->params.KFSavePercent){
            cf->kf_list.push_back(keyframe(*new_buf.ef,*new_buf.gt,new_buf.t,new_buf.K,new_buf.nav.Rot,new_buf.nav.RotLie,new_buf.nav.Vel,new_buf.nav.Pose,new_buf.nav.PoseLie,new_buf.nav.Pos));
            std::cout <<"added keyframe "<<cf->kf_list.size()<<" matchs"<<num_kf_fow_m<<"\n";
            kfvo::resetForwardMatch(cf->kf_list.back());
            kfvo::resetKFMatch(*new_buf.ef);
        }

        cf->poses.odoMeas().back().KF_ID=cf->kf_list.size()-1;
        new_buf.dtp1=tproc.stop();



        //Pus the nav data in the REBVO class (thread safe)


        cf->pushNav(new_buf.nav);


        if(cf->system_reset){   //Do a depth reset to the New Edgemap
            for (auto &kl: (*new_buf.ef)) {
                kl.rho=RhoInit;                     //Rho & srho init point
                kl.s_rho=RHO_MAX;
            }
            Pose=Identity;  //Reset trayectory
            Pos=Zeros;
            W=Zeros;
            V=Zeros;

            cf->system_reset=false;
        }

        cf->pipe.ReleaseBuffer(1);
        cf->pipe.ReleaseBuffer(2);

        n_frame++;



    }



    Thr2.join();

    return;
}

}
