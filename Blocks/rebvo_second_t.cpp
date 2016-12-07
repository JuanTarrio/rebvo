


#include "rebvo.h"

#include <TooN/so3.h>

#include <iostream>
#include <iomanip>
#include "ttimer.h"
#include "datasetcam.h"
#include <TooN/Cholesky.h>
#include <scaleestimator.h>





using namespace std;

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

    istate.RGBias=Identity*cf->GiroBiasStdDev*cf->GiroBiasStdDev;
    istate.RGiro=Identity*cf->GiroMeasStdDev*cf->GiroMeasStdDev;
    istate.W_Bg=util::Matrix3x3Inv(istate.RGBias*100);

    istate.Rg=cf->g_module_uncer*cf->g_module_uncer;
    istate.Rs=Identity*cf->AcelMeasStdDev*cf->AcelMeasStdDev;
    istate.Qbias=Identity*cf->VBiasStdDev*cf->VBiasStdDev;

    istate.X=makeVector(M_PI/4,0,cf->g_module,0,0,0,0);
    istate.P=makeVector(M_PI/16*M_PI/16*1e-4,\
                        100,100,100,\
                        cf->VBiasStdDev*cf->VBiasStdDev*1e1,cf->VBiasStdDev*cf->VBiasStdDev*1e1,cf->VBiasStdDev*cf->VBiasStdDev*1e1).as_diagonal();

    istate.u_est=makeVector(1,0,0);

    int n_giro_init=0;
    Vector <3> giro_init=Zeros;

    //**** Set cpu Afinity for this thread *****

    if(cf->cpuSetAffinity){
        cpu_set_t cpusetp;

        CPU_ZERO(&cpusetp);
        CPU_SET(cf->cpu1,&cpusetp);
        if(pthread_setaffinity_np(pthread_self(),sizeof(cpu_set_t),&cpusetp)!=0){
            printf("\nSecond Thread: No puedo setear CPU affinity!\n");
            cf->quit=true;
            return;
        }
    }


    //***** Launch Thread 2 Pipeline thread ******

    std::thread Thr2(ThirdThread,cf);

    COND_TIME_DEBUG(util::interval_list tlist;)

    COND_TIME_DEBUG(util::timer t_loop;)

    //****** Dummy processing of the first frame ******


    cf->pipe.RequestBuffer(1);
    cf->pipe.ReleaseBuffer(1);

    //****** Main loop *******

    while(!cf->quit){

        bool EstimationOk=true;

        COND_TIME_DEBUG(dtp=t_loop.stop();)

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
            dt_frame=1/cf->config_fps;

        COND_TIME_DEBUG(t_loop.start();)
        COND_TIME_DEBUG(tlist.clear();)

        int klm_num=0;

        COND_TIME_DEBUG(tlist.push_new();)
        //Estimate uncertainity cut-off for some quantile (90 percent typicaly)
        double s_rho_q=old_buf.ef->EstimateQuantile(RHO_MIN,RHO_MAX,cf->QCutOffQuantile,cf->QCutOffNumBins);


        COND_TIME_DEBUG(tlist.push_new();)

        //new_buf.gt->build_field(*new_buf.ef,cf->SearchRange);

        COND_TIME_DEBUG(tlist.push_new();)

        P_V=Identity*1e50;
        P_W=Identity*1e50;

        R=Identity;

        //***** Use the Tracker to obtain Velocity and Rotation  ****

        if(cf->ImuMode>0){      //Imu data avaiable?? Use it!

            if(!istate.init && cf->InitBias && n_frame>0){

                giro_init+=new_buf.imu.giro*new_buf.imu.dt;
                //giro_init+=istate.dWv;
                if(++n_giro_init>cf->InitBiasFrameNum){
                    istate.Bg=giro_init/n_giro_init;
                    istate.init=true;
                    istate.W_Bg=util::Matrix3x3Inv(istate.RGBias*1e2);
                }
            }


            R=new_buf.imu.Rot;  //Use imu rotation


            R.T()=TooN::SO3<>(istate.Bg)*R.T();

            old_buf.ef->rotate_keylines(R.T()); //Apply foward prerotation to the old key lines


            new_buf.gt->Minimizer_V<double>(istate.Vg,istate.P_Vg,*old_buf.ef,cf->TrackerMatchThresh,cf->TrackerIterNum,s_rho_q,cf->MatchNumThresh);   //Estimate translation only


            //***** Match from the old EdgeMap to the new one using the information from the minimization *****
            old_buf.ef->FordwardMatch(new_buf.ef);

            //***** Visual RotoTranslation estimation using forward matches
            Matrix <6,6> R_Xv,R_Xgv,W_Xv,W_Xgv;
            Vector <6> Xv,Xgv,Xgva;
            EstimationOk&=new_buf.ef->ExtRotVel(istate.Vg,W_Xv,R_Xv,Xv,cf->LocationUncertainty,cf->ReweigthDistance);


            istate.dVv=Xv.slice<0,3>();
            istate.dWv=Xv.slice<3,3>();

            Xgv=Xv;
            W_Xgv=W_Xv;

           // std::cout<<W_Xv<<"\n";

            Vector <3> dgbias=Zeros;
            edge_tracker::BiasCorrect(Xgv,W_Xgv,dgbias,istate.W_Bg,istate.RGiro,istate.RGBias);
            istate.Bg+=dgbias;

           // std::cout<<W_Xv<<W_Xgv<<istate.W_Bg;

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


            //Mix with accelerometer using bayesian filter


            ScaleEstimator::EstAcelLsq4(-istate.Vgv/dt_frame,istate.Av,R,dt_frame);
            ScaleEstimator::MeanAcel4(new_buf.imu.cacel,istate.As,R);

            Xgva=Xgv;


            istate.Rv=(P_V/(dt_frame*dt_frame*dt_frame*dt_frame));
            istate.Qrot=P_W;
            istate.QKp=std::min((1/(1/istate.Rv(0,0)+1/istate.Rv(1,1)+1/istate.Rv(2,2)))*cf->ScaleStdDevMult*cf->ScaleStdDevMult,cf->ScaleStdDevMax);


            if(n_frame>4+cf->InitBiasFrameNum){
                K=ScaleEstimator::estKaGMEKBias(istate.As,istate.Av,1,R,istate.X,istate.P,
                                                istate.Qrot,istate.Qbias,istate.QKp,istate.Rg,istate.Rs,istate.Rv,
                                                istate.g_est,istate.b_est,W_Xgv,Xgva,cf->g_module);

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


            //***** Forward Rotate the old edge-map points *****
            old_buf.ef->rotate_keylines(R0.get_matrix());


        }else{                  //Regular procesing

            new_buf.gt->Minimizer_RV<double>(V,W,P_V,P_W,*old_buf.ef,cf->TrackerMatchThresh,cf->TrackerIterNum,cf->TrackerInitType,cf->ReweigthDistance,error_vel,error_score,s_rho_q,cf->MatchNumThresh,cf->TrackerInitIterNum);

            //***** Match from the old EdgeMap to the new one using the information from the minimization *****
            old_buf.ef->FordwardMatch(new_buf.ef);


            //***** extract rotation matrix *****
            SO3 <>R0(W);                    //R0 is a forward rotation
            R.T()=R0.get_matrix()*R.T();    //R is a backward rotation


            //***** Forward Rotate the old edge-map points *****
            old_buf.ef->rotate_keylines(R0.get_matrix());

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



            COND_TIME_DEBUG(tlist.push_new();)

            COND_TIME_DEBUG(tlist.push_new();)

            //***** Match from the new EdgeMap to the old one searching on the stereo line *****

            //Because the old edge map mask is used, New keylines and Translation are back rotated
            klm_num=new_buf.ef->directed_matching(V,P_V,R,old_buf.ef,cf->MatchThreshModule,cf->MatchThreshAngle,cf->SearchRange,cf->LocationUncertaintyMatch);

            if(klm_num<cf->MatchThreshold){     //If matching keylines are below a certain threshold, reestart the estimation

                P_V=Identity*1e50;
                V=Zeros;

                Kp=1;
                P_Kp=10;

                EstimationOk=false;

                printf("\nCamara Frontal: restarting the estimation, match threshold low (%d,%d)?\n",new_buf.ef->KNum(),klm_num);
            }else{


                COND_TIME_DEBUG(tlist.push_new();)

                //****** Regularize the EdgeMap Depth ******

                for(int i=0;i<1;i++)
                        new_buf.ef->Regularize_1_iter(cf->RegularizeThresh);


                COND_TIME_DEBUG(tlist.push_new();)

                //****** Improve Depth using kalman filter ****

                new_buf.ef->UpdateInverseDepthKalman(V,P_V,P_W,cf->ReshapeQAbsolute,cf->ReshapeQRelative,cf->LocationUncertainty); //1e-5



                COND_TIME_DEBUG(tlist.push_new();)


                //****** Optionally reescale the EdgeMap's Depth
                Kp=new_buf.ef->EstimateReScaling(P_Kp,RHO_MAX,1,cf->DoReScaling>0);
               // cout << P_Kp<<"\n";


                COND_TIME_DEBUG(tlist.push_new();)


            }

        }



        COND_TIME_DEBUG(printf("\nr%f b%f v%f f%f w%f m%f r%f k%f t%f %f %d %f %s\n",tlist[0],tlist[1],tlist[2],tlist[3],\
               tlist[4],tlist[5],tlist[6],tlist[7],tlist.total(),dtp,new_buf.ef->KNum(),s_rho_q,EstimationOk?"OK":"NOK");)



        //Estimate position and pose incrementally

        if(cf->ImuMode>0){  //If imu present use the filtered version...


            if(n_frame>4+cf->InitBiasFrameNum){

                /*
                istate.u_est=Rgva.T()*istate.u_est;

                istate.u_est=istate.u_est-(istate.u_est*istate.g_est)/(istate.g_est*istate.g_est)*istate.g_est;
                TooN::normalize(istate.u_est);


                Matrix<3> PoseP1=TooN::SO3<>(istate.g_est,makeVector(0,1,0)).get_matrix();
                Matrix<3> PoseP2=TooN::SO3<>(PoseP1*istate.u_est,makeVector(1,0,0)).get_matrix();

                Pose=PoseP2*PoseP1;*/

                Pose=Pose*Rgva;
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

        old_buf.dt=dt_frame;
        old_buf.K=K;
        old_buf.Kp=Kp;
        old_buf.RKp=P_Kp;

        old_buf.Rot=R;
        old_buf.RotLie=SO3<>(R).ln();
        old_buf.Vel=-V*K/dt_frame;

        old_buf.Pose=Pose;
        old_buf.PoseLie=SO3<>(Pose).ln();
        old_buf.Pos=Pos;

        old_buf.s_rho_p=s_rho_q;

        old_buf.EstimationOK=EstimationOk;

        old_buf.imustate=istate;

      /*  if((old_buf.p_id%25)==0){
            cf->kf_list.push_back(keyframe(*old_buf.ef,*old_buf.gt,old_buf.t,old_buf.Rot,old_buf.RotLie,old_buf.Vel,old_buf.Pose,old_buf.PoseLie,old_buf.Pos));

            std::cout <<"\nadded keyframe\n";
        }*/

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

