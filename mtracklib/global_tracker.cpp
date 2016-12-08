#include "global_tracker.h"
#include <TooN/so3.h>
#include <TooN/SVD.h>
#include <TooN/Cholesky.h>


using namespace TooN;

global_tracker::global_tracker(cam_model &cam)
    :field(cam.sz),cam_mod(cam),FrameCount(0)
{

    Ne10::InitNe10();

}

global_tracker::global_tracker(const global_tracker&gt)
    :field(gt.field.Size()),cam_mod(gt.cam_mod),
      max_r(gt.max_r),klist_f(gt.klist_f),FrameCount(gt.FrameCount)
{
    field=gt.field;
}


global_tracker::~global_tracker()
{

}


//********************************************************************
// build_field(): Builds the auxiliary image used to quick match on
// the optimization
//********************************************************************

void global_tracker::build_field(edge_tracker &klist,int radius){

    max_r=radius;
    klist_f=&klist;

    for(int inx=0;inx<field.bSize();inx++){
        field[inx].ikl=-1;                  //Reset ikl on whole image
    }


    for(int ikl=0;ikl<klist.KNum();ikl++){

        KeyLine &kl=klist[ikl];             //Short cut for current KL

        for(int t=-radius;t<radius;t+=1){   //For a segment of +- radious

            int inx=field.GetIndexRC(kl.u_m.x*(float)t+kl.c_p.x,kl.u_m.y*(float)t+kl.c_p.y); //Get the linear index of the segment point

            if(inx<0)
                continue;   //Out of border

            int at=abs(t);                                  //Distance to the actual keyline
            if(field[inx].ikl>=0 && at>field[inx].dist)     //If alreafy a KL in the aux image, and the distance is less
                continue;                                   //Dont save anything

            field[inx].dist=at;                             //Save KL data in the aux
            field[inx].ikl=ikl;

        }

    }

}


//********************************************************************
// Test_f_k(): Quick test for 2 KLs to match
//********************************************************************

template <class T>
inline bool Test_f_k(const  Point2DF &f_m   //KL1 gradient m
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



//**********************************************************************
// Calc_f_J(): Search for KL match in axuiliary image and calc gradients
//**********************************************************************

template <class T>
inline T global_tracker::Calc_f_J(const int f_inx,      //Field image index where to look
                                  T &df_dx,T &df_dy,    //Gradient vector
                                  KeyLine &kl,          //KeyLine gradient
                                  const Point2D<T> &p,  //Transformed KL coordinate in homogeneous
                                  const T &max_r,       //Maximun allowed radius
                                  const T &simil_t,     //Match threshold
                                  int &mnum,            //Match counter
                                  T &fi)                //Pixel residual
{


    if(field[f_inx].ikl<0){                 //Check for KL presence
        df_dx=0;
        df_dy=0;
        return max_r/kl.s_rho;              //Return max posible error (weighted)
    }

    KeyLine &f_kl=(*klist_f)[field[f_inx].ikl]; //Field keyline

    if(!Test_f_k<T>(f_kl.m_m,kl,simil_t)){
        df_dx=0;
        df_dy=0;
        return max_r/kl.s_rho;          //Return max posible error (weighted)
    }


    T dx=p.x-f_kl.c_p.x;
    T dy=p.y-f_kl.c_p.y;

    fi=(dx*f_kl.u_m.x+dy*f_kl.u_m.y);   //Residual in the direction of the gradient


    df_dx=f_kl.u_m.x/kl.s_rho;          //Return weigthed gradient
    df_dy=f_kl.u_m.y/kl.s_rho;

    mnum++;                             //Count match

    kl.m_id_f=field[f_inx].ikl;         //Forward match

    return fi/kl.s_rho;                 //weigthed residual

}



//*******************************************************************************************
// Calc_f_J(): Search for KL match in axuiliary image and calc gradients, dont scale by s_rho
//*******************************************************************************************

template <class T>
inline T global_tracker::Calc_f_J2(const int f_inx,      //Field image index where to look
                                  T &df_dx,T &df_dy,    //Gradient vector
                                  KeyLine &kl,          //KeyLine gradient
                                  const Point2D<T> &p,  //Transformed KL coordinate in homogeneous
                                  const T &max_r,       //Maximun allowed radius
                                  const T &simil_t,     //Match threshold
                                  int &mnum,            //Match counter
                                  T &fi)                //Pixel residual
{


    if(field[f_inx].ikl<0){                 //Check for KL presence
        df_dx=0;
        df_dy=0;
        return max_r;              //Return max posible error (weighted)
    }

    KeyLine &f_kl=(*klist_f)[field[f_inx].ikl]; //Field keyline

    if(!Test_f_k<T>(f_kl.m_m,kl,simil_t)){
        df_dx=0;
        df_dy=0;
        return max_r;          //Return max posible error (weighted)
    }


    T dx=p.x-f_kl.c_p.x;
    T dy=p.y-f_kl.c_p.y;

    fi=(dx*f_kl.u_m.x+dy*f_kl.u_m.y);   //Residual in the direction of the gradient


    df_dx=f_kl.u_m.x;          //Return weigthed gradient
    df_dy=f_kl.u_m.y;

    mnum++;                             //Count match

    kl.m_id_f=field[f_inx].ikl;         //Forward match

    return fi;                 //weigthed residual

}


//**************************************************************************
// TryVelRot(): Search for KL match in axuiliary image and calc gradients
//
// ******** IMPORTANT  NOTE about NE10:  *****************************************
// for more efficient processeing, this function uses
// NE10 Wrapper. This library requires the vectors to be aligned in
// the following format: {x1 x2 x3 .... xn y1 y2 y3 ... yn z1 z2 z3 ... zn}
// this kind of vectors will be caled "linear transpose coodinates vectors (LTCV)"
//*********************************************************************************


template <class T,          //Presition used for calculations, NEON only suports float, double is better...
          bool ReWeight,    //ReWeigthing compile time switch
          bool ProcJF,      //Calculate Jacobians or just energy score?
          bool UsePriors>   //Switch to use priors
double global_tracker::TryVelRot(TooN::Matrix<6,6,T> &JtJ,          //Estimated Matrixes J=Jacobian Matrix
                                 TooN::Vector<6,T> &JtF,            //F=Residual matrix
                                 const TooN::Vector<6,T> &VelRot,   //Proposed state vector  [Transtalation Rotation]
                                 const TooN::Vector<3> &V0p,        //Translation Model
                                 const TooN::Matrix<3,3> &PV0,      //Model uncertainty
                                 const TooN::Vector<3> &W0p,        //Rotation Model
                                 const TooN::Matrix<3,3> &PW0,      //Model uncertainty
                                 edge_tracker &klist,               //KL list for matching
                                 T *P0m,                            //linear transpose coodinates vector of 3D klist positions
                                 int pnum,                          //Number of points (floured to a multiple of four)
                                 double match_thresh,               //...
                                 double s_rho_min,                  //Threshold on IDepth uncertainty
                                 uint MatchNumThresh,               //Threshold on the matching history of the keyline
                                 T k_huber,                         //ReWeigth distance
                                 T* DResidual,                      //Last iteration Distance Residuals
                                 T* DResidualNew)                   //New iteration Distance Residuals
{

    double score=0; //Total error energy

    SO3 <> RotW0(VelRot.template slice<3,3>());
    const Matrix <3,3,T> &R0=RotW0.get_matrix();    //State Rotation matrix


    SO3 <> RotM(makeVector(0,0,VelRot[2]));                         //Rotation about Z-Axis (for gradient direction)
    const Matrix <2,2,T> RM=RotM.get_matrix().slice<0,0,2,2>();     //Get the 2D rotation...


    int mnum=0;                 //

    T Ptm[pnum*3];              //LTCV of transformed 3d coordinates
    T PtIm[pnum*3];             //LTCV of transformed image coordinates (z-reprensents IDepth)

    T df_dPi_x[pnum];           //weigthed x and y image derivative of residual
    T df_dPi_y[pnum];           //with respect to image coodinates
    T fm[pnum];                 //weighted residuals


    //Convert rotation  and translation format for SE3on3PMatrix
    T Rt[9];
    T Vt[3];

    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++)
            Rt[Index2Dto1D<3>(j,i)]=R0(i,j);
        Vt[i]=VelRot[i];
    }

    //Perform SE3 using NE10 wrapper
    Ne10::SE3on3PMatrix<T>(Rt,Vt,Ptm,P0m,pnum);

    //Proyect using NE10 wrapper
    Ne10::ProyP3toI3PMatrix<T>(PtIm,Ptm,cam_mod.zfm,pnum);


    T fi=0;

    //For each keyline in klist (old edgemap)...
    for(int ikl=0;ikl<klist.KNum();ikl++){

        KeyLine &kl=klist[ikl];

        kl.m_id_f=-1;               //Reset forward match

        if(kl.s_rho>s_rho_min || kl.m_num<std::min(MatchNumThresh,FrameCount)){     //If uncertainty is to hi, dont use this keyline
            fm[ikl]=0;                                                              //Also apply a thresh on the number of frames
            df_dPi_x[ikl]=0;                                                        //that this keyline has been matched for
            df_dPi_y[ikl]=0;
            continue;
        }


        Point2D<T> p_pji=cam_mod.Hom2Img(Point2D<T>(PtIm[ikl],PtIm[pnum+ikl])); //Conver to image coordinates (just add principal point)


        int x=util::round2int_positive(p_pji.x);    //Round to integer image coordinates
        int y=util::round2int_positive(p_pji.y);

        T weigth=1;                                     //Estimate reweighting
        if(ReWeight && fabs(DResidual[ikl])>k_huber)
            weigth=k_huber/fabs(DResidual[ikl]);



        if(x<1|| y<1|| x>=(int)cam_mod.sz.w-1 || y>=(int)cam_mod.sz.h-1){ //If tranformed position is outside image border
            fm[ikl]=max_r;/*/kl.s_rho*/;                                       //consider it a missmatch with maximun possible residual
            if(ReWeight)
                fm[ikl]*=weigth;
            df_dPi_x[ikl]=0;
            df_dPi_y[ikl]=0;
            DResidualNew[ikl]=max_r;
            continue;
        }

        Point2DF kl_m=kl.m_m;
        kl.m_m.x=RM(0,0)*kl_m.x+RM(0,1)*kl_m.y;
        kl.m_m.y=RM(1,0)*kl_m.x+RM(1,1)*kl_m.y;     //Temporaly rotate keylines gradient on z-axis for improved matching

        //Match and Calculate residuals and gradients
        fm[ikl]=Calc_f_J2<T>(y*cam_mod.sz.w+x,df_dPi_x[ikl],df_dPi_y[ikl],kl,p_pji,max_r,match_thresh,mnum,fi);

        kl.m_m=kl_m;        //Return gradient to it's original state

        //Optionally appy reweigting
        if(ReWeight){
            fm[ikl]*=weigth;
            df_dPi_x[ikl]*=weigth;
            df_dPi_y[ikl]*=weigth;
        }

        //Save pixel residual for this iteration
        DResidualNew[ikl]=fi;



    }


    for(int ikl=0;ikl<klist.KNum();ikl++){

    }

    if(ProcJF){         //Estimate Jacobians using NE10Wrapper

        T Jm[pnum*6];   //LTCV Jacobian with respect to the state


        T RhoTmp0[pnum];

        Ne10::MulCVect<T>(RhoTmp0,cam_mod.zfm,&PtIm[pnum*2],pnum);      //RhoTmp0 = Rho_p*zf

        Ne10::MulVect<T>(&Jm[pnum*0],RhoTmp0,df_dPi_x,pnum);     //Jx=Rho_p*zf*df_Pix
        Ne10::MulVect<T>(&Jm[pnum*1],RhoTmp0,df_dPi_y,pnum);     //Jy=Rho_p*zf*df_Piy

        Ne10::MulVect<T>(RhoTmp0,&PtIm[pnum*2],&PtIm[pnum*0],pnum);     //RhoTmp0=Rho_p * PtIx
        Ne10::MulVect<T>(&Jm[pnum*2],RhoTmp0,df_dPi_x,pnum);     //Jz=Rho_p * PtIx*sf_Pix

        Ne10::MulVect<T>(RhoTmp0,&PtIm[pnum*2],&PtIm[pnum*1],pnum);     //RhoTmp0=Rho_p * PtIy
        Ne10::MlAcVect<T>(&Jm[pnum*2],RhoTmp0,df_dPi_y,pnum);    //Jz+=Rho_p*PtIy*df_Piy queda invertido en signo

    //Matrix <2,3,T> dPi_dp;
    //dPi_dp[0]=makeVector(cam_mod.zfm*rho_p,0,-p_pj.x*rho_p);
    //dPi_dp[1]=makeVector(0,cam_mod.zfm*rho_p,-p_pj.y*rho_p);

        Ne10::MulVect<T>(&Jm[pnum*3],&Jm[pnum*1],&Ptm[pnum*2],pnum);
        Ne10::MlAcVect<T>(&Jm[pnum*3],&Jm[pnum*2],&Ptm[pnum*1],pnum);      //Jwx queda invertido en signo

        Ne10::MulVect<T>(&Jm[pnum*4],&Jm[pnum*0],&Ptm[pnum*2],pnum);
        Ne10::MlAcVect<T>(&Jm[pnum*4],&Jm[pnum*2],&Ptm[pnum*0],pnum);      //Jwy queda bien en signo

        Ne10::MulVect<T>(RhoTmp0,&Jm[pnum*0],&Ptm[pnum*1],pnum);
        Ne10::MulCVect<T>(&Jm[pnum*5],-1,RhoTmp0,pnum);
        Ne10::MlAcVect<T>(&Jm[pnum*5],&Jm[pnum*1],&Ptm[pnum*0],pnum);      //Jwz queda bien en signo


        for(int ikl=0;ikl<klist.KNum();ikl++){

            double qvel=(cam_mod.zfm*df_dPi_x[ikl]*Vt[0]+cam_mod.zfm*df_dPi_y[ikl]*Vt[1]+\
                    (PtIm[pnum*0+ikl]*df_dPi_x[ikl]+PtIm[pnum*1+ikl]*df_dPi_y[ikl])*Vt[2]);
            double q_rho=sqrt(klist[ikl].s_rho*qvel*klist[ikl].s_rho*qvel+1);
            if(!ReWeight)
                q_rho=klist[ikl].s_rho;
            for(int j=0;j<6;j++)
                Jm[pnum*j+ikl]/=q_rho;
            fm[ikl]/=q_rho;

        }

    //dRp0_dw[0]=makeVector(   0  , P0[2] ,-P0[1]);
    //dRp0_dw[1]=makeVector(-P0[2],   0   , P0[0]);
    //dRp0_dw[2]=makeVector( P0[1],-P0[0],   0  );


        for(int ikl=klist.KNum();ikl<pnum;ikl++){           //pnnum has to be a multiple of 4, make 0
            for(int i=0;i<6;i++){                           //the points between knum and pnum
                Jm[pnum*i+ikl]=0;
            }
            fm[ikl]=0;
        }

        for(int i=0;i<6;i++){                               //Fast dot product
            for(int j=i;j<6;j++){
                JtJ(i,j)=Ne10::DotProduct(&Jm[pnum*i],&Jm[pnum*j],pnum);//Jf.T()[i]*Jf.T()[j];
            }
            JtF[i]=Ne10::DotProduct<T>(&Jm[pnum*i],fm,pnum);
        }

        for(int i=0;i<2;i++){                               //Adjust for signs wrongly estimated
            JtF[i+2]=-JtF[i+2];                             //in the jacobians
            for(int j=0;j<2;j++){
                JtJ(i+0,j+2)=-JtJ(i+0,j+2);
                JtJ(i+2,j+4)=-JtJ(i+2,j+4);
            }
        }

        for(int i=0;i<6;i++){                               //Symetrize the matrix
            for(int j=i+1;j<6;j++){
                JtJ(j,i)=JtJ(i,j);
            }
        }

    }else{

        for(int ikl=0;ikl<klist.KNum();ikl++){

            double qvel=(cam_mod.zfm*df_dPi_x[ikl]*Vt[0]+cam_mod.zfm*df_dPi_y[ikl]*Vt[1]+\
                    (PtIm[pnum*0+ikl]*df_dPi_x[ikl]+PtIm[pnum*1+ikl]*df_dPi_y[ikl])*Vt[2]);
            double q_rho=sqrt(klist[ikl].s_rho*qvel*klist[ikl].s_rho*qvel+1);
            if(!ReWeight)
                q_rho=klist[ikl].s_rho;
            fm[ikl]/=q_rho;
        }

        for(int ikl=klist.KNum();ikl<pnum;ikl++){
            fm[ikl]=0;
        }
    }


    score=Ne10::DotProduct(fm,fm,pnum);                     //The score is the dot product of the residual

    if(UsePriors){

        //If using priors, mix them with the model

        Matrix <3,3,T> YW0 = util::Matrix3x3Inv(PW0);
        Matrix <3,3,T> YV0 = util::Matrix3x3Inv(PV0);


        if(ProcJF){

            JtJ.template slice<3,3,3,3>()+=YW0;
            JtF.template slice<3,3>()+=YW0*(VelRot.template slice<3,3>()-W0p);

            JtJ.template slice<0,0,3,3>()+=YV0;
            JtF.template slice<0,3>()+=YV0*(VelRot.template slice<0,3>()-V0p);

        }

        score+=(VelRot.template slice<3,3>()-W0p)*YW0*(VelRot.template slice<3,3>()-W0p);
        score+=(VelRot.template slice<0,3>()-V0p)*YV0*(VelRot.template slice<0,3>()-V0p);

    }

    return score;

}


//********************************************************************************
// KltoI3PMatrix(): convert KL coordinates to LTCV format for use with NE10Wrapper
//
// *******************************************************************************


template <class T>
void KltoI3PMatrix(edge_tracker & klist,    //
                   const int pnum,          //Number of klist points, rounded up to a multiple of 4
                   T* Pos)
{

    int ikl=0,ikly=pnum,iklr=2*pnum;
    for(;ikl<klist.KNum();ikl++,ikly++,iklr++){
        Pos[ikl]=klist[ikl].p_m.x;
        Pos[ikly]=klist[ikl].p_m.y;
        Pos[iklr]=klist[ikl].rho;
    }

    for(;ikl<pnum;ikl++,ikly++,iklr++){
        Pos[ikl]=0;
        Pos[ikly]=0;
        Pos[iklr]=1;
    }
}

//********************************************************************************
// Optimization on the velocity and rotation using lenveng-marquart
//
// *******************************************************************************


template <class T,          //Presition, NEON only supperots float
          bool UsePriors>   //Use priors switch
double global_tracker::Minimizer_RV(TooN::Vector<3> &Vel,       //Initial estimate on the velocity (translation)
                                    TooN::Vector<3> &W0,        //Initial estimate on the rotation
                                    TooN::Matrix<3,3> &RVel,    //Uncertainty Model if the initial estimate will
                                    TooN::Matrix<3,3> &RW0,     //be used as prior
                                    edge_tracker &klist,        //KList to match to
                                    double match_thresh,        //Match Gradient Treshold
                                    int iter_max,               //Iteration number
                                    int init_type,              //Initialization type
                                    double reweigth_distance,   //Distance cut-off for reweigthing
                                    double &rel_error,          //Estimated relative error on the state
                                    double &rel_error_score,    //Estimated relative error on the score
                                    const double &max_s_rho,    //Threshold on the IDepth uncertainty
                                    const uint& MatchNumThresh, //Threshold on the KL match number
                                    const double& init_iter){   //Number of iterations for double init



    if(klist.KNum()<=0)
        return 0;


    TooN::Matrix<6,6,T> JtJ,ApI,JtJnew;
    TooN::Vector<6,T> JtF,JtFnew;
    TooN::Vector<6,T> h,Xnew,X,Xt;



    int pnum=(klist.KNum()+0x3)&(~0x3);     //Number of points up-rounded to a multiple of 4 (NE10Wrapper requirment)
    T P0Im[pnum*3];                         //Image coordinates (x,y,rho)
    T P0m[pnum*3];                          //3Dcoordinates
    T Res0[pnum], Res1[pnum],Rest[pnum];    //DIstance Residuals
    T *Residual=Res0,*ResidualNew=Res1;     //

    //Convert to LTCV format
    KltoI3PMatrix<T>(klist,pnum,P0Im);
    //Proyect to 3D using NE10Wrapper
    Ne10::ProyI3Pto3PMatrix<T>(P0m,P0Im,cam_mod.zfm,pnum);


    double F,Fnew,F0;           //Total energy score
    double v=2,tau=1e-3;        //LM params
    double u,gain;
    int eff_steps=0;

    for(int i=0;i<pnum;i++)     //Init residuals
        Residual[i]=0;

    double k_hubber=reweigth_distance;  //Reweigth distance


    switch(init_type){
    case 0:             //Zero initial condition
        X=Zeros;


        break;
    case 1:

        X.template slice<0,3>()=Vel;    //Prior initial condition
        X.template slice<3,3>()=W0;     //

        break;

    case 2:
    default:

        //Double initialition: tryes a few iteration to see what is best, more accurate but slower

        //**** Try Zero initial condition
        X=Zeros;
        F=TryVelRot<T,false,true,UsePriors>(JtJ,JtF,X,Vel,RVel,W0,RW0,klist,P0m,pnum,match_thresh,max_s_rho,MatchNumThresh,k_hubber,Residual,Rest);
        F0=F;
        u=tau*TooN::max_element(JtJ).first;

        //Iterate init_iter times NOT using reweigting (more robust)

        for(int i=0;i<init_iter;i++){

            ApI=JtJ+Identity*u;
            SVD <>svdApI(ApI);
            h=svdApI.backsub(-JtF);

            Xnew=X+h;

            if(i==init_iter-1){     //On the last iteration dont calculate Jacs (saves time)
                Fnew=TryVelRot<T,false,false,UsePriors>(JtJnew,JtFnew,Xnew,Vel,RVel,W0,RW0,klist, P0m,pnum,match_thresh,max_s_rho,MatchNumThresh,k_hubber,Residual,Rest);
                gain=(F-Fnew);
            }else{
                Fnew=TryVelRot<T,false,true,UsePriors>(JtJnew,JtFnew,Xnew,Vel,RVel,W0,RW0,klist, P0m,pnum,match_thresh,max_s_rho,MatchNumThresh,k_hubber,Residual,Rest);
                gain=(F-Fnew)/(0.5*h*(u*h-JtF));
            }



            if(gain>0){     //LM update
                F=Fnew;
                X=Xnew;
                JtJ=JtJnew;
                JtF=JtFnew;
                u*=std::max(0.33,1-((2*gain-1)*(2*gain-1)*(2*gain-1)));
                v=2;
                eff_steps++;
            }else{
                u*=v;
                v*=2;
            }
        }

        //Save the scores in temporals
        Xt=X;
        double Ft=F;
        double F0t=F0;
        double ut=u,vt=v,eff_steps_t=eff_steps;

        eff_steps=0;


        //****** Then try with prior init
        X.template slice<0,3>()=Vel;
        X.template slice<3,3>()=W0;

        F=TryVelRot<T,false,true,UsePriors>(JtJ,JtF,X,Vel,RVel,W0,RW0,klist, P0m,pnum,match_thresh,max_s_rho,MatchNumThresh,k_hubber,Residual,ResidualNew);

        F0=F;
        u=tau*TooN::max_element(JtJ).first;
        v=2;

        for(int i=0;i<init_iter;i++){

            ApI=JtJ+Identity*u;
            SVD <>svdApI(ApI);
            h=svdApI.backsub(-JtF);

            Xnew=X+h;
            if(i==init_iter-1){
                Fnew=TryVelRot<T,false,false,UsePriors>(JtJnew,JtFnew,Xnew,Vel,RVel,W0,RW0,klist, P0m,pnum,match_thresh,max_s_rho,MatchNumThresh,k_hubber,Residual,ResidualNew);
                gain=(F-Fnew);
            }else{
                Fnew=TryVelRot<T,false,true,UsePriors>(JtJnew,JtFnew,Xnew,Vel,RVel,W0,RW0,klist, P0m,pnum,match_thresh,max_s_rho,MatchNumThresh,k_hubber,Residual,ResidualNew);
                gain=(F-Fnew)/(0.5*h*(u*h-JtF));
            }

            if(gain>0){
                F=Fnew;
                X=Xnew;
                JtJ=JtJnew;
                JtF=JtFnew;
                u*=std::max(0.33,1-((2*gain-1)*(2*gain-1)*(2*gain-1)));
                v=2;
                eff_steps++;
            }else{
                u*=v;
                v*=2;
            }
        }

        if(F>Ft){       //Check for the the lowerst score, and use it!
            X=Xt;       //If necesary retreive temporals
            F=Ft;
            F0=F0t;
            u=ut;
            v=vt;
            eff_steps=eff_steps_t;
            ResidualNew=Rest;

        }


        std::swap(ResidualNew,Residual);    //Swap to use residuals

        break;
    }


    //Start or continue iterating, now using reweigth
    F0=F=TryVelRot<T,true,true,UsePriors>(JtJ,JtF,X,Vel,RVel,W0,RW0,klist, P0m,pnum,match_thresh,max_s_rho,MatchNumThresh,k_hubber,Residual,ResidualNew);
    u=tau*TooN::max_element(JtJ).first; //Levenberg-Marquart param reset
    v=2;

    int lm_iter=0;
    for(;lm_iter<iter_max;lm_iter++){

        ApI=JtJ+Identity*u;

        //Solve ApI*h=-JtF

        Cholesky <6>svdApI(ApI);    //ApI is symetric, use cholesky
        h=svdApI.backsub(-JtF);


        Xnew=X+h;
        Fnew=TryVelRot<T,true,true,UsePriors>(JtJnew,JtFnew,Xnew,Vel,RVel,W0,RW0,klist, P0m,pnum,match_thresh,max_s_rho,MatchNumThresh,k_hubber,Residual,ResidualNew);


        gain=(F-Fnew)/(0.5*h*(u*h-JtF));    //Check gain

        if(gain>0){ //if positive, update step!
            F=Fnew;
            X=Xnew;
            JtJ=JtJnew;
            JtF=JtFnew;
            u*=std::max(0.33,1-((2*gain-1)*(2*gain-1)*(2*gain-1)));
            v=2;
            eff_steps++;
            std::swap(ResidualNew,Residual);        //Swap residuals

        }else{      //if not, go to Gradient Descent
            u*=v;
            v*=2;
        }



    }

    Cholesky <6>svdJtJ(JtJ);
    Matrix <6,6>RRV=svdJtJ.get_inverse();  //Extract linealized uncertaintyes

    Vel=X.template slice<0,3>();           //Return the estimation
    W0=X.template slice<3,3>();

    RVel=RRV.slice<0,0,3,3>();             //And the uncertaintyes
    RW0=RRV.slice<3,3,3,3>();


    if(eff_steps>0){
        rel_error=norm(h)/(norm(X)+1e-30);  //Relative step error
        rel_error_score=F/F0;               //Relative score error
    }else{

        rel_error=1e20;
        rel_error_score=1e20;
    }

    FrameCount++;
    return F;

}




template double global_tracker::Minimizer_RV<float,false>(TooN::Vector<3> &Vel,TooN::Vector<3> &W0,TooN::Matrix<3,3> &RVel,TooN::Matrix<3,3> &RW0, edge_tracker &klist,double match_thresh,int iter_max,int init_type,double reweigth_distance,double &rel_error,double& rel_error_score,const double &max_s_rho,const uint& MatchNumThresh,const double& init_iter);
template double global_tracker::Minimizer_RV<double,false>(TooN::Vector<3> &Vel,TooN::Vector<3> &W0,TooN::Matrix<3,3> &RVel,TooN::Matrix<3,3> &RW0, edge_tracker &klist,double match_thresh,int iter_max,int init_type,double reweigth_distance,double &rel_error,double &rel_error_score,const double &max_s_rho,const uint& MatchNumThresh,const double& init_iter);



template <class T>
double global_tracker::TryVel(TooN::Matrix<3,3> &JtJ,TooN::Vector<3> &JtF, const TooN::Vector<3> &Vel, edge_tracker &klist,double match_thresh, double s_rho_min,uint MatchNumThresh,T *Residuals,double reweigth_distance)
{

    double score=0,f;

    JtJ=Zeros;
    JtF=Zeros;

    int mnum=0;
    double fi=0;



    for(int ikl=0;ikl<klist.KNum();ikl++){

        KeyLine &kl=klist[ikl];


        kl.m_id_f=-1;

        if(kl.s_rho>s_rho_min || kl.m_num<std::min(MatchNumThresh,FrameCount)){     //If uncertainty is to hi, dont use this keyline
                                                                                     //Also apply a thresh on the number of frames
                                                                                    //that this keyline has been matched for
            continue;
        }


        Point2D<double> p_pj;

        double weight=1;
        if(Residuals[ikl]>reweigth_distance)
            weight=reweigth_distance/Residuals[ikl];



        double z_p=1.0/kl.rho+Vel[2];
        if(z_p<=0){
            f=(1/(kl.s_rho))*max_r*weight;
            score+=f*f;
            continue;
        }
        double rho_p=1.0/z_p;

        p_pj.x=rho_p*(Vel[0]*cam_mod.zfm-Vel[2]*kl.p_m.x)+kl.p_m.x;
        p_pj.y=rho_p*(Vel[1]*cam_mod.zfm-Vel[2]*kl.p_m.y)+kl.p_m.y;

        Point2D<double> p_pji=cam_mod.Hom2Img(p_pj);


        int x=util::round2int_positive(p_pji.x);
        int y=util::round2int_positive(p_pji.y);

        if(x<1|| y<1|| x>=(int)cam_mod.sz.w-1 || y>=(int)cam_mod.sz.h-1){
            f=(1/(kl.s_rho))*max_r*weight;
            score+=f*f;
            continue;
        }


        double df_dx;
        double df_dy;

        f=Calc_f_J<double>(y*cam_mod.sz.w+x,df_dx,df_dy,kl,p_pji,max_r,match_thresh,mnum,fi);

        f*=weight;
        score+=f*f;

        double jx=rho_p*cam_mod.zfm*df_dx*weight;
        double jy=rho_p*cam_mod.zfm*df_dy*weight;
        double jz=-rho_p*(p_pj.x*df_dx+p_pj.y*df_dy)*weight;


        JtJ(0,0)+=jx*jx;
        JtJ(1,1)+=jy*jy;
        JtJ(2,2)+=jz*jz;
        JtJ(0,1)+=jx*jy;
        JtJ(0,2)+=jx*jz;
        JtJ(1,2)+=jy*jz;

        JtF[0]+=jx*f;
        JtF[1]+=jy*f;
        JtF[2]+=jz*f;

        Residuals[ikl]=fabs(fi);

        //step=1+(rand()&0x03);

    }


    JtJ(1,0)=JtJ(0,1);
    JtJ(2,0)=JtJ(0,2);
    JtJ(2,1)=JtJ(1,2);

    // printf("\nMNum: %d, %d, %f\n",mnum,mnum2,score);

    // std::cout<<JtJ <<"\n"<<JtF <<"\n";

    return score;

}


template <class T>
double global_tracker::TryVel_vect(TooN::Matrix<3,3> &JtJ, TooN::Vector<3> &JtF, const TooN::Vector<3> &Vel, edge_tracker &klist, T match_thresh, double s_rho_min,uint MatchNumThresh,double reweigth_distance){

    int pnum=(klist.KNum()+0x3)&(~0x3);
    T Jx[pnum];
    T Jy[pnum];
    T Jz[pnum];
    T f[pnum];


    int mnum=0;

    int step=1;

    int ip=0;
    for(int ikl=0;ikl<klist.KNum();ikl+=step,ip++){

        //step=1+(rand()&0x03);

        KeyLine &kl=klist[ikl];

        kl.m_id_f=-1;

        Point2D<T>  p_pj;

        double z_p=1.0/kl.rho+Vel[2];
        if(z_p<=0)
            continue;
        double rho_p=1.0/z_p;

        p_pj.x=rho_p*(Vel[0]*cam_mod.zfm-Vel[2]*kl.p_m.x)+kl.p_m.x;
        p_pj.y=rho_p*(Vel[1]*cam_mod.zfm-Vel[2]*kl.p_m.y)+kl.p_m.y;

        Point2D<T>  p_pji=cam_mod.Hom2Img(p_pj);


        int x=util::round2int_positive(p_pji.x);
        int y=util::round2int_positive(p_pji.y);

        if(x<1|| y<1|| x>=(int)cam_mod.sz.w-1 || y>=(int)cam_mod.sz.h-1){
            f[ip]=(1/(kl.s_rho))*max_r;
            Jx[ip]=0;
            Jy[ip]=0;
            Jz[ip]=0;
            continue;
        }


        T df_dx;
        T df_dy;
        T fi;

        f[ip]=Calc_f_J<T>(y*cam_mod.sz.w+x,df_dx,df_dy,kl,p_pji,max_r,match_thresh,mnum,fi);



        Jx[ip]=rho_p*cam_mod.zfm*df_dx;
        Jy[ip]=rho_p*cam_mod.zfm*df_dy;
        Jz[ip]=-rho_p*(p_pj.x*df_dx+p_pj.y*df_dy);





    }


    pnum=(ip+0x3)&(~0x3);

    for(int i=ip;i<pnum;i++){
        Jx[i]=0;
        Jy[i]=0;
        Jz[i]=0;
        f[i]=0;
    }

    JtJ(0,0)=Ne10::DotProduct(Jx,Jx,pnum);
    JtJ(1,1)=Ne10::DotProduct(Jy,Jy,pnum);
    JtJ(2,2)=Ne10::DotProduct(Jz,Jz,pnum);
    JtJ(0,1)=Ne10::DotProduct(Jx,Jy,pnum);
    JtJ(0,2)=Ne10::DotProduct(Jx,Jz,pnum);
    JtJ(1,2)=Ne10::DotProduct(Jy,Jz,pnum);

    JtF[0]=Ne10::DotProduct(Jx,f,pnum);
    JtF[1]=Ne10::DotProduct(Jy,f,pnum);
    JtF[2]=Ne10::DotProduct(Jz,f,pnum);

    JtJ(1,0)=JtJ(0,1);
    JtJ(2,0)=JtJ(0,2);
    JtJ(2,1)=JtJ(1,2);

    // printf("\nMNum: %d, %d, %f\n",mnum,mnum2,score);

    // std::cout<<JtJ <<"\n"<<JtF <<"\n";

    return Ne10::DotProduct(f,f,pnum);

}

template <class T>
double global_tracker::Minimizer_V(TooN::Vector<3> &Vel, TooN::Matrix<3,3> &RVel, edge_tracker &klist,  T match_thresh, int iter_max, T s_rho_min,uint MatchNumThresh,double reweigth_distance){

    TooN::Matrix<3,3> JtJ,ApI,JtJnew;
    TooN::Vector<3> JtF,JtFnew;
    TooN::Vector<3> h,Vnew;

    T residuals[klist.KNum()];
    for(int i=0;i<klist.KNum();i++)
        residuals[i]=0;

    double F=TryVel(JtJ,JtF,Vel,klist,match_thresh,s_rho_min,MatchNumThresh,residuals,reweigth_distance),Fnew;

    double v=2,tau=1e-3;
    double u=tau*TooN::max_element(JtJ).first;
    double gain;



    for(int lm_iter=0;lm_iter<iter_max;lm_iter++){

        ApI=JtJ+Identity*u;

        //Solve ApI*h=-g

        h=util::Matrix3x3Inv(ApI)*(-JtF);

        //Check step criteria

         Vnew=Vel+h;
        //Fnew=TryVel_vect<float>(JtJnew,JtFnew,Vnew,klist, knum,use_a);
        Fnew=TryVel(JtJnew,JtFnew,Vnew,klist,match_thresh,s_rho_min,MatchNumThresh,residuals,reweigth_distance);

        gain=(F-Fnew)/(0.5*h*(u*h-JtF));

        if(gain>0){
            F=Fnew;
            Vel=Vnew;
            JtJ=JtJnew;
            JtF=JtFnew;
            u*=std::max(0.33,1-((2*gain-1)*(2*gain-1)*(2*gain-1)));
            v=2;
        }else{
            u*=v;
            v*=2;
        }



    }

    RVel=util::Matrix3x3Inv(JtJ);


    return F;

}

template  double global_tracker::Minimizer_V<double>(TooN::Vector<3> &Vel, TooN::Matrix<3,3> &RVel, edge_tracker &klist,  double match_thresh, int iter_max, double s_rho_min,uint MatchNumThresh,double reweigth_distance);
template  double global_tracker::Minimizer_V<float>(TooN::Vector<3> &Vel, TooN::Matrix<3,3> &RVel, edge_tracker &klist,  float match_thresh, int iter_max, float s_rho_min,uint MatchNumThresh,double reweigth_distance);
