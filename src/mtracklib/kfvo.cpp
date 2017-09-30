#include "mtracklib/kfvo.h"
#include "TooN/so3.h"
#include <TooN/SVD.h>
#include <TooN/Cholesky.h>
namespace rebvo{
using namespace TooN;
kfvo::kfvo()
{
}
void kfvo::countMatches(keyframe &kf_to, keyframe &kf_from,int &kl_on_fov,int &kl_match,float match_mod,float match_ang,float rho_tol,float max_r){

    kl_on_fov=0;
    kl_match=0;

    cam_model &cam=kf_to.camera;
    for(KeyLine &kl:kf_from.edges()){

        Vector<3> p_to=kf_from.reProjectTo(kf_to,makeVector(kl.p_m.x,kl.p_m.y,kl.rho));

        Point2D<float> p_img=cam.Hom2Img(Point2D<float>(p_to[0],p_to[1]));

        int x=util::round2int_positive(p_img.x);    //Round to integer image coordinates
        int y=util::round2int_positive(p_img.y);

        if(p_img.x<1|| p_img.y<1|| p_img.x>=(int)cam.sz.w-1 || p_img.y>=(int)cam.sz.h-1 || p_to[2]<=0){ //If tranformed position is outside image border
            continue;
        }

        //     Point2DF kl_m=kl.m_m;
        //        kl.m_m.x=RM(0,0)*kl_m.x+RM(0,1)*kl_m.y;
        //       kl.m_m.y=RM(1,0)*kl_m.x+RM(1,1)*kl_m.y;     //Temporaly rotate keylines gradient on z-axis for improved matching

        //Match and Calculate residuals and gradients

        float df_dPi_x,df_dPi_y;
        int mnum=0;
        float fi=kf_to.tracker().Calc_f_J_Complete<float>(y*cam.sz.w+x,df_dPi_x,df_dPi_y,kl,p_img,max_r,match_mod,cos(match_ang),mnum,p_to[2],rho_tol,fi);
        if(fi<max_r){
            kl_match++;
        }

        kl_on_fov++;


    }

}


double kfvo::OptimizePosGT(keyframe &mkf, edge_tracker &et, TooN::Matrix<3, 3> &Pose, TooN::Vector<3> &Pos,double &K,int iter_max,double rew_dis,double rho_tol, double s_rho_q,int &mnum,TooN::Vector<6> &X,TooN::Matrix<6> &RRV){



    Matrix <3,3> BRelRot=mkf.Pose.T()*Pose;
    Vector <3> BRelPos=mkf.Pose.T()*(Pos-mkf.Pos)/K;

    TooN::Matrix<3,3> RVel=Identity*1e50;
    TooN::Matrix<3,3> RW0=Identity*1e50;

    Vector <3> BRelRotW=SO3<>(BRelRot).ln();

    for(KeyLine &kl:et)
        kl.m_id_f=-1;


    double r=Minimizer_RV_KF<double>(BRelPos,BRelRotW,RVel,RW0,mkf.tracker(),et,mkf.camera,K/mkf.K,5,30.0*M_PI/180.0,rho_tol,iter_max,rew_dis,s_rho_q,0,X,RRV);

    mnum=0;
    for(KeyLine &kl:et){
        //kl.m_id_kf=kl.m_id_f;
        if(kl.m_id_f>=0){
            mnum++;
        }
    }




    //std::cout << mnum<<"\n";

    BRelRot=SO3<>(BRelRotW).get_matrix();

    double Kp=optimizeScale(mkf,et,BRelRot,BRelPos);

    K=mkf.K*Kp;

    Pose=(SO3<>(mkf.Pose)*SO3<>(BRelRotW)).get_matrix();
    Pos=SO3<>(mkf.Pose)*BRelPos*K+mkf.Pos;

    /*
    std::ofstream debug_out("debug_out.txt");
    for(KeyLine &kl:et){

        if(kl.m_id_f<0){
            continue;
        }
        Vector <3> lp=mkf.camera.projectImgCordVec(mkf.Pose.T()*(Pose*mkf.camera.unprojectHomCordVec(makeVector(kl.p_m.x,kl.p_m.y,kl.rho))*K+Pos-mkf.Pos));
        debug_out << lp<<"\n";
    }*/

    return r;

    //Actualizar consatraits
}



double kfvo::OptimizePos1Iter(keyframe &mkf, edge_tracker &et, const TooN::Matrix<3, 3> &BRelRot, const TooN::Vector<3> &BRelPos,Matrix <6,6> &JtJ,Vector<6>&JtF){



    Matrix <Dynamic, 6> J(mkf.edges().KNum(),6);
    Vector <Dynamic>    f(mkf.edges().KNum());
    double E=0;
    for(int i=0;i<mkf.edges().KNum();i++){
        KeyLine & kl_b=mkf.edges()[i];

        if(kl_b.m_id_f<0){
            f[i]=0;
            J[i]=Zeros;
            continue;
        }
        KeyLine &kl=et[kl_b.m_id_f];


        Vector <3> Rx0=BRelRot*mkf.camera.unprojectHomCordVec(makeVector(kl.p_m.x,kl.p_m.y,kl.rho));
        Vector <3> q1=mkf.camera.projectHomCordVec(Rx0+BRelPos);



        f[i]=(q1[0]-kl_b.p_m.x)*kl_b.u_m.x+(q1[1]-kl_b.p_m.y)*kl_b.u_m.y;     //f=(q1-qm)*u


        Matrix <1,3> dfdx=Data( q1[2]*mkf.camera.zfm*kl_b.u_m.x,
                q1[2]*mkf.camera.zfm*kl_b.u_m.y,
                -q1[2]*q1[0]*kl_b.u_m.x-q1[2]*q1[1]*kl_b.u_m.y);



        E+=f[i]*f[i];
        double stddev=kl.s_rho;//util::norm(1.0,((dfdx*BRelPos)*kl.s_rho_kf/q1[2])[0]);
        f[i]/=stddev;
        J.slice(i,0,1,3)=dfdx/stddev;
        J.slice(i,3,1,3)=dfdx*util::crossMatrix(Rx0).T()/stddev;


    }

    JtJ=J.T()*J;
    JtF=J.T()*f;
    return E;
}

double kfvo::OptimizePos(keyframe &mkf, edge_tracker &et, TooN::Matrix<3, 3> &Pose, TooN::Vector<3> &Pos, const double &K, int iter_max){



    Matrix <6,6> JtJ,ApI,JtJnew;
    Vector<6>JtF,JtFnew;


    Matrix <3,3> BRelRot=mkf.Pose.T()*Pose;
    Vector <3> BRelPos=mkf.Pose.T()*(Pos-mkf.Pos)/K;


    double E=OptimizePos1Iter(mkf,et,BRelRot,BRelPos,JtJ,JtF);
    double E0=E;

    double v=2,tau=1e-3;
    double u=tau*TooN::max_element(JtJ).first;
    double gain;

    for(int iter=0;iter<iter_max;iter++){


        ApI=JtJ+Identity*u;
        Vector <6> dX=Cholesky<6>(ApI).backsub(-JtF);

        Vector <3> BRelPosNew=BRelPos+dX.slice<0,3>();
        Matrix <3,3>BRelRotNew=SO3<>(SO3<>(dX.slice<3,3>()).get_matrix()*BRelRot).get_matrix();



        double Enew=OptimizePos1Iter(mkf,et,BRelRotNew,BRelPosNew,JtJnew,JtFnew);

        gain=(E-Enew)/(0.5*dX*(u*dX-JtF));

        if(gain>0){
            E=Enew;
            BRelPos=BRelPosNew;
            BRelRot=BRelRotNew;
            JtJ=JtJnew;
            JtF=JtFnew;
            u*=std::max(0.33,1-((2*gain-1)*(2*gain-1)*(2*gain-1)));
            v=2;
        }else{
            u*=v;
            v*=2;
        }


    }

    //Kp=1;//optimizeScale(mkf,et,BRelRot,BRelPos);

    BRelPos*=K;

    Pose=mkf.Pose*BRelRot;
    Pos=mkf.Pose*BRelPos+mkf.Pos;
    return E/E0;
    //Actualizar consatraits
}

double kfvo::optimizeScale(keyframe &mkf, edge_tracker &et, const TooN::Matrix<3, 3> &BRelRot, const TooN::Vector<3> &BRelPos){



    double Kr=1;

    for(int iter=0;iter<1;iter++){


        double rTr=0,rTr0=0;
        for(int i=0;i<et.KNum();i++){
            KeyLine & kl=et[i];

            if(kl.m_id_f<0){
                continue;
            }
            KeyLine &kl_b=mkf.edges()[kl.m_id_f];


            Vector <3> Rx0=BRelRot*kfvo::unProject(makeVector(kl.p_m.x,kl.p_m.y,kl.rho),mkf.camera.zfm);
            Vector <3> q1=kfvo::project(Rx0+BRelPos,mkf.camera.zfm);

            if(q1[2]>0){
                double v=kl.s_rho*kl.s_rho*Kr*Kr+kl_b.s_rho*kl_b.s_rho;

                rTr0+=q1[2]*q1[2]/v;         //Weighted mean of corrected IDepth
                rTr+=q1[2]*kl_b.rho/v;    //Weighted mean of predicted IDepth

            }

        }

//        std::cout <<rTr<<" "<<rTr0<<"\n";
        Kr=(rTr0>0 && rTr>0)?(rTr0/rTr):1;
    }


    return Kr;
}


double kfvo::optimizeScaleF2KF(keyframe &mkf, edge_tracker &et, const TooN::Matrix<3, 3> &BRelRot, const TooN::Vector<3> &BRelPos,double &W_Kp){



    double Kr=1;

    for(int iter=0;iter<1;iter++){


        double rTr=0,rTr0=0;
        for(int i=0;i<et.KNum();i++){
            KeyLine & kl=et[i];

            if(kl.m_id_kf<0){
                continue;
            }
            KeyLine &kl_b=mkf.edges()[kl.m_id_kf];


            Vector <3> Rx0=BRelRot*kfvo::unProject(makeVector(kl.p_m.x,kl.p_m.y,kl.rho),mkf.camera.zfm);
            Vector <3> q1=kfvo::project(Rx0+BRelPos,mkf.camera.zfm);

            if(q1[2]>0){
                double v=kl.s_rho*kl.s_rho*Kr*Kr*(q1[2]/kl.rho)*(q1[2]/kl.rho)+kl_b.s_rho*kl_b.s_rho;

                rTr0+=q1[2]*q1[2]/v;         //Weighted mean of corrected IDepth
                rTr+=kl_b.rho*kl_b.rho/v;    //Weighted mean of predicted IDepth

            }

        }

//        std::cout <<rTr<<" "<<rTr0<<"\n";
        Kr=(rTr0>0 && rTr>0)?(rTr/rTr0):1;
        W_Kp=rTr0;
    }


    return Kr;
}


double kfvo::optimizeScaleBack(keyframe &mkf, edge_tracker &et, const TooN::Matrix<3, 3> &BRelRot, const TooN::Vector<3> &BRelPos,double K){

    double Kr=mkf.K;

    for(int iter=0;iter<1;iter++){


        double rTr=0,rTr0=0;
        for(KeyLine & kl_b:mkf.edges()){

            if(kl_b.m_id_f<0){
                continue;
            }

            KeyLine &kl=et[kl_b.m_id_f];


            Vector <3> Rx0=BRelRot*kfvo::unProject(makeVector(kl.p_m.x,kl.p_m.y,kl.rho),mkf.camera.zfm)*K;
            Vector <3> q1=kfvo::project(Rx0+BRelPos,mkf.camera.zfm);

            if(q1[2]>0){
                double v=util::norm2(kl.s_rho*q1[2]/kl.rho*Kr,kl_b.s_rho);

                rTr0+=q1[2]*q1[2]/v;         //Weighted mean of corrected IDepth
                rTr+=q1[2]*kl_b.rho/v;    //Weighted mean of predicted IDepth

            }

        }

//        std::cout <<rTr<<" "<<rTr0<<"\n";
        Kr=(rTr0>0 && rTr>0)?(rTr/rTr0):1;
    }


    return Kr;
}


double kfvo::OptimizeRelConstraint1Iter(keyframe &mkf, edge_tracker &et, const TooN::Matrix<3, 3> &BRelRot, const TooN::Vector<3> &BRelPos,Matrix <6,6> &JtJ,Vector<6>&JtF,bool etracket_2_keyframe,double k_huber,Vector <Dynamic> &residual,std::pair<int,int> &match_num){


    int kl_num=etracket_2_keyframe?et.KNum():mkf.edges().KNum();


    Matrix <Dynamic, 6> J(kl_num,6);
    Vector <Dynamic>    f(kl_num);
    double E=0;

    match_num.first=0;match_num.second=0;

    for(int i=0;i<kl_num;i++){

        KeyLine  *kl_b,*kl;

        if(etracket_2_keyframe){
            kl=&et[i];
            if(kl->m_id_kf<0){
                f[i]=0;
                J[i]=Zeros;
                continue;
            }
            kl_b=&mkf.edges()[kl->m_id_kf];
        }else{
            kl=&mkf.edges()[i];
            if(kl->m_id_f<0){
                f[i]=0;
                J[i]=Zeros;
                continue;
            }
            kl_b=&(et[kl->m_id_f]);
        }



        Vector <3> Rx0=BRelRot*mkf.camera.unprojectHomCordVec(makeVector(kl->p_m.x,kl->p_m.y,kl->rho));
        Vector <3> q1=mkf.camera.projectHomCordVec(Rx0+BRelPos);



        f[i]=(q1[0]-kl_b->p_m.x)*kl_b->u_m.x+(q1[1]-kl_b->p_m.y)*kl_b->u_m.y;     //f=(q1-qm)*u


        Matrix <1,3> dfdx=Data( q1[2]*mkf.camera.zfm*kl_b->u_m.x,
                q1[2]*mkf.camera.zfm*kl_b->u_m.y,
                -q1[2]*q1[0]*kl_b->u_m.x-q1[2]*q1[1]*kl_b->u_m.y);



        E+=f[i]*f[i];
        double stddev=util::norm(1.0,((dfdx*BRelPos)*kl->s_rho/q1[2])[0]);


        match_num.first++;
        double weigth=1;
        //Estimate reweighting
        if(k_huber>0 && fabs(residual[i])>k_huber)
            weigth=k_huber/fabs(residual[i]);
        else
            match_num.second++;


        residual[i]=f[i];

        f[i]*=weigth/stddev;
        J.slice(i,0,1,3)=weigth*dfdx/stddev;
        J.slice(i,3,1,3)=weigth*dfdx*util::crossMatrix(Rx0).T()/stddev;



    }

    JtJ=J.T()*J;
    JtF=J.T()*f;
    return E;
}


std::pair<int,int> kfvo::mutualExclusionSimple(keyframe &kf0,keyframe &kf1,double dist_t,bool discart_non_mut,bool kf0_2_kf1){


    std::pair<int,int> count(0,0);
    if(kf0_2_kf1){

     //   Matrix <3,3> BRelRot=kf0.Pose.T()*kf1.Pose;
     //   Vector <3> BRelPos=kf0.Pose.T()*(kf1.Pos-kf0.Pos)/kf1.K;


        for(KeyLine &kl:kf0.edges()){

            int m_f;
            if((m_f=kl.m_id_f)<0)
                continue;
            count.first++;

            int m_b=kf1.edges()[m_f].m_id_kf;

            if(m_b<0){

                if(discart_non_mut)
                    kl.m_id_f=-1;
                continue;
            }



            KeyLine &kl_m=kf0.edges()[m_b];

            if(util::norm(kl.p_m.x-kl_m.p_m.x,kl.p_m.y-kl_m.p_m.y)>dist_t){
                kl.m_id_f=-1;
                continue;
            }

        /*    KeyLine &kl_b=kf1.edges()[m_f];

            Vector<4> qr=rotateQs(makeVector(kl_b.p_m.x,kl_b.p_m.y,kl_b.rho,kl_b.s_rho),BRelRot,kf0.camera.zfm);

            float dq=(kl.p_m.x-qr[0])*kl.u_m.x+(kl.p_m.y-qr[1])*kl.u_m.y;
            float dv=(kf0.camera.zfm*BRelPos[0]-kl.p_m.x*BRelPos[2])*kl.u_m.x+(kf0.camera.zfm*BRelPos[1]-kl.p_m.y*BRelPos[2])*kl.u_m.y;

            if(dq<(dv*(qr[2]-qr[3])-1) || dq>(dv*(qr[2]+qr[3])+1)){
                kl.m_id_f=-1;
                continue;
            }*/

            count.second++;


        }
    }else{

     //   Matrix <3,3> BRelRot=kf1.Pose.T()*kf0.Pose;
      //  Vector <3> BRelPos=kf1.Pose.T()*(kf0.Pos-kf1.Pos)/kf0.K;

        for(KeyLine &kl:kf1.edges()){

            int m_f;
            if((m_f=kl.m_id_kf)<0)
                continue;
            count.first++;

            int m_b=kf0.edges()[m_f].m_id_f;

            if(m_b<0){
                if(discart_non_mut)
                    kl.m_id_kf=-1;
                continue;
            }



            KeyLine &kl_m=kf1.edges()[m_b];

            if(fabs((kl.p_m.x-kl_m.p_m.x)*kl.u_m.x+(kl.p_m.y-kl_m.p_m.y)*kl.u_m.y)>dist_t){
                kl.m_id_kf=-1;
                continue;
            }
/*
            KeyLine &kl_b=kf0.edges()[m_f];

            Vector<4> qr=rotateQs(makeVector(kl_b.p_m.x,kl_b.p_m.y,kl_b.rho,kl_b.s_rho*2),BRelRot,kf0.camera.zfm);

            float dq=(kl.p_m.x-qr[0])*kl.u_m.x+(kl.p_m.y-qr[1])*kl.u_m.y;
            float dv=(kf0.camera.zfm*BRelPos[0]-kl.p_m.x*BRelPos[2])*kl.u_m.x+(kf0.camera.zfm*BRelPos[1]-kl.p_m.y*BRelPos[2])*kl.u_m.y;

            if(dq<(dv*(qr[2]-qr[3])-1) || dq>(dv*(qr[2]+qr[3])+1)){
                kl.m_id_f=-1;
                continue;
            }*/
            count.second++;

        }


    }

    return count;
}




double kfvo::OptimizeRelContraint(keyframe &mkf, edge_tracker &et,const TooN::Matrix<3, 3> &Pose, const TooN::Vector<3> &Pos, const double &K, int iter_max,TooN::Vector<6> &X,TooN::Matrix<6, 6> &W_X,TooN::Matrix<6, 6> &R_X,std::pair<double,double>&optimK,bool etracket_2_keyframe,double k_huber,std::pair<int,int> &match_num){



    Matrix <6,6> JtJ,ApI,JtJnew;
    Vector<6>JtF,JtFnew;


    Matrix <3,3> BRelRot;
    Vector <3> BRelPos;

    if(etracket_2_keyframe){
        BRelRot=mkf.Pose.T()*Pose;
        BRelPos=mkf.Pose.T()*(Pos-mkf.Pos)/K;
    }else{
        BRelRot=Pose.T()*mkf.Pose;
        BRelPos=Pose.T()*(mkf.Pos-Pos)/mkf.K;
    }


    int kl_num=etracket_2_keyframe?et.KNum():mkf.edges().KNum();
    Vector <Dynamic>residual(kl_num);
    residual=Zeros;


    double E=OptimizeRelConstraint1Iter(mkf,et,BRelRot,BRelPos,JtJ,JtF,etracket_2_keyframe,k_huber,residual,match_num);
    E=OptimizeRelConstraint1Iter(mkf,et,BRelRot,BRelPos,JtJ,JtF,etracket_2_keyframe,k_huber,residual,match_num);
    double E0=E;

    double v=2,tau=1e-3;
    double u=tau*TooN::max_element(JtJ).first;
    double gain;

    for(int iter=0;iter<iter_max;iter++){

        ApI=JtJ+Identity*u;
        Vector <6> dX=Cholesky<6>(ApI).backsub(-JtF);


        Vector <3> BRelPosNew=BRelPos+dX.slice<0,3>();
        Matrix <3,3>BRelRotNew=SO3<>(SO3<>(dX.slice<3,3>()).get_matrix()*BRelRot).get_matrix();



        double Enew=OptimizeRelConstraint1Iter(mkf,et,BRelRotNew,BRelPosNew,JtJnew,JtFnew,etracket_2_keyframe,k_huber,residual,match_num);

        gain=(E-Enew)/(0.5*dX*(u*dX-JtF));

        if(gain>0){
            E=Enew;
            BRelPos=BRelPosNew;
            BRelRot=BRelRotNew;
            JtJ=JtJnew;
            JtF=JtFnew;
            u*=std::max(0.33,1-((2*gain-1)*(2*gain-1)*(2*gain-1)));
            v=2;
        }else{
            u*=v;
            v*=2;
        }


    }

    X.slice<0,3>()=BRelPos;
    X.slice<3,3>()=SO3<>(BRelRot).ln();


    Cholesky <6>svdJtJ(JtJ);
    R_X=svdJtJ.get_inverse();
    W_X=JtJ;

    if(etracket_2_keyframe)
        optimK.first=optimizeScaleF2KF(mkf,et,BRelRot,BRelPos,optimK.second);

    return E/E0;
    //Actualizar consatraits
}


int kfvo::translateDepth_KF2F(keyframe &mkf, edge_tracker &et, TooN::Matrix<3, 3> &Pose, TooN::Vector<3> &Pos,double &K){


    Matrix <3,3> BRelRot=Pose.T()*mkf.Pose;
    Vector <3> BRelPos=Pose.T()*(mkf.Pos-Pos);


    int count=0;

    for(KeyLine &klf:et){
        if(klf.m_id_kf>=0){
            KeyLine &kl=mkf.edges()[klf.m_id_kf];

            Vector <3> p0=mkf.camera.unprojectHomCordVec(makeVector(kl.p_m.x,kl.p_m.y,kl.rho));

            Vector <3> q1=mkf.camera.projectHomCordVec(BRelRot*p0*K+BRelPos);

            klf.rho=q1[2]*mkf.K;
            klf.s_rho=klf.rho/kl.rho*kl.s_rho;
            count++;
        }

    }

    return count;

    //Actualizar consatraits
}


void kfvo::translateDepth_New2Old(std::vector<keyframe> &kf_list,int iters){


    for(int iter=0;iter<iters;iter++){
        for (int i = 0; i < (int) kf_list.size()-1; i++) {

            keyframe &kf0=kf_list[i];
            keyframe &kf1=kf_list[i+1];

            translateDepth_F2KF(kf0,kf1.edges(),kf1.Pose,kf1.Pos,kf1.K,false);
        }
    }

}


int kfvo::translateDepth_F2KF(keyframe &mkf, edge_tracker &et, const TooN::Matrix<3, 3> &Pose, const TooN::Vector<3> &Pos,const double &K,bool optim_scale){




    Matrix <3,3> BRelRot=mkf.Pose.T()*Pose;
    Vector <3> BRelPos=mkf.Pose.T()*(Pos-mkf.Pos);


    if(optim_scale)
        mkf.K=optimizeScaleBack(mkf,et,BRelRot,BRelPos,K);

    int count=0;

    for(KeyLine &klf:mkf.edges()){
        if(klf.m_id_f>=0){
            KeyLine &kl=et[klf.m_id_f];

            Vector <3> p0=mkf.camera.unprojectHomCordVec(makeVector(kl.p_m.x,kl.p_m.y,kl.rho));

            Vector <3> q1=mkf.camera.projectHomCordVec(BRelRot*p0*K+BRelPos);

            klf.rho=q1[2]*mkf.K;
            klf.s_rho=klf.rho*(kl.s_rho/kl.rho);
            klf.m_num++;
            count++;
        }

    }

    return count;

    //Actualizar consatraits
}

int kfvo::kls_on_fov(keyframe &mkf,TooN::Matrix<3, 3> &Pose, TooN::Vector<3> &Pos){

    int count=0;

    Matrix <3,3> BRelRot=Pose.T()*mkf.Pose;
    Vector <3> BRelPos=Pose.T()*(mkf.Pos-Pos);

    for(KeyLine &kl:mkf.edges()){
        Vector <3> p0=mkf.camera.unprojectHomCordVec(makeVector(kl.p_m.x,kl.p_m.y,kl.rho))*mkf.K;

        Vector <3> q1=mkf.camera.projectImgCordVec(BRelRot*p0+BRelPos);


        int x=util::round2int_positive(q1[0]);    //Round to integer image coordinates
        int y=util::round2int_positive(q1[1]);

        if(x<0|| y<0|| x>=(int)mkf.camera.sz.w || y>=(int)mkf.camera.sz.h || q1[2]<=0){ //If tranformed position is outside image border
            continue;
        }
        count++;
    }

    return count;

}

int kfvo::matchStereo(keyframe &mkf, edge_tracker &et, TooN::Matrix<3, 3> &Pose, TooN::Vector<3> &Pos,double &K,double mod_thr,double ang_thr,int max_r,double loc_error){


    int count=0;


    Matrix <3,3> BRelRot=Pose.T()*mkf.Pose;
    Vector <3> BRelPos=Pose.T()*(mkf.Pos-Pos)/mkf.K;

    for(KeyLine &kl:mkf.edges()){
        int m_id;
        if((m_id=et.search_match(kl,-BRelPos,Zeros,BRelRot,mod_thr,ang_thr,max_r,loc_error))>=0){



            count++;
        }

    }

    return count;

    //Actualizar constraits
}

int kfvo::buildForwardMatch(keyframe &mkf, edge_tracker &et, edge_tracker &et0){


    int count=0;
    int fowMatch[et0.KNum()];

    for(int i=0;i<et0.KNum();i++)
        fowMatch[i]=-1;

    for(int i=0;i<et.KNum();i++){

        if(et[i].m_id>=0){
            fowMatch[et[i].m_id]=i; //match from old_frame to new frame
        }
    }

    for(KeyLine &kl:mkf.edges()){
        if(kl.m_id_f>=0){
            int new_match=fowMatch[kl.m_id_f];
            if(new_match<0){
                kl.m_id_f=-1;
            }else{
                kl.m_id_f=new_match;
                count ++;
            }
        }
    }


    return count;

    //Actualizar constraits
}


void kfvo::resetForwardMatch(keyframe &mkf){
    for(int i=0;i<mkf.edges().KNum();i++){
        mkf.edges()[i].m_id_f=i;

        mkf.edges()[i].rho0=mkf.edges()[i].rho;
        mkf.edges()[i].s_rho0=mkf.edges()[i].s_rho;
    }
}

void kfvo::resetKFMatch(edge_tracker &et){
    for(int i=0;i<et.KNum();i++){
        et[i].m_id_kf=i;
    }
}


double kfvo::stereoCorrect(keyframe &mkf, edge_tracker &et, const TooN::Matrix<3, 3> &Pose, const TooN::Vector<3> &Pos,double dist_tolerance)
{

    Matrix <3,3> R=mkf.Pose.T()*Pose;
    Vector <3> t=Pose.T()*(mkf.Pos-Pos);
    Matrix <3,3> E=R*util::crossMatrix(t);

    for(int i=0;i<et.KNum();i++)
        stereoCorrect(mkf,et[i],E,dist_tolerance);
    return 0;
}



double kfvo::stereoCorrect(keyframe &mkf,KeyLine &kl,const TooN::Matrix<3,3> &E,double dist_tolerance){

    if(kl.m_id_kf<0)
        return -1;


    TooN::Vector<3> e=E*TooN::makeVector(kl.p_m.x,kl.p_m.y,mkf.camera.zfm);
    TooN::Vector<3> r=e/util::norm(e[0],e[1]);
    r[2]*=mkf.camera.zfm;


    KeyLine *klm=&mkf.edges()[kl.m_id_kf];
    double d0=fabs(klm->p_m.x*r[0]+klm->p_m.y*r[1]+r[2]);

    if(d0<dist_tolerance)
        return d0;

    if(klm->n_id>=0){
        KeyLine *klmn=&mkf.edges()[klm->n_id];

        double d=fabs(klmn->p_m.x*r[0]+klmn->p_m.y*r[1]+r[2]);

        if(fabs(d)<fabs(d0)){

            while(1){

                kl.m_id_kf=klm->n_id;
                d0=d;

                if(d0<dist_tolerance)
                    return d0;

                klm=&mkf.edges()[kl.m_id_kf];

                if(klm->n_id>=0){

                    klmn=&mkf.edges()[klm->n_id];
                    d=fabs(klmn->p_m.x*r[0]+klmn->p_m.y*r[1]+r[2]);

                    if(d>=d0)
                        return d0;

                }else{
                    return d0;
                }

            }

        }
    }

    if(klm->p_id>=0){
        KeyLine *klmn=&mkf.edges()[klm->p_id];
        double d=fabs(klmn->p_m.x*r[0]+klmn->p_m.y*r[1]+r[2]);
        if(d<d0){

            while(1){
                kl.m_id_kf=klm->p_id;
                d0=d;


                if(d0<dist_tolerance)
                    return d0;

                klm=&mkf.edges()[kl.m_id_kf];

                if(klm->p_id>=0){

                    klmn=&mkf.edges()[klm->p_id];
                    d=fabs(klmn->p_m.x*r[0]+klmn->p_m.y*r[1]+r[2]);
                    if(d>=d0)
                        return d0;

                }else{
                    return d0;
                }

            }

        }
    }
    return d0;

}





int kfvo::correctAugmentate(keyframe &mkf, edge_tracker &et, const TooN::Matrix<3, 3> &Pose, const TooN::Vector<3> &Pos,const double & dist_thesh,const double &dist_tolerance,bool augmentate){


    Matrix <3,3> R=mkf.Pose.T()*Pose;
    Vector <3> t=Pose.T()*(mkf.Pos-Pos);
    Matrix <3,3> E=R*util::crossMatrix(t);

    double dist[et.KNum()];

    for(int i=0;i<et.KNum();i++){
        dist[i]=stereoCorrect(mkf,et[i],E,dist_tolerance);
    }

    if(augmentate)
        for(int i=0;i<et.KNum();i++)
            correctAugmentate(mkf,et,i,E,dist_thesh,dist_tolerance);

    int count=0;

    for(int i=0;i<et.KNum();i++){
        if(dist[i]>dist_thesh)
            et[i].m_id_kf=-1;

        if(et[i].m_id_kf>=0)
            count++;
    }
    return count;
}


void kfvo::correctAugmentate(keyframe &mkf, edge_tracker &et,int kl_id,const TooN::Matrix<3,3> &E,const double &dist_thesh,const double &dist_tolerance){

    KeyLine *kl=&et[kl_id];

    if(kl->m_id_kf<0)
        return;


    while(1){
        if(kl->p_id<0)
            break;
        KeyLine *kln=&(et)[kl->p_id];
        if(kln->m_id_kf>=0)
            break;
        kln->m_id_kf=kl->m_id_kf;
        if(stereoCorrect(mkf,*kln,E,dist_tolerance)>dist_thesh){
            kln->m_id_kf=-1;
            break;
        }


        kl=kln;
    }

    kl=&et[kl_id];

    while(1){
        if(kl->n_id<0)
            break;
        KeyLine *kln=&(et)[kl->n_id];
        if(kln->m_id_kf>=0)
            break;
        kln->m_id_kf=kl->m_id_kf;
        if(stereoCorrect(mkf,*kln,E,dist_tolerance)>dist_thesh){
            kln->m_id_kf=-1;
            break;
        }


        kl=kln;
    }


}


int kfvo::forwardCorrectAugmentate(keyframe &mkf, edge_tracker &et, const TooN::Matrix<3, 3> &Pose, const TooN::Vector<3> &Pos,double dist_thesh,const double &dist_tolerance,bool augmentate){


    Matrix <3,3> R=Pose.T()*mkf.Pose;
    Vector <3> t=mkf.Pose.T()*(Pos-mkf.Pos);
    Matrix <3,3> E=R*util::crossMatrix(t);


    double dist[mkf.edges().KNum()];

    for(int i=0;i<mkf.edges().KNum();i++)
        dist[i]=forwardStereoCorrect(et,mkf.edges()[i],mkf.edges()[i].m_id_f,E,dist_tolerance,mkf.camera.zfm);

    if(augmentate)
        for(int i=0;i<mkf.edges().KNum();i++)
            forwardCorrectAugmentate(et,mkf,i,E,dist_thesh,dist_tolerance,mkf.camera.zfm);

    int count=0;
    for(int i=0;i<mkf.edges().KNum();i++){
        if(dist[i]>dist_thesh)
            mkf.edges()[i].m_id_f=-1;

        if(mkf.edges()[i].m_id_f>=0)
            count++;
    }
    return count;
}


void kfvo::forwardCorrectAugmentate(edge_tracker &et2match,keyframe &mkf,const int &kl_id,const TooN::Matrix<3,3> &E,const double &dist_thesh,const double &dist_tolerance,const double &zf){

    KeyLine *kl=&mkf.edges()[kl_id];    //keyline to use as reference

    if(kl->m_id_f<0)                    //If no match.. nothing to do
        return;


    while(1){
        if(kl->p_id<0)
            break;
        KeyLine *kln=&mkf.edges()[kl->p_id];    //kln  is  is a neighbour
        if(kln->m_id_f>=0)                      //if it has a match there is nothing to do
            break;
        kln->m_id_f=kl->m_id_f;                 //if not... use the current match
        if(forwardStereoCorrect(et2match,*kln,kln->m_id_f,E,dist_tolerance,zf)>dist_thesh){ //stereo correct
            kln->m_id_f=-1;
            break;
        }

        kl=kln;                                 //move on the edge forward
    }

    kl=&mkf.edges()[kl_id];                     //back to he first kl

    while(1){
        if(kl->n_id<0)                          //do the same on the other direction
            break;

        KeyLine *kln=&mkf.edges()[kl->n_id];

        if(kln->m_id_f>=0)
            break;
        kln->m_id_f=kl->m_id_f;
        if(forwardStereoCorrect(et2match,*kln,kln->m_id_f,E,dist_tolerance,zf)>dist_thesh){
            kln->m_id_f=-1;
            break;
        }

        kl=kln;
    }


}


double kfvo::forwardStereoCorrect(keyframe &mkf, edge_tracker &et, const TooN::Matrix<3, 3> &Pose, const TooN::Vector<3> &Pos,double dist_tolerance)
{

    Matrix <3,3> R=Pose.T()*mkf.Pose;
    Vector <3> t=mkf.Pose.T()*(Pos-mkf.Pos);
    Matrix <3,3> E=R*util::crossMatrix(t);

    for(int i=0;i<mkf.edges().KNum();i++)
        forwardStereoCorrect(et,mkf.edges()[i],mkf.edges()[i].m_id_f,E,dist_tolerance,mkf.camera.zfm);
    return 0;
}



double kfvo::forwardStereoCorrect(edge_tracker &et,KeyLine &kl,int &f_id,const TooN::Matrix<3,3> &E,double dist_tolerance,const double &zf){

    if(f_id<0)
        return -1;



    TooN::Vector<3> e=E*TooN::makeVector(kl.p_m.x,kl.p_m.y,zf);
    TooN::Vector<3> r=e/util::norm(e[0],e[1]);
    r[2]*=zf;


    KeyLine *klm=&et[f_id];                                 //Matched keyline
    double d0=fabs(klm->p_m.x*r[0]+klm->p_m.y*r[1]+r[2]);   //stereo distance

    if(d0<dist_tolerance)
        return d0;

    if(klm->n_id>=0){
        KeyLine *klmn=&et[klm->n_id];   //try n neigbour

        double d=fabs(klmn->p_m.x*r[0]+klmn->p_m.y*r[1]+r[2]);  //new distance

        if(fabs(d)<fabs(d0)){           //if distance is less move in that direction

            while(1){

                f_id=klm->n_id;         //change to this kl
                d0=d;

                if(d0<dist_tolerance)
                    return d0;

                klm=&et[f_id];

                if(klm->n_id>=0){       //repeat...

                    klmn=&et[klm->n_id];
                    d=fabs(klmn->p_m.x*r[0]+klmn->p_m.y*r[1]+r[2]);

                    if(d>=d0)
                        return d0;

                }else{
                    return d0;
                }

            }

        }
    }

    if(klm->p_id>=0){                   //try the other direction
        KeyLine *klmn=&et[klm->p_id];   //go on the neighbour
        double d=fabs(klmn->p_m.x*r[0]+klmn->p_m.y*r[1]+r[2]);
        if(d<d0){

            while(1){
                f_id=klm->p_id;
                d0=d;


                if(d0<dist_tolerance)
                    return d0;

                klm=&et[f_id];

                if(klm->p_id>=0){

                    klmn=&et[klm->p_id];
                    d=fabs(klmn->p_m.x*r[0]+klmn->p_m.y*r[1]+r[2]);
                    if(d>=d0)
                        return d0;

                }else{
                    return d0;
                }

            }

        }
    }
    return d0;

}




void kfvo::mapKFUsingIDK(keyframe &mkf,
                                     edge_tracker &et,
                                    TooN::Matrix<3, 3> &Pose,
                                    TooN::Vector<3> &Pos,
                                     double ReshapeQAbsolute,
                                     double ReshapeQRelative,
                                     double LocationUncertainty)
{


    for(int i=0;i<0;i++)
        mkf.edges().Regularize_1_iter(0.5);

    Matrix <3,3> FRelRot=Pose.T()*mkf.Pose;
    Vector <3> FRelPos=Pose.T()*(mkf.Pos-Pos);

    for(int i=0;i<mkf.edges().KNum();i++)
    {
        KeyLine &kl=mkf.edges()[i];
        if(kl.m_id_f>=0){
            mapKLUsingIDK(kl,et[kl.m_id_f],FRelRot,FRelPos,mkf.camera.zfm,ReshapeQRelative,ReshapeQAbsolute,LocationUncertainty);

        }
    }
/*
    for(KeyLine &etkl:et){
        if(etkl.m_id_kf>=0){
            KeyLine &kl=mkf.edges()[etkl.kf_id];

            Vector <3> q=project(FRelRot*unProject(makeVector(kl.p_m.x,kl.p_m.y,kl.rho),mkf.GetCam().zfm)+FRelPos,mkf.GetCam().zfm);

            etkl.rho_kf=q[2];
            etkl.rho=q[2];
            etkl.s_rho=kl.s_rho;
        }
    }*/

}


/*
double kfvo::mapKLUsingIDK(   KeyLine &kli,KeyLine &klb,
                              const TooN::Matrix<3, 3> &BRelRot,
                              const TooN::Vector<3> &BRelPos,
                              double zf,
                              double ReshapeQRelative,
                              double ReshapeQAbsolute,
                              double LocationUncertainty){




    if(kli.s_rho<0 || kli.rho<0){       //Debug check
        std::cout << "\nKF IDK panic! "<<kli.s_rho<<" "<<kli.rho<<"\n";
    }

    //klb=BRelRot*kli+BRelPos

    double rho_p=kli.rho;
    Vector <3> qR0=BRelRot*makeVector(kli.p_m.x/zf,kli.p_m.y/zf,1);

    double v_rho=kli.s_rho*kli.s_rho;


    double u_x=klb.u_m.x;     //Shortcut for edge perpendicular direction
    double u_y=klb.u_m.y;

    double qMx=klb.p_m.x;    //Meassurement homogeneus coordinates
    double qMy=klb.p_m.y;



    double p_p=v_rho+                               //Uncertainty propagation
            util::square(rho_p*ReshapeQRelative)+     //Relative uncertainty model
            util::square(ReshapeQAbsolute);             //Absolute uncertainty model


    for(int iter=0;iter<10;iter++){


        double qPx=zf*(qR0[0]+rho_p*BRelPos[0])/(qR0[2]+rho_p*BRelPos[2]);       //KeyLine new homogeneus coordinates
        double qPy=zf*(qR0[1]+rho_p*BRelPos[1])/(qR0[2]+rho_p*BRelPos[2]);


        double Y = u_x*(qMx-qPx)+u_y*(qMy-qPy); //Pixel displacement proyected on u
        double H= (u_x*zf*(BRelPos[0]*qR0[2]-BRelPos[2]*qR0[0])+u_y*zf*(BRelPos[1]*qR0[2]-BRelPos[2]*qR0[1]))
                /util::square(qR0[2]+BRelPos[2]*rho_p);

        double e=Y-H*rho_p;                     //error correction

        //*** Kalman update ecuations ****
        double S=H*p_p*H+util::square(LocationUncertainty);

        double K=p_p*H*(1/S);

        rho_p=rho_p+(K*e);


        kli.rho=rho_p;
        kli.s_rho=sqrt((1-K*H)*p_p);

    }


    //*** if inverse depth goes beyond limits apply correction ****

    if(kli.rho<RHO_MIN){
        kli.s_rho+=RHO_MIN-kli.rho;
        kli.rho=RHO_MIN;
    }else if(kli.rho>RHO_MAX){
        kli.rho=RHO_MAX;
    }else if(isnan(kli.rho) || isnan(kli.s_rho) || isinf(kli.rho) || isinf(kli.s_rho)){     //This checks should never happen
        std::cout<<"\nKL EKF Nan Rho!"<<rho_p<< " " << p_p<< " "<<BRelPos[2]<<"\n";
        kli.rho=RhoInit;
        kli.s_rho=RHO_MAX;
    }else if(kli.s_rho<0){                                                                  //only for debug
        std::cout<<"\nKL EKF Panic! Neg VRho!"<<BRelPos <<kli.rho << kli.s_rho <<"\n";
        kli.rho=RhoInit;
        kli.s_rho=RHO_MAX;

    }

    return kli.rho;


}

*/

double kfvo::mapKLUsingIDK(   KeyLine &kli,KeyLine &klb,
                              const TooN::Matrix<3, 3> &BRelRot,
                              const TooN::Vector<3> &BRelPos,
                              double zf,
                              double ReshapeQRelative,
                              double ReshapeQAbsolute,
                              double LocationUncertainty){




    if(kli.s_rho<0 || kli.rho<0){       //Debug check
        std::cout << "\nKF IDK panic! "<<kli.s_rho<<" "<<kli.rho<<"\n";
    }

    //klb=BRelRot*kli+BRelPos

    Vector <4> q=rotateQs(makeVector(kli.p_m.x,kli.p_m.y,kli.rho,kli.s_rho),BRelRot,zf);

    double qx,qy,q0x,q0y;
    qx=q[0];       //KeyLine new homogeneus coordinates
    qy=q[1];


    q0x=klb.p_m.x;    //Keyline old homogeneus coordinates
    q0y=klb.p_m.y;


    double v_rho=q[3]*q[3];   //IDepth variance

    double u_x=klb.u_m.x;     //Shortcut for edge perpendicular direction
    double u_y=klb.u_m.y;


    double rho_p=q[2];                      //Predicted Inverse Depth


    double p_p=v_rho+                               //Uncertainty propagation
            util::square(q[2]*ReshapeQRelative)+     //Relative uncertainty model
            util::square(ReshapeQAbsolute);             //Absolute uncertainty model



    double Y = u_x*(q0x-qx)+u_y*(q0y-qy); //Pixel displacement proyected on u
    double H= u_x*(BRelPos[0]*zf-BRelPos[2]*q0x)+u_y*(BRelPos[1]*zf-BRelPos[2]*q0y);

    double e=Y-H*rho_p;                     //error correction

    //*** Kalman update ecuations ****
    double S=H*p_p*H+util::square(LocationUncertainty);

    double K=p_p*H*(1/S);

    q[2]=rho_p+(K*e);

    v_rho=(1-K*H)*p_p;

    q[3]=sqrt(v_rho);

    Vector <4> q_up=rotateQs(q,BRelRot.T(),zf);

    kli.rho=q_up[2];
    kli.s_rho=q_up[3];

    //*** if inverse depth goes beyond limits apply correction ****

    if(kli.rho<RHO_MIN){
        kli.s_rho+=RHO_MIN-kli.rho;
        kli.rho=RHO_MIN;
    }else if(kli.rho>RHO_MAX){
        kli.rho=RHO_MAX;
    }else if(isnan(kli.rho) || isnan(kli.s_rho) || isinf(kli.rho) || isinf(kli.s_rho)){     //This checks should never happen
        std::cout<<"\nKL EKF Nan Rho!"<<rho_p<< " " << p_p<< " "<<BRelPos[2]<<"\n";
        kli.rho=RhoInit;
        kli.s_rho=RHO_MAX;
    }else if(kli.s_rho<0){                                                                  //only for debug
        std::cout<<"\nKL EKF Panic! Neg VRho!"<<BRelPos <<kli.rho << kli.s_rho <<"\n";
        kli.rho=RhoInit;
        kli.s_rho=RHO_MAX;

    }

    return kli.rho;

}


template <class T>
void KltoI3PMatrixKF(edge_tracker & klist,    //
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



template <class T,          //Presition used for calculations, NEON only suports float, double is better...
          bool ReWeight,    //ReWeigthing compile time switch
          bool ProcJF,      //Calculate Jacobians or just energy score?
          bool UsePriors>   //Switch to use priors
double kfvo::TryVelRot(TooN::Matrix<6,6,T> &JtJ,          //Estimated Matrixes J=Jacobian Matrix
                                 TooN::Vector<6,T> &JtF,            //F=Residual matrix
                                 const TooN::Vector<6,T> &VelRot,   //Proposed state vector  [Transtalation Rotation]
                                 const TooN::Vector<3> &V0p,        //Translation Model
                                 const TooN::Matrix<3,3> &PV0,      //Model uncertainty
                                 const TooN::Vector<3> &W0p,        //Rotation Model
                                 const TooN::Matrix<3,3> &PW0,      //Model uncertainty
                                 global_tracker &gt,
                                 edge_tracker &klist,               //KL list for matching
                                 cam_model &cam_mod,
                                 double Kr,
                                 T *P0m,                            //linear transpose coodinates vector of 3D klist positions
                                 int pnum,                          //Number of points (floured to a multiple of four)
                                 double match_mod,               //...
                                 double match_cang,               //...
                                 double rho_tol,
                                 double s_rho_min,                  //Threshold on IDepth uncertainty
                                 uint MatchNumThresh,               //Threshold on the matching history of the keyline
                                 T k_huber,                         //ReWeigth distance
                                 T* DResidual,                      //Last iteration Distance Residuals
                                 T* DResidualNew)                   //New iteration Distance Residuals
{

    double score=0; //Total error energy

    SO3 <> RotW0(VelRot.template slice<3,3>());
    const Matrix <3,3,T> &R0=RotW0.get_matrix();    //State Rotation matrix


    SO3 <> RotM(makeVector(0,0,VelRot[5]));                         //Rotation about Z-Axis (for gradient direction) //BUG! it was VelRot[2]
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
            Rt[Ne10::Index2Dto1D<3>(j,i)]=R0(i,j);
        Vt[i]=VelRot[i];
    }

    //Perform SE3 using NE10 wrapper
    Ne10::SE3on3PMatrix<T>(Rt,Vt,Ptm,P0m,pnum);

    //Proyect using NE10 wrapper
    Ne10::ProyP3toI3PMatrix<T>(PtIm,Ptm,cam_mod.zfm,pnum);


    T fi=0;

/*
    static int iter=0;
    char debug_name[250];
    snprintf(debug_name,250,"debug_optim%d.txt",iter);
    iter++;
    std::ofstream debug_file(debug_name);
*/


    //For each keyline in klist (old edgemap)...
    for(int ikl=0;ikl<klist.KNum();ikl++){

        KeyLine &kl=klist[ikl];

        kl.m_id_f=-1;               //Reset forward match

        if(kl.s_rho>s_rho_min || kl.m_num<MatchNumThresh){     //If uncertainty is to hi, dont use this keyline
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
            fm[ikl]=gt.getMaxSRadius();/*/kl.s_rho*/;                                       //consider it a missmatch with maximun possible residual
            if(ReWeight)
                fm[ikl]*=weigth;
            df_dPi_x[ikl]=0;
            df_dPi_y[ikl]=0;
            DResidualNew[ikl]=gt.getMaxSRadius();
            continue;
        }


        Point2DF kl_m=kl.m_m;
      //  kl.m_m.x=RM(0,0)*kl_m.x+RM(0,1)*kl_m.y;
      //  kl.m_m.y=RM(1,0)*kl_m.x+RM(1,1)*kl_m.y;     //Temporaly rotate keylines gradient on z-axis for improved matching

        //Match and Calculate residuals and gradients
        fm[ikl]=gt.Calc_f_J_Complete<T>(y*cam_mod.sz.w+x,df_dPi_x[ikl],df_dPi_y[ikl],kl,p_pji,gt.getMaxSRadius(),match_mod,match_cang,mnum,PtIm[2*pnum+ikl]/Kr,rho_tol,fi);

        kl.m_m=kl_m;        //Return gradient to it's original state

/*
        {
            if(fm[ikl]<gt.getMaxSRadius()){                 //Check for KL presence

                KeyLine &f_kl=(*klist_f)[field[y*cam_mod.sz.w+x].ikl]; //Field keyline


                debug_file<<x<<" "<<y<<" "<<f_kl.c_p.x<<" "<<f_kl.c_p.y<<"\n";
            }

        }

        */


        //Optionally appy reweigting
        if(ReWeight){
            fm[ikl]*=weigth;
            df_dPi_x[ikl]*=weigth;
            df_dPi_y[ikl]*=weigth;
        }

        //Save pixel residual for this iteration
        DResidualNew[ikl]=fi;



    }

    //debug_file.close();


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
// Optimization on the velocity and rotation using lenveng-marquart for KF bases optim
//
// *******************************************************************************


template <class T,          //Presition, NEON only supperots float
          bool UsePriors>   //Use priors switch
double kfvo::Minimizer_RV_KF(
                                    TooN::Vector<3> &Vel,       //Initial estimate on the velocity (translation)
                                    TooN::Vector<3> &W0,        //Initial estimate on the rotation
                                    TooN::Matrix<3,3> &RVel,    //Uncertainty Model if the initial estimate will
                                    TooN::Matrix<3,3> &RW0,     //be used as prior
                                    global_tracker &gt,
                                    edge_tracker &klist,        //KList to match to
                                    cam_model &cam_mod,
                                    double Kr,
                                    double match_mod,        //Match Gradient Treshold
                                    double match_ang,        //Match Gradient Treshold angle
                                    double rho_tol,
                                    int iter_max,               //Iteration number
                                    double reweigth_distance,   //Distance cut-off for reweigthing
                                    const double &max_s_rho,    //Threshold on the IDepth uncertainty
                                    const uint& MatchNumThresh,  //Threshold on the KL match number
                                    TooN::Vector<6,T> &X,
                                    TooN::Matrix<6> &RRV
                                    )
{



    if(klist.KNum()<=0)
        return 0;


    TooN::Matrix<6,6,T> JtJ,ApI,JtJnew;
    TooN::Vector<6,T> JtF,JtFnew;
    TooN::Vector<6,T> h,Xnew;



    int pnum=(klist.KNum()+0x3)&(~0x3);     //Number of points up-rounded to a multiple of 4 (NE10Wrapper requirment)
    T P0Im[pnum*3];                         //Image coordinates (x,y,rho)
    T P0m[pnum*3];                          //3Dcoordinates
    T Res0[pnum], Res1[pnum];               //DIstance Residuals
    T *Residual=Res0,*ResidualNew=Res1;     //

    //Convert to LTCV format
    KltoI3PMatrixKF<T>(klist,pnum,P0Im);
    //Proyect to 3D using NE10Wrapper
    Ne10::ProyI3Pto3PMatrix<T>(P0m,P0Im,cam_mod.zfm,pnum);


    double F,Fnew,F0;           //Total energy score
    double v=2,tau=1e-3;        //LM params
    double u,gain;
    int eff_steps=0;

    for(int i=0;i<pnum;i++)     //Init residuals
        Residual[i]=0;

    double k_hubber=reweigth_distance;  //Reweigth distance

    X.template slice<0,3>()=Vel;    //Prior initial condition
    X.template slice<3,3>()=W0;     //



    //Start or continue iterating, now using reweigth
    F0=F=TryVelRot<T,true,true,UsePriors>(JtJ,JtF,X,Vel,RVel,W0,RW0,gt,klist,cam_mod,Kr, P0m,pnum,match_mod,cos(match_ang),rho_tol,max_s_rho,MatchNumThresh,k_hubber,Residual,ResidualNew);
    u=tau*TooN::max_element(JtJ).first; //Levenberg-Marquart param reset
    v=2;


  /*  double mean_res=0;
    std::ofstream ofile("residuals_0.txt");
    for(int i=0;i<klist.KNum();i++){
        mean_res+=fabs(ResidualNew[i]);
        ofile << ResidualNew[i]<<" ";
    }
    ofile.close();
    mean_res/=klist.KNum();
    std::cout<<"Minimizer RV: Mean error start: "<<mean_res;
*/

    int lm_iter=0;
    for(;lm_iter<iter_max;lm_iter++){

        ApI=JtJ+Identity*u;

        //Solve ApI*h=-JtF

        Cholesky <6>svdApI(ApI);    //ApI is symetric, use cholesky
        h=svdApI.backsub(-JtF);


        Xnew=X+h;
        Fnew=TryVelRot<T,true,true,UsePriors>(JtJnew,JtFnew,Xnew,Vel,RVel,W0,RW0,gt,klist,cam_mod,Kr, P0m,pnum,match_mod,cos(match_ang),rho_tol,max_s_rho,MatchNumThresh,k_hubber,Residual,ResidualNew);
/*
        if(lm_iter==iter_max-1){
            std::ofstream ofile("residuals.txt");
            mean_res=0;
            for(int i=0;i<klist.KNum();i++){
                mean_res+=fabs(ResidualNew[i]);
                ofile << ResidualNew[i]<<" ";
            }
            ofile.close();
            mean_res/=klist.KNum();


            std::cout<<" end: "<<mean_res<<"\n";
        }*/

        gain=(F-Fnew)/(0.5*h*(u*h-JtF));    //Check gain

        //std::cout <<"i: "<<lm_iter<<" g:"<<gain<<" F:"<<F<<" Fn:"<<Fnew<<" ";

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
    RRV=svdJtJ.get_inverse();  //Extract linealized uncertaintyes

    Vel=X.template slice<0,3>();           //Return the estimation
    W0=X.template slice<3,3>();

    RVel=RRV.slice<0,0,3,3>();             //And the uncertaintyes
    RW0=RRV.slice<3,3,3,3>();


    return F/F0;

}




}
