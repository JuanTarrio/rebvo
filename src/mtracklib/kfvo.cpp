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
        //kl.kf_id=kl.m_id_f;
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


double kfvo::OptimizeRelConstraint1Iter(keyframe &mkf, edge_tracker &et, const TooN::Matrix<3, 3> &BRelRot, const TooN::Vector<3> &BRelPos,Matrix <6,6> &JtJ,Vector<6>&JtF,bool etracket_2_keyframe){



    Matrix <Dynamic, 6> J(mkf.edges().KNum(),6);
    Vector <Dynamic>    f(mkf.edges().KNum());
    double E=0;
    for(int i=0;i<mkf.edges().KNum();i++){


        KeyLine  kl_b=mkf.edges()[i];

        if(kl_b.m_id_f<0){
            f[i]=0;
            J[i]=Zeros;
            continue;
        }
        KeyLine kl=et[kl_b.m_id_f];

        if(!etracket_2_keyframe)
            std::swap(kl_b,kl);


        Vector <3> Rx0=BRelRot*mkf.camera.unprojectHomCordVec(makeVector(kl.p_m.x,kl.p_m.y,kl.rho));
        Vector <3> q1=mkf.camera.projectHomCordVec(Rx0+BRelPos);



        f[i]=(q1[0]-kl_b.p_m.x)*kl_b.u_m.x+(q1[1]-kl_b.p_m.y)*kl_b.u_m.y;     //f=(q1-qm)*u


        Matrix <1,3> dfdx=Data( q1[2]*mkf.camera.zfm*kl_b.u_m.x,
                q1[2]*mkf.camera.zfm*kl_b.u_m.y,
                -q1[2]*q1[0]*kl_b.u_m.x-q1[2]*q1[1]*kl_b.u_m.y);



        E+=f[i]*f[i];
        double stddev=util::norm(1.0,((dfdx*BRelPos)*kl.s_rho/q1[2])[0]);
        f[i]/=stddev;
        J.slice(i,0,1,3)=dfdx/stddev;
        J.slice(i,3,1,3)=dfdx*util::crossMatrix(Rx0).T()/stddev;


    }

    JtJ=J.T()*J;
    JtF=J.T()*f;
    return E;
}
double kfvo::OptimizeRelContraint(keyframe &mkf, edge_tracker &et,const TooN::Matrix<3, 3> &Pose, const TooN::Vector<3> &Pos, const double &K, int iter_max,TooN::Vector<6> &X,TooN::Matrix<6, 6> &R_X,bool etracket_2_keyframe){



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


    double E=OptimizeRelConstraint1Iter(mkf,et,BRelRot,BRelPos,JtJ,JtF,etracket_2_keyframe);
    double E0=E;

    double v=2,tau=1e-3;
    double u=tau*TooN::max_element(JtJ).first;
    double gain;

    for(int iter=0;iter<iter_max;iter++){


        ApI=JtJ+Identity*u;
        Vector <6> dX=Cholesky<6>(ApI).backsub(-JtF);

        Vector <3> BRelPosNew=BRelPos+dX.slice<0,3>();
        Matrix <3,3>BRelRotNew=SO3<>(SO3<>(dX.slice<3,3>()).get_matrix()*BRelRot).get_matrix();



        double Enew=OptimizeRelConstraint1Iter(mkf,et,BRelRotNew,BRelPosNew,JtJnew,JtFnew,etracket_2_keyframe);

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

    return E/E0;
    //Actualizar consatraits
}


int kfvo::translateDepth(keyframe &mkf, edge_tracker &et, TooN::Matrix<3, 3> &Pose, TooN::Vector<3> &Pos,double &K){


    Matrix <3,3> BRelRot=mkf.Pose.T()*Pose;
    Vector <3> BRelPos=mkf.Pose.T()*(Pos-mkf.Pos);




    int count=0;

    for(KeyLine &kl:et){
        if(kl.m_id_f>=0){
            KeyLine &klf=mkf.edges()[kl.m_id_f];

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

int kfvo::translateDepthBack(keyframe &mkf, edge_tracker &et, TooN::Matrix<3, 3> &Pose, TooN::Vector<3> &Pos,double &K){




    Matrix <3,3> BRelRot=mkf.Pose.T()*Pose;
    Vector <3> BRelPos=mkf.Pose.T()*(Pos-mkf.Pos);


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
    }
}


int kfvo::forwardCorrectAugmentate(keyframe &mkf, edge_tracker &et, const TooN::Matrix<3, 3> &Pose, const TooN::Vector<3> &Pos,double dist_thesh,bool augmentate){


    Matrix <3,3> R=Pose.T()*mkf.Pose;
    Vector <3> t=mkf.Pose.T()*(Pos-mkf.Pos);
    Matrix <3,3> E=R*util::crossMatrix(t);


    double dist[mkf.edges().KNum()];

    for(int i=0;i<mkf.edges().KNum();i++)
        dist[i]=forwardStereoCorrect(et,mkf.edges()[i],mkf.edges()[i].m_id_f,E,dist_thesh,mkf.camera.zfm);

    if(augmentate)
        for(int i=0;i<mkf.edges().KNum();i++)
            forwardCorrectAugmentate(et,mkf,i,E,dist_thesh,mkf.camera.zfm);

    int count=0;
    for(int i=0;i<mkf.edges().KNum();i++){
        if(dist[i]>dist_thesh)
            mkf.edges()[i].m_id_f=-1;

        if(mkf.edges()[i].m_id_f>=0)
            count++;
    }
    return count;
}


void kfvo::forwardCorrectAugmentate(edge_tracker &et2match,keyframe &mkf,const int &kl_id,const TooN::Matrix<3,3> &E,const double &dist_thesh,const double &zf){

    KeyLine *kl=&mkf.edges()[kl_id];

    if(kl->m_id_f<0)
        return;


    while(1){
        if(kl->p_id<0)
            break;
        KeyLine *kln=&mkf.edges()[kl->p_id];
        if(kln->m_id_f>=0)
            break;
        kln->m_id_f=kl->m_id_f;
        if(forwardStereoCorrect(et2match,*kln,kln->m_id_f,E,dist_thesh,zf)>dist_thesh){
            kln->m_id_f=-1;
            break;
        }

        kl=kln;
    }

    kl=&mkf.edges()[kl_id];

    while(1){
        if(kl->n_id<0)
            break;

        KeyLine *kln=&mkf.edges()[kl->n_id];

        if(kln->m_id_f>=0)
            break;
        kln->m_id_f=mkf.edges()[kl_id].m_id_f;
        if(forwardStereoCorrect(et2match,*kln,kln->m_id_f,E,dist_thesh,zf)>dist_thesh){
            kln->m_id_f=-1;
            break;
        }

        kl=kln;
    }


}


double kfvo::forwardStereoCorrect(keyframe &mkf, edge_tracker &et, const TooN::Matrix<3, 3> &Pose, const TooN::Vector<3> &Pos,double dist_thesh)
{

    Matrix <3,3> R=Pose.T()*mkf.Pose;
    Vector <3> t=mkf.Pose.T()*(Pos-mkf.Pos);
    Matrix <3,3> E=R*util::crossMatrix(t);

    for(int i=0;i<mkf.edges().KNum();i++)
        forwardStereoCorrect(et,mkf.edges()[i],mkf.edges()[i].m_id_f,E,dist_thesh,mkf.camera.zfm);
    return 0;
}



double kfvo::forwardStereoCorrect(edge_tracker &et,KeyLine &kl,int &f_id,const TooN::Matrix<3,3> &E,double dist_thesh,const double &zf){

    if(f_id<0)
        return -1;



    TooN::Vector<3> e=E*TooN::makeVector(kl.p_m.x,kl.p_m.y,zf);
    TooN::Vector<3> r=e/util::norm(e[0],e[1]);
    r[2]*=zf;


    KeyLine *klm=&et[f_id];
    double d0=fabs(klm->p_m.x*r[0]+klm->p_m.y*r[1]+r[2]);

    if(d0<dist_thesh)
        return d0;

    if(klm->n_id>=0){
        KeyLine *klmn=&et[f_id];

        double d=fabs(klmn->p_m.x*r[0]+klmn->p_m.y*r[1]+r[2]);

        if(fabs(d)<fabs(d0)){

            while(1){

                f_id=klm->n_id;
                d0=d;

                if(d0<dist_thesh)
                    return d0;

                klm=&et[f_id];

                if(klm->n_id>=0){

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

    if(klm->p_id>=0){
        KeyLine *klmn=&et[f_id];
        double d=fabs(klmn->p_m.x*r[0]+klmn->p_m.y*r[1]+r[2]);
        if(d<d0){

            while(1){
                f_id=klm->p_id;
                d0=d;


                if(d0<dist_thesh)
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
