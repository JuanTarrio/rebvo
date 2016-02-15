#include "edge_tracker.h"
#include <iostream>

#include <stdio.h>

using namespace TooN;

edge_tracker::edge_tracker(float max_img_value, Point2DF pp, Point2DF zf, double Kc[2], Size2D sz)
    :edge_finder(sz,max_img_value,RhoInit,SRhoInit),cam_mod(pp,zf,Kc,sz),norm_dist(4)
{

    tag_inx=0;

}

void edge_tracker::build_kl_mask(){


    for(int ikl=0;ikl<kn;ikl++){

        Point2DF pi=cam_mod.Hom2Img(kl[ikl].p_m);

        int inx=util::round2int(pi.y)*cam_mod.sz.w+util::round2int(pi.x);

        img_mask_klr[inx]=ikl;

    }

}

void edge_tracker::rotate_klpos(KLPos &kl,TooN::Matrix <3,3> RotF,double zf){


    Vector <3>p;
    double r;
    p=RotF*makeVector(kl.p.x/zf,kl.p.y/zf,1);

    if(fabs(p[2])>0){

        r=p[2];

        kl.p.x=p[0]/r*zf;
        kl.p.y=p[1]/r*zf;

        kl.rho/=r;
        kl.s_rho/=r;

    }

    p=RotF*makeVector(kl.m.x,kl.m.y,0);
    kl.m.x=p[0];
    kl.m.y=p[1];

}


void edge_tracker::rotate_keylines(TooN::Matrix <3,3> RotF,double zf){


    Vector <3>p;

    double r;

    for(int i=0;i<kn;i++){

        kl[i].P0=makeVector(kl[i].p_m.x/zf,kl[i].p_m.y/zf,1);

        p=RotF*kl[i].P0;

        kl[i].P0/=kl[i].rho;

        if(fabs(p[2])>0){

            r=p[2];

            kl[i].p_m.x=p[0]/r*zf;
            kl[i].p_m.y=p[1]/r*zf;


            kl[i].rho/=r;
            kl[i].s_rho/=r;

        }

        p=RotF*makeVector(kl[i].m_m.x,kl[i].m_m.y,0);
        kl[i].m_m.x=p[0];
        kl[i].m_m.y=p[1];


    }

}


void edge_tracker::BackRotate_keylines(TooN::Matrix <3,3> RotF){


    Vector <3>p;

    double r;

    for(int i=0;i<kn;i++){


        p=RotF*makeVector(kl[i].p_m.x,kl[i].p_m.y,cam_mod.zfm);

        if(fabs(p[2])>0){

            r=cam_mod.zfm/p[2];

            kl[i].p_m.x=p[0]*r;
            kl[i].p_m.y=p[1]*r;

        }

    }

}


void edge_tracker::LimitRho(double &rho,double &s_rho){

    if(rho<RHO_MIN){
        //kl[kl_inx].s_rho+=RHO_MIN-kl[kl_inx].rho;
        rho=RHO_MIN;
        s_rho=1e3;
    }else if(rho>RHO_MIN){
        // kl[kl_inx].s_rho+=kl[kl_inx].rho-RHO_MAX;
        rho=RHO_MIN;
        s_rho=1e3;
    }else if(isnan(rho) || isnan(s_rho) || isinf(rho) || isinf(s_rho)){
        rho=RhoInit;
        s_rho=1e3;
    }

}

void edge_tracker::LimitZ(double &rho,double &s_rho){


}

void edge_tracker::AddMeas(TooN::Vector <3> vel,TooN::Matrix <3,3> RVel,TooN::Matrix <3,3> RotF,edge_tracker *et0,double zf){

    et0->vlist.CopyTo(&vlist);

    for(int i=0;i<vlist.Size();i++){

        vlist[i]=RotF*vlist[i];

    }

    vlist.Put(vel);

    for(int ikl=0;ikl<kn;ikl++){

        KLPos pos;

        pos.p=cam_mod.Img2Hom(kl[ikl].c_p);

        //cam_mod.UndistortHom2Hom(pos.p,3);

        pos.m=kl[ikl].m_m0;

        pos.rho=RhoInit;
        pos.s_rho=SRhoInit;

        if(kl[ikl].plist0==0){

            kl[ikl].plist.Init(pos);

        }else{

            kl[ikl].plist0->CopyTo(&kl[ikl].plist);

            for(int ip=0;ip<kl[ikl].plist.Size();ip++){


                Vector <3>p3d;

                double r;

                p3d=RotF*makeVector(kl[ikl].plist[ip].p.x/zf,kl[ikl].plist[ip].p.y/zf,1);

                if(fabs(p3d[2])>0){
                    r=p3d[2];
                    kl[ikl].plist[ip].p.x=p3d[0]/r*zf;
                    kl[ikl].plist[ip].p.y=p3d[1]/r*zf;

                    kl[ikl].plist[ip].rho/=r;
                    kl[ikl].plist[ip].s_rho=kl[ikl].plist[ip].s_rho/r;//+kl[ikl].plist.data[ip].rho*EKF_SGIRO;

                }else{
                    printf("\nZero %d %d",ikl,ip);
                }

                p3d=RotF*makeVector(kl[ikl].plist[ip].m.x,kl[ikl].plist[ip].m.y,0);
                kl[ikl].plist[ip].m.x=p3d[0];
                kl[ikl].plist[ip].m.y=p3d[1];

            }

            kl[ikl].plist.Put(pos);

        }



    }


}



int edge_tracker::Regularize_1_iter(double thresh)
{

    int r_num=0;

    double r[kn],s[kn];
    bool set[kn];
    for(int ikl=0;ikl<kn;ikl++){

        set[ikl]=false;

        if(kl[ikl].n_id <0 || kl[ikl].p_id<0)
            continue;

        KeyLine &k=kl[ikl];
        KeyLine &kn=kl[kl[ikl].n_id];
        KeyLine &kp=kl[kl[ikl].p_id];


        if(fabs(kn.rho-kp.rho)>(kn.s_rho+kp.s_rho))
            continue;


        double alpha=(kn.m_m.x*kp.m_m.x+kn.m_m.y*kp.m_m.y)/(kn.n_m*kp.n_m);

        if(alpha-thresh<0)
            continue;

        alpha=(alpha-thresh)/(1-thresh);

        double wr=1/(k.s_rho*k.s_rho);
        double wrn=alpha/(kn.s_rho*kn.s_rho);
        double wrp=alpha/(kp.s_rho*kp.s_rho);

        //k.rho=(k.rho*wr+kn.rho*wrn+kp.rho*wrp)/(wr+wrn+wrp);
        //k.s_rho=(k.s_rho*wr+kn.s_rho*wrn+kp.s_rho*wrp)/(wr+wrn+wrp);


        r[ikl]=(k.rho*wr+kn.rho*wrn+kp.rho*wrp)/(wr+wrn+wrp);

        s[ikl]=(k.s_rho*wr+kn.s_rho*wrn+kp.s_rho*wrp)/(wr+wrn+wrp);

        set[ikl]=true;

        r_num++;

    }
    for(int ikl=0;ikl<kn;ikl++){
        if(set[ikl]){
            kl[ikl].rho=r[ikl];
            kl[ikl].s_rho=s[ikl];
        }
    }
    return r_num;

}


double edge_tracker::CalcDephs(double zf,double &RKp,TooN::Matrix <3,3> RVel)
{


    static int frame=0;

    int np=0;
    double a,b,n_ab;
    int maxnp=0;

    double sum_z=0;
    double mass_z=0;
    int maxi=-1;

    for(int ikl=0;ikl<kn;ikl++){

        np=kl[ikl].plist.Size();

        if(np<2){
                kl[ikl].batch_z=1;
                kl[ikl].batch_vz=1e6;
        }else{



            //printf("\n%d %d %d\n",ikl,np,vlist.num);

            if(util::keep_max(maxnp,np))
                maxi=ikl;


            Matrix <> A(2,np-1);


            Vector <> DQ(np-1);
            Vector <> Q(np-1);

            double dz=0;

            for(int ip=0;ip<np-1;ip++){

                KLPos *pos=&kl[ikl].plist.GetRelative(ip), *pos0=&kl[ikl].plist.GetRelative(ip+1);


                Vector<3> vel=vlist.GetRelative(ip);


                a=pos->m.x;
                b=pos->m.y;
                n_ab=util::norm(a,b);
                a/=n_ab;
                b/=n_ab;


                double dq= a*(pos->p.x-pos0->p.x)+b*(pos->p.y-pos0->p.y);

                double dv= a*(vel[0]*zf-vel[2]*pos0->p.x)+b*(vel[1]*zf-vel[2]*pos0->p.y);

                A(0,ip)=dq;
                A(1,ip)=dv+dq*dz;

                DQ[ip]=dq;
                Q[ip]=dv+dq*dz;

                dz+=vel[2];


               // printf("%d (%f %f) (%f %f) (%f %f %f) (%f %f %f)\n",ip,pos->p.x,pos->p.y,pos0->p.x,pos0->p.y,dq,dv,dz,vel[0],vel[1],vel[2]);

            }

            double v_rho=(1/(Q*Q));
            double rho=v_rho*(Q*DQ);


            kl[ikl].batch_z=1/rho;
            kl[ikl].batch_vz=v_rho/3/(1/kl[ikl].batch_z/kl[ikl].batch_z+v_rho)*kl[ikl].batch_z*kl[ikl].batch_z;



            double s_rho=sqrt(v_rho);

            LimitRho(rho,s_rho);


            {
                Vector<3> vel=vlist.GetRelative(0);

                double z0=1/kl[ikl].rho;

                double v_z0=(z0*z0)*(z0*z0)*(kl[ikl].s_rho*kl[ikl].s_rho);

                double z1=z0+vel[2];

                if(z1<1/RHO_MIN){
                    z1=1/RHO_MIN;
                }

                double v_z1=(v_z0+RVel(2,2));

                kl[ikl].rho=1/z1;
                kl[ikl].s_rho=sqrt(v_z1)*kl[ikl].rho*kl[ikl].rho;
            }

            if(kl[ikl].s_rho/kl[ikl].rho<s_rho/rho){
                rho=kl[ikl].rho;
                s_rho=kl[ikl].s_rho;
            }

            kl[ikl].rho=rho;
            kl[ikl].s_rho=s_rho;

            sum_z+=rho*rho/(s_rho*s_rho);
            mass_z+=1/(s_rho*s_rho);



        }


    }



    if(mass_z<=0){
        RKp=1e-3;
        return 1;
    }

    double mean_z=sqrt(sum_z/mass_z);
    double K=mean_z;

    RKp=1/mass_z;

  // K=1;
   // RKp=0.001;

    if(isnan(K) || isinf(K)|| K<=0)
        K=1;

    for(int ikl=0;ikl<kn;ikl++){



        kl[ikl].plist.GetNewest().rho=kl[ikl].rho;
        kl[ikl].plist.GetNewest().s_rho=kl[ikl].s_rho;

        kl[ikl].rho/=K;
        kl[ikl].s_rho=kl[ikl].s_rho/K+ RKp*kl[ikl].rho;

        for(int ip=0;ip<kl[ikl].plist.Size();ip++){
            kl[ikl].plist[ip].rho/=K;
            kl[ikl].plist[ip].s_rho/=K;
        }


        kl[ikl].batch_z*=K;
       kl[ikl].batch_vz*=K*K;

    }

    for(int i=0;i<vlist.Size();i++){

        vlist[i]=K*vlist[i];

    }



        frame++;
    return 1/K;


}


template <bool rotate,bool test_rho>
int edge_tracker::search_match(KeyLine &k,TooN::Vector <3> Vel,TooN::Matrix <3,3> RVel,TooN::Matrix <3,3> BackRot,double min_score,double max_radius){

    const double cang_min_edge=cos(30*M_PI/180.0);

    double dr_min,dr_max,t_x,t_y,dr_rho=0;

    int t_steps=0;

    Point2DF p_m;
    double k_rho;

    if(rotate){

        Vector <3> p_m3=BackRot*makeVector(k.p_m.x,k.p_m.y,cam_mod.zfm);


        p_m.x=p_m3[0]*cam_mod.zfm/p_m3[2];
        p_m.y=p_m3[1]*cam_mod.zfm/p_m3[2];

        k_rho=k.rho*cam_mod.zfm/p_m3[2];
    }else{

        p_m=k.p_m;
        k_rho=k.rho;

    }


    Point2DF pi,pi0=cam_mod.Hom2Img(p_m);


    t_x=-(Vel[0]*cam_mod.zfm-Vel[2]*p_m.x);
    t_y=-(Vel[1]*cam_mod.zfm-Vel[2]*p_m.y);

    double dr=util::norm(t_x,t_y),v_dr0;

    Vector <3> DrDv=makeVector(cam_mod.zfm,cam_mod.zfm,-p_m.x-p_m.y);

    v_dr0=(DrDv.as_row()*RVel*DrDv.as_col())(0,0);


    if(dr>1e-6){
        t_x/=dr;
        t_y/=dr;

        dr_rho=dr*k_rho;

        dr_min=std::max(0.0,dr*(k_rho-k.s_rho))-ERR_DQ;
        dr_max=std::min(max_radius,dr*(k_rho+k.s_rho))+ERR_DQ;

        if(dr_rho>dr_max){
            dr_rho=(dr_max+dr_min)/2;
            t_steps=util::round2int(dr_rho);
        }else{
            t_steps=util::round2int(std::max(dr_max-dr_rho,dr_rho-dr_min));
        }
    }else if(dr<0){
        printf("\nWTFFFFF\n");
        dr_min=0;
        dr_max=max_radius;
    }else{
        t_x=k.m_m.x;
        t_y=k.m_m.y;
        dr=k.n_m;
        t_x/=dr;
        t_y/=dr;
        dr=1;
        dr_min=-max_radius-ERR_DQ;
        dr_max=max_radius+ERR_DQ;

        dr_rho=0;
        t_steps=dr_max;


    }




    double norm_m=k.n_m;

    int j,inx;

    double tn=dr_rho,tp=dr_rho+1;

    for(int t_i=0;t_i<t_steps;t_i++,tp+=1,tn-=1){


        for(int i_inx=0;i_inx<2;i_inx++){

            double t;

            if(i_inx){
                t=tp;
                if(t>dr_max)
                    continue;
            }else{
                t=tn;
                if(t<dr_min)
                    continue;
            }


            double dq=t_x*t;
            double dp=t_y*t;

            pi.x=(dq+pi0.x);
            pi.y=(dp+pi0.y);


            if(pi.x<0 || pi.x>=fsz.w-2 || pi.y<0 || pi.y>=fsz.h-2)
                continue;

            inx=round(pi.y)*cam_mod.sz.w+round(pi.x);//+sum_inx[i_inx];

            if((j=img_mask_kl[inx])>=0){


                double norm_m0=kl[j].n_m;
                double cang=(kl[j].m_m.x*k.m_m.x+kl[j].m_m.y*k.m_m.y)/(norm_m0*norm_m);

                if(cang<cang_min_edge || fabs(norm_m0/norm_m-1)>min_score)
                    continue;

                /*                if(fabs((kl[j].m_m.x*k.m_m.x+kl[j].m_m.y*k.m_m.y)-kl[j].n_m*kl[j].n_m)>min_score*kl[j].n_m*kl[j].n_m)
                    continue;*/

                if(test_rho){

                    double dr_m=t;
                    // dr_m=util::norm(edge_mask_p[inx].x-pi0.x,edge_mask_p[inx].y-pi0.y);

                    double &s_rho=kl[j].s_rho;
                    double &rho=kl[j].rho;
                    double v_rho_dr=(ERR_DQ*ERR_DQ+s_rho*s_rho*dr*dr+v_dr0*rho*rho);

                    //Comprueba que el match convalide con el modelo
                    if((dr_m-dr*rho)*(dr_m-dr*rho)>v_rho_dr)
                        continue;
                }



                return j;


            }

        }

    }

    return -1;


}

int edge_tracker::directed_matching(TooN::Vector <3> Vel,TooN::Matrix <3,3> RVel,TooN::Matrix <3,3> BackRot,edge_tracker *et0,double min_score,double max_radius,bool clear){



    nmatch=0;


    int i_mch;

    for(int i_kn=0;i_kn<kn;i_kn++){


        if(clear){
            kl[i_kn].m_id=-1;
            kl[i_kn].m_num=0;
            kl[i_kn].plist0=0;
        }

        if((i_mch=et0->search_match<true,true>(kl[i_kn],Vel,RVel,BackRot,min_score,max_radius))<0)
                continue;



        kl[i_kn].rho=et0->kl[i_mch].rho;
        kl[i_kn].s_rho=et0->kl[i_mch].s_rho;


        kl[i_kn].m_id=i_mch;
        kl[i_kn].m_num=et0->kl[i_mch].m_num+1;

        kl[i_kn].p_m_0=et0->kl[i_mch].p_m;
        kl[i_kn].m_m0=et0->kl[i_mch].m_m;
        kl[i_kn].n_m0=et0->kl[i_mch].n_m;
        kl[i_kn].plist0=&et0->kl[i_mch].plist;

        nmatch++;


    }


    return nmatch;

}


int edge_tracker::forward_directed_matching(TooN::Vector <3> Vel,TooN::Matrix <3,3> RVel,edge_tracker *et,double min_score,double max_radius,bool clear){



    et->nmatch=0;

    if(clear){
        for(int ikl=0;ikl<et->kn;ikl++){

            et->kl[ikl].m_id=-1;
            et->kl[ikl].m_num=0;
            et->kl[ikl].plist0=0;
        }
    }

    int ikl_f;


    for(int i_kn=0;i_kn<kn;i_kn++){

        KeyLine &k=kl[i_kn];

        if((ikl_f=et->search_match<false,false>(k,-Vel,RVel,Identity,min_score,max_radius))<0)
            continue;

        et->kl[ikl_f].rho=k.rho;
        et->kl[ikl_f].s_rho=k.s_rho;

        et->kl[ikl_f].m_num=k.m_num+1;

        et->kl[ikl_f].m_id=i_kn;

        et->kl[ikl_f].p_m_0=k.p_m;

        et->kl[ikl_f].m_m0=k.m_m;
        et->kl[ikl_f].n_m0=k.n_m;

        et->kl[ikl_f].plist0=&k.plist;

        et->nmatch++;


    }


    return et->nmatch;

}

int edge_tracker::FordwardMatch(edge_tracker *et,bool clear){



    if(clear){
        for(int ikl=0;ikl<et->kn;ikl++){

            et->kl[ikl].m_id=-1;
            et->kl[ikl].m_num=0;
            et->kl[ikl].plist0=0;
        }
    }

    for(int ikl=0;ikl<kn;ikl++){

        KeyLine &k=kl[ikl];



        int ikl_f=k.m_id_f;
        if(ikl_f<0)
            continue;

        et->kl[ikl_f].rho=k.rho;
        et->kl[ikl_f].s_rho=k.s_rho;

        et->kl[ikl_f].m_num=k.m_num+1;

        et->kl[ikl_f].m_id=ikl;

        et->kl[ikl_f].p_m_0=k.p_m;

        et->kl[ikl_f].m_m0=k.m_m;
        et->kl[ikl_f].n_m0=k.n_m;

        et->kl[ikl_f].plist0=&k.plist;



    }

  /*  for(int ikl=0;ikl<et->kn;ikl++){
        if(et->kl[ikl].m_id>0)
            nmatch++;
    }


    printf("\nFordward Matcher: %d\n",nmatch);*/

}


void edge_tracker::UpdateInverseDepthKalman(Vector <3> vel, Matrix <3,3> RVel, TooN::Matrix<3, 3> RW0, double ReshapeFactor,double RelError){

    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            if(isnan(RVel(i,j)))
            {
                std::cout << "\nNAN RVel\n";
                return;
            }
        }
    }



    for(int i=0;i<kn;i++){


#ifdef USE_ONLY_EDGES
        if(!kl[i].is_edge)
            continue;
#endif

        if(kl[i].m_id>=0){
            //UpdateInverseDepthKalmanUniformMix(vel,RVel,RW0,ReshapeFactor,i);
            //UpdateInverseDepthKalmanU_TNorm(vel,RVel,RW0,ReshapeFactor,i);
            //UpdateInverseDepthIterativeKalman2(vel,RVel,RelError,ReshapeFactor,i);
            UpdateInverseDepthKalman(vel,RVel,RelError,ReshapeFactor,i);
        }


    }

}

double edge_tracker::UpdateInverseDepthKalman(Vector <3> vel,Matrix <3,3> RVel, double RelError, double ReshapeFactor, int kl_inx){

    double qx,qy,q0x,q0y,v_rho;

    double zf=cam_mod.zfm;

    if(kl[kl_inx].s_rho<0 || kl[kl_inx].rho<0){

        std::cout << "\nUIDK panic! "<<kl[kl_inx].s_rho<<" "<<kl[kl_inx].rho<<"\n";

    }

    kl[kl_inx].s_rho0=kl[kl_inx].s_rho;


    Point2DF pu=kl[kl_inx].p_m;

   // cam_mod.UndistortHom2Hom(pu,3);
    qx=pu.x;
    qy=pu.y;

    pu=kl[kl_inx].p_m_0;

   // cam_mod.UndistortHom2Hom(pu,3);
    q0x=pu.x;
    q0y=pu.y;


    v_rho=kl[kl_inx].s_rho*kl[kl_inx].s_rho;

    double a,b;

    a=kl[kl_inx].m_m0.x;
    b=kl[kl_inx].m_m0.y;
    a/=kl[kl_inx].n_m0;
    b/=kl[kl_inx].n_m0;

    Matrix <1,1> Y;
    Matrix <1,1> H;


    Y(0,0) = a*(qx-q0x)+b*(qy-q0y);

    H(0,0)= a*(vel[0]*zf-vel[2]*q0x)+b*(vel[1]*zf-vel[2]*q0y);

    double rho_p=1/(1.0/kl[kl_inx].rho+vel[2]);

    kl[kl_inx].rho0=rho_p;

    double F=1/(1+kl[kl_inx].rho*vel[2]);
    F=F*F;

    double p_p=F*v_rho*F+(kl[kl_inx].rho*EKF_SGIRO)*(kl[kl_inx].rho*EKF_SGIRO)+rho_p*rho_p*RVel(2,2)*rho_p*rho_p+ReshapeFactor;


    Matrix <1,1> e=Y-H*rho_p;

    Matrix <1,6> Mk;

    Mk[0]=makeVector(-1,a*rho_p*zf,b*rho_p*zf,-rho_p*(a*q0x+b*q0y),a*(rho_p*vel[2]),b*(rho_p*vel[2]));


    Matrix <6,6> R = Zeros;


    R(0,0)=EKF_DQ;
    R(4,4)=EKF_DQ;
    R(5,5)=EKF_DQ;
    R.slice <1,1,3,3> () =vel.as_diagonal()*vel.as_diagonal();
    R.slice <1,1,3,3> ()=R.slice <1,1,3,3> ()*RelError*RelError+RVel;

    Matrix <1,1> S=H*p_p*H.T()+Mk*R*Mk.T();

    Matrix <1,1> K=p_p*H.T()*(1/S(0,0));

    kl[kl_inx].rho=rho_p+(K*e)(0,0);

    v_rho=(1-(K*H)(0,0))*p_p;

    kl[kl_inx].s_rho=sqrt(v_rho);

    if(kl[kl_inx].rho<RHO_MIN){
        kl[kl_inx].s_rho+=RHO_MIN-kl[kl_inx].rho;
        kl[kl_inx].rho=RHO_MIN;
    }else if(kl[kl_inx].rho>RHO_MAX){
       // kl[kl_inx].s_rho+=kl[kl_inx].rho-RHO_MAX;
        kl[kl_inx].rho=RHO_MAX;
    }else if(isnan(kl[kl_inx].rho) || isnan(kl[kl_inx].s_rho) || isinf(kl[kl_inx].rho) || isinf(kl[kl_inx].s_rho)){
        std::cout<<"\nKL EKF Nan Rho!"<<rho_p<< " " << p_p<< " "<<vel[2]<<"\n";
        kl[kl_inx].rho=Rho0;
        kl[kl_inx].s_rho=RHO_MAX;
    }else if(kl[kl_inx].s_rho<0){
        std::cout<<"\nKL EKF Panic! Neg VRho!"<<vel<<RVel <<kl[kl_inx].rho << kl[kl_inx].s_rho <<"\n";
        kl[kl_inx].rho=Rho0;
        kl[kl_inx].s_rho=RHO_MAX;

    }

    //kl[kl_inx].rho*=1.000124002531646;
    //kl[kl_inx].s_rho*=1.000124002531646;
    return kl[kl_inx].rho;


}


double edge_tracker::UpdateInverseDepthKalmanU(Vector <3> vel,Matrix <3,3> RVel,int kl_inx){

    double qx,qy,q0x,q0y,v_rho,r0;

    double zf=cam_mod.zfm;

    if(kl[kl_inx].s_rho<0 || kl[kl_inx].rho<0){

        std::cout << "\nUIDK panic! "<<kl[kl_inx].s_rho<<" "<<kl[kl_inx].rho<<"\n";

    }

    kl[kl_inx].s_rho0=kl[kl_inx].s_rho;
    kl[kl_inx].rho0=kl[kl_inx].rho;

    Point2DF pu=kl[kl_inx].p_m;

    //cam_mod.UndistortHom2Hom(pu,3);
    qx=pu.x;
    qy=pu.y;

    pu=kl[kl_inx].p_m_0;

    //cam_mod.UndistortHom2Hom(pu,3);
    q0x=pu.x;
    q0y=pu.y;


    v_rho=kl[kl_inx].s_rho*kl[kl_inx].s_rho;

    double a,b;

    a=kl[kl_inx].m_m0.x;
    b=kl[kl_inx].m_m0.y;
    a/=kl[kl_inx].n_m0;
    b/=kl[kl_inx].n_m0;




    r0=kl[kl_inx].rho;
    double rho_p=1/(1.0/kl[kl_inx].rho+vel[2]);

    double F=1/(1+kl[kl_inx].rho*vel[2]);
    F=F*F;


    double p_p=F*v_rho*F +(kl[kl_inx].rho*EKF_SGIRO)*(kl[kl_inx].rho*EKF_SGIRO)+rho_p*rho_p*RVel(2,2)*rho_p*rho_p;




    double mean_rh=a*(vel[0]*zf-vel[2]*q0x)+b*(vel[1]*zf-vel[2]*q0y);


    Matrix <1,5> Mk;

    Mk[0]=makeVector(a*zf,b*zf,-(a*q0x+b*q0y),a*(vel[2]),b*(vel[2]));

    Matrix <5,5> R = Zeros;

    R(3,3)=EKF_DQ;
    R(4,4)=EKF_DQ;
    R.slice <0,0,3,3> () =RVel;

    double dev_rh=sqrt((Mk*R*Mk.T())(0,0));

    util::GaussReciprocal(mean_rh,dev_rh,1,21);

    double dq=a*(qx-q0x)+b*(qy-q0y);

    Matrix <1,1> Y;
    Matrix <1,1> H;

    Y(0,0) = dq*mean_rh;

    H(0,0)= 1;

    Matrix <1,1> e=Y-H*rho_p;

    Matrix <1,1> R0;

    R0(0,0)=util::norm2(dq*dev_rh,mean_rh*EKF_DQ);

    Matrix <1,1> S=H*p_p*H.T()+R0;

    Matrix <1,1> K=p_p*H.T()*(1/S(0,0));

    kl[kl_inx].rho=rho_p+(K*e)(0,0);

    v_rho=(1-(K*H)(0,0))*p_p;

    kl[kl_inx].s_rho=sqrt(v_rho);

    if(kl[kl_inx].rho<RHO_MIN){
        kl[kl_inx].s_rho+=RHO_MIN-kl[kl_inx].rho;
        kl[kl_inx].rho=RHO_MIN;
    }else if(kl[kl_inx].rho>RHO_MAX){
       // kl[kl_inx].s_rho+=kl[kl_inx].rho-RHO_MAX;
        kl[kl_inx].rho=RHO_MAX;
    }else if(isnan(kl[kl_inx].rho) || isnan(kl[kl_inx].s_rho) || isinf(kl[kl_inx].rho) || isinf(kl[kl_inx].s_rho)){
        std::cout<<"\nKL EKF Nan Rho!"<<rho_p<< " " << p_p<< " "<<vel[2]<< " "<<r0<<"\n";
        kl[kl_inx].rho=Rho0;
        kl[kl_inx].s_rho=RHO_MAX;
    }else if(kl[kl_inx].s_rho<0){
        std::cout<<"\nKL EKF Panic! Neg VRho!"<<vel<<RVel <<kl[kl_inx].rho << kl[kl_inx].s_rho <<"\n";
        kl[kl_inx].rho=Rho0;
        kl[kl_inx].s_rho=RHO_MAX;

    }

    return kl[kl_inx].rho;


}



double edge_tracker::UpdateInverseDepthKalmanU_TNorm(Vector <3> vel,Matrix <3,3> RVel,Matrix <3,3> RW0,double ReshapeFactor,int kl_inx){

    double qx,qy,q0x,q0y,v_rho;

    double zf=cam_mod.zfm;

    if(kl[kl_inx].s_rho<0 || kl[kl_inx].rho<0){

        std::cout << "\nUIDK panic! "<<kl[kl_inx].s_rho<<" "<<kl[kl_inx].rho<<"\n";

    }

    kl[kl_inx].s_rho0=kl[kl_inx].s_rho;
    kl[kl_inx].rho0=kl[kl_inx].rho;

    Point2DF pu=kl[kl_inx].p_m;

    //cam_mod.UndistortHom2Hom(pu,3);
    qx=pu.x;
    qy=pu.y;

    pu=kl[kl_inx].p_m_0;

    //cam_mod.UndistortHom2Hom(pu,3);
    q0x=pu.x;
    q0y=pu.y;


    v_rho=kl[kl_inx].s_rho*kl[kl_inx].s_rho;

    double a,b;

    a=kl[kl_inx].m_m0.x;
    b=kl[kl_inx].m_m0.y;
    a/=kl[kl_inx].n_m0;
    b/=kl[kl_inx].n_m0;


    double rho_p=1/(1.0/kl[kl_inx].rho+vel[2]);

    double F=1/(1+kl[kl_inx].rho*vel[2]);
    F=F*F;

    double err_w=RW0(0,0)+RW0(1,0)+RW0(0,1)+RW0(1,1);

    double p_p=F*v_rho*F +(kl[kl_inx].rho*err_w*kl[kl_inx].rho)+rho_p*rho_p*RVel(2,2)*rho_p*rho_p+ReshapeFactor;

    double mean_rh=a*(vel[0]*zf-vel[2]*q0x)+b*(vel[1]*zf-vel[2]*q0y);

    Matrix <1,5> Mk;

    Mk[0]=makeVector(a*zf,b*zf,-(a*q0x+b*q0y),a*(vel[2]),b*(vel[2]));

    Matrix <5,5> R = Zeros;

    R(3,3)=EKF_DQ;
    R(4,4)=EKF_DQ;
    R.slice <0,0,3,3> () =RVel;

    double dev_rh=sqrt((Mk*R*Mk.T())(0,0));

    if(true || fabs(mean_rh)>dev_rh*0.005){


       // double mean_rh0=mean_rh;
       // double dev_rh0=dev_rh;

        norm_dist.EvalReciprocal<false>(mean_rh,dev_rh,1,21);

/*
        util::GaussReciprocal(mean_rh0,dev_rh0,1,21);

        if(fabs(mean_rh0-mean_rh)>(1e-2)*fabs(mean_rh) || fabs(dev_rh0-dev_rh)>(1e-2)*fabs(dev_rh)){
            printf("\n GAUSS!!!! %f %f %f %f\n",fabs(mean_rh0-mean_rh),(1e-2)*fabs(mean_rh),fabs(dev_rh0-dev_rh),(1e-2)*fabs(dev_rh));
        }*/

        double dq=a*(qx-q0x)+b*(qy-q0y);

        double Y;
        double H;

        Y = dq*mean_rh;

        H= 1;

        double e=Y-H*rho_p;

        double R0;

        R0=util::norm2(dq*dev_rh,mean_rh*EKF_DQ,EKF_DQ*dev_rh);

        double S=H*p_p*H+R0;

        double K=p_p*H*(1/S);

        kl[kl_inx].rho=rho_p+(K*e);

        v_rho=(1-(K*H))*p_p;

        kl[kl_inx].s_rho=sqrt(v_rho);

    }else{


        kl[kl_inx].rho=rho_p;

        kl[kl_inx].s_rho=sqrt(p_p);

    }

    if(kl[kl_inx].rho<RHO_MIN){
        kl[kl_inx].s_rho+=RHO_MIN-kl[kl_inx].rho;
        kl[kl_inx].rho=RHO_MIN;
    }else if(kl[kl_inx].rho>RHO_MAX){
       // kl[kl_inx].s_rho+=kl[kl_inx].rho-RHO_MAX;
        kl[kl_inx].rho=RHO_MAX;
    }else if(isnan(kl[kl_inx].rho) || isnan(kl[kl_inx].s_rho) || isinf(kl[kl_inx].rho) || isinf(kl[kl_inx].s_rho)){
        std::cout<<"\nKL EKF Nan Rho!"<<rho_p<< " " << p_p<< " "<<vel[2]<< "\n";
        kl[kl_inx].rho=Rho0;
        kl[kl_inx].s_rho=RHO_MAX;
    }else if(kl[kl_inx].s_rho<0){
        std::cout<<"\nKL EKF Panic! Neg VRho!"<<vel<<RVel <<kl[kl_inx].rho << kl[kl_inx].s_rho <<"\n";
        kl[kl_inx].rho=Rho0;
        kl[kl_inx].s_rho=RHO_MAX;

    }

    return kl[kl_inx].rho;


}


double edge_tracker::UpdateInverseDepthIterativeKalman(Vector <3> vel,Matrix <3,3> RVel, double RelError, double ReshapeFactor, int kl_inx){

    double qx,qy,q0x,q0y,v_rho;

    double zf=cam_mod.zfm;

    if(kl[kl_inx].s_rho<0 || kl[kl_inx].rho<0){

        std::cout << "\nUIDK panic! "<<kl[kl_inx].s_rho<<" "<<kl[kl_inx].rho<<"\n";

    }

    kl[kl_inx].s_rho0=kl[kl_inx].s_rho;
    kl[kl_inx].rho0=kl[kl_inx].rho;

    Point2DF pu=kl[kl_inx].p_m;

   // cam_mod.UndistortHom2Hom(pu,3);
    qx=pu.x;
    qy=pu.y;

    pu=kl[kl_inx].p_m_0;

   // cam_mod.UndistortHom2Hom(pu,3);
    q0x=pu.x;
    q0y=pu.y;


    v_rho=kl[kl_inx].s_rho*kl[kl_inx].s_rho;

    double a,b;

    a=kl[kl_inx].m_m0.x;
    b=kl[kl_inx].m_m0.y;
    a/=kl[kl_inx].n_m0;
    b/=kl[kl_inx].n_m0;

    double Y;
    double H;


    Y = a*(qx-q0x)+b*(qy-q0y);

    H= a*(vel[0]*zf-vel[2]*q0x)+b*(vel[1]*zf-vel[2]*q0y);

    double rho_p=1/(1.0/kl[kl_inx].rho+vel[2]);

    double F=1/(1+kl[kl_inx].rho*vel[2]);
    F=F*F;

    double p_p=F*v_rho*F+(kl[kl_inx].rho*EKF_SGIRO)*(kl[kl_inx].rho*EKF_SGIRO)+rho_p*rho_p*RVel(2,2)*rho_p*rho_p+ReshapeFactor;


    Matrix <6,6> R = Zeros;
    R(0,0)=EKF_DQ;
    R(4,4)=EKF_DQ;
    R(5,5)=EKF_DQ;
    R.slice <1,1,3,3> () =vel.as_diagonal()*vel.as_diagonal();
    R.slice <1,1,3,3> ()=R.slice <1,1,3,3> ()*RelError*RelError+RVel;

    double rho_i=rho_p;
    double R0,Vn;

    for(int i=0;i<10;i++){
        Matrix <1,6> Mk;
        Mk[0]=makeVector(-1,a*rho_i*zf,b*rho_i*zf,-rho_i*(a*q0x+b*q0y),a*(rho_i*vel[2]),b*(rho_i*vel[2]));
        R0=(Mk*R*Mk.T())(0,0);

        Matrix <2,1> G;
        G(0,0)=1;
        G(1,0)=H;

        Matrix <2,2> iC=inv(makeVector(p_p,R0).as_diagonal());

        Matrix <2,1> Z;
        Z(0,0)=rho_p;
        Z(1,0)=Y;

        Vn=1.0/(G.T()*iC*G)(0,0);

        rho_i=(Vn*(G.T()*iC)*Z)(0,0);
    }

    kl[kl_inx].rho=rho_i;

    kl[kl_inx].s_rho=sqrt(Vn);

    if(kl[kl_inx].rho<RHO_MIN){
        kl[kl_inx].s_rho+=RHO_MIN-kl[kl_inx].rho;
        kl[kl_inx].rho=RHO_MIN;
    }else if(kl[kl_inx].rho>RHO_MAX){
       // kl[kl_inx].s_rho+=kl[kl_inx].rho-RHO_MAX;
        kl[kl_inx].rho=RHO_MAX;
    }else if(isnan(kl[kl_inx].rho) || isnan(kl[kl_inx].s_rho) || isinf(kl[kl_inx].rho) || isinf(kl[kl_inx].s_rho)){
        std::cout<<"\nKL EKF Nan Rho!"<<rho_p<< " " << p_p<< " "<<vel[2]<<"\n";
        kl[kl_inx].rho=Rho0;
        kl[kl_inx].s_rho=RHO_MAX;
    }else if(kl[kl_inx].s_rho<0){
        std::cout<<"\nKL EKF Panic! Neg VRho!"<<vel<<RVel <<kl[kl_inx].rho << kl[kl_inx].s_rho <<"\n";
        kl[kl_inx].rho=Rho0;
        kl[kl_inx].s_rho=RHO_MAX;

    }

    return kl[kl_inx].rho;


}


double edge_tracker::UpdateInverseDepthIterativeKalman2(Vector <3> vel,Matrix <3,3> RVel, double RelError, double ReshapeFactor, int kl_inx){

    double qx,qy,q0x,q0y;

    double zf=cam_mod.zfm;

    if(kl[kl_inx].s_rho<0 || kl[kl_inx].rho<0){

        std::cout << "\nUIDK panic! "<<kl[kl_inx].s_rho<<" "<<kl[kl_inx].rho<<"\n";

    }

    kl[kl_inx].s_rho0=kl[kl_inx].s_rho;
    kl[kl_inx].rho0=kl[kl_inx].rho;

    Point2DF pu=kl[kl_inx].p_m;

   // cam_mod.UndistortHom2Hom(pu,3);
    qx=pu.x;
    qy=pu.y;

    pu=kl[kl_inx].p_m_0;

   // cam_mod.UndistortHom2Hom(pu,3);
    q0x=pu.x;
    q0y=pu.y;


    double v_rho_r=util::norm2(kl[kl_inx].s_rho)+(kl[kl_inx].rho*EKF_SGIRO)*(kl[kl_inx].rho*EKF_SGIRO)+ReshapeFactor;

    double rho_r=kl[kl_inx].rho;

    double a,b;
    a=kl[kl_inx].m_m0.x;
    b=kl[kl_inx].m_m0.y;
    a/=kl[kl_inx].n_m0;
    b/=kl[kl_inx].n_m0;

    double dr=a*(vel[0]*zf-vel[2]*q0x)+b*(vel[1]*zf-vel[2]*q0y);
    double dq=a*(qx-q0x)+b*(qy-q0y);

    Matrix <6,6> R = Zeros;
    R(0,0)=EKF_DQ;
    R(4,4)=EKF_DQ;
    R(5,5)=EKF_DQ;
    R.slice <1,1,3,3> () =vel.as_diagonal()*vel.as_diagonal();
    R.slice <1,1,3,3> ()=R.slice <1,1,3,3> ()*RelError*RelError+RVel;

    double rho_i=1/(1/rho_r + vel[2]);
    double R0,Vn;

    for(int i=0;i<10;i++){
        Matrix <1,6> Mk;
        Mk[0]=makeVector(-1,a*rho_i*zf,b*rho_i*zf,-rho_i*(a*q0x+b*q0y),a*(rho_i*vel[2]),b*(rho_i*vel[2]));
        R0=(Mk*R*Mk.T())(0,0);

        Matrix <2,1> g_xi;
        g_xi(0,0)=1/(1/rho_i-vel[2]);
        g_xi(1,0)=dr*rho_i;

        double p_p=v_rho_r+g_xi(0,0)*g_xi(0,0)*RVel(2,2)*g_xi(0,0)*g_xi(0,0);



        Matrix <2,1> G;
        G(0,0)=1/util::norm2(1-rho_i*vel[2]);
        G(1,0)=dr;

        Matrix <2,2> iC=inv(makeVector(p_p,R0).as_diagonal());

        Matrix <2,1> Z;
        Z(0,0)=rho_r;
        Z(1,0)=dq;

        Vn=1.0/(G.T()*iC*G)(0,0);

        rho_i=(Vn*(G.T()*iC)*(Z - g_xi +G*rho_i))(0,0);
    }

    kl[kl_inx].rho=rho_i;

    kl[kl_inx].s_rho=sqrt(Vn);

    if(kl[kl_inx].rho<RHO_MIN){
        kl[kl_inx].s_rho+=RHO_MIN-kl[kl_inx].rho;
        kl[kl_inx].rho=RHO_MIN;
    }else if(kl[kl_inx].rho>RHO_MAX){
       // kl[kl_inx].s_rho+=kl[kl_inx].rho-RHO_MAX;
        kl[kl_inx].rho=RHO_MAX;
    }else if(isnan(kl[kl_inx].rho) || isnan(kl[kl_inx].s_rho) || isinf(kl[kl_inx].rho) || isinf(kl[kl_inx].s_rho)){
        std::cout<<"\nKL EKF Nan Rho!"<<rho_i<< " " << Vn<< " "<<vel[2]<<"\n";
        kl[kl_inx].rho=Rho0;
        kl[kl_inx].s_rho=RHO_MAX;
    }else if(kl[kl_inx].s_rho<0){
        std::cout<<"\nKL EKF Panic! Neg VRho!"<<vel<<RVel <<kl[kl_inx].rho << kl[kl_inx].s_rho <<"\n";
        kl[kl_inx].rho=Rho0;
        kl[kl_inx].s_rho=RHO_MAX;

    }

    return kl[kl_inx].rho;


}



double edge_tracker::UpdateInverseDepthKalmanUniformMix(Vector <3> vel,Matrix <3,3> RVel,Matrix <3,3> RW0,double ReshapeFactor,int kl_inx){

    double qx,qy,q0x,q0y,v_rho;

    double zf=cam_mod.zfm;

    if(kl[kl_inx].s_rho<0 || kl[kl_inx].rho<0){

        std::cout << "\nUIDK panic! "<<kl[kl_inx].s_rho<<" "<<kl[kl_inx].rho<<"\n";

    }

    kl[kl_inx].s_rho0=kl[kl_inx].s_rho;
    kl[kl_inx].rho0=kl[kl_inx].rho;

    Point2DF pu=kl[kl_inx].p_m;

    //cam_mod.UndistortHom2Hom(pu,3);
    qx=pu.x;
    qy=pu.y;

    pu=kl[kl_inx].p_m_0;

    //cam_mod.UndistortHom2Hom(pu,3);
    q0x=pu.x;
    q0y=pu.y;


    v_rho=kl[kl_inx].s_rho*kl[kl_inx].s_rho;

    double a,b;

    a=kl[kl_inx].m_m0.x;
    b=kl[kl_inx].m_m0.y;
    a/=kl[kl_inx].n_m0;
    b/=kl[kl_inx].n_m0;


    double rho_p=1/(1.0/kl[kl_inx].rho+vel[2]);

    double F=1/(1+kl[kl_inx].rho*vel[2]);
    F=F*F;

    double err_w=RW0(0,0)+RW0(1,0)+RW0(0,1)+RW0(1,1);

    double p_p=F*v_rho*F +(kl[kl_inx].rho*err_w*kl[kl_inx].rho)+rho_p*rho_p*RVel(2,2)*rho_p*rho_p+ReshapeFactor;

    double mean_rh=a*(vel[0]*zf-vel[2]*q0x)+b*(vel[1]*zf-vel[2]*q0y);

    Matrix <1,5> Mk;

    Mk[0]=makeVector(a*zf,b*zf,-(a*q0x+b*q0y),a*(vel[2]),b*(vel[2]));

    Matrix <5,5> R = Zeros;

    R(3,3)=EKF_DQ;
    R(4,4)=EKF_DQ;
    R.slice <0,0,3,3> () =RVel;

    double var_rh=(Mk*R*Mk.T())(0,0);


    double Y;
    double H;
    double R0;
    double e;


    double dq=a*(qx-q0x)+b*(qy-q0y);

    if(mean_rh*mean_rh>var_rh*3){

        if(mean_rh<0){
            mean_rh=-mean_rh;
            dq=-dq;
        }

        double devsq3=sqrt(var_rh*3);

        Y=dq*log((mean_rh+devsq3)/((mean_rh-devsq3)))/(2*devsq3);
        R0=(EKF_DQ*EKF_DQ+dq*dq)/(mean_rh*mean_rh-3*var_rh)-Y*Y;


    }else{

        double dev_rh=sqrt(var_rh);

        norm_dist.EvalReciprocal<false>(mean_rh,dev_rh,1,21);
        Y = dq*mean_rh;
        R0=util::norm2(dq*dev_rh,mean_rh*EKF_DQ,dev_rh*EKF_DQ);

    }

    H= 1;
    e=Y-H*rho_p;

    double S=H*p_p*H+R0;

    double K=p_p*H*(1/S);

    kl[kl_inx].rho=rho_p+(K*e);

    v_rho=(1-(K*H))*p_p;

    kl[kl_inx].s_rho=sqrt(v_rho);

    if(kl[kl_inx].rho<RHO_MIN){
        kl[kl_inx].s_rho+=RHO_MIN-kl[kl_inx].rho;
        kl[kl_inx].rho=RHO_MIN;
    }else if(kl[kl_inx].rho>RHO_MAX){
       // kl[kl_inx].s_rho+=kl[kl_inx].rho-RHO_MAX;
        kl[kl_inx].rho=RHO_MAX;
    }else if(isnan(kl[kl_inx].rho) || isnan(kl[kl_inx].s_rho) || isinf(kl[kl_inx].rho) || isinf(kl[kl_inx].s_rho)){
        std::cout<<"\nKL EKF Nan Rho!"<<rho_p<< " " << p_p<< " "<<vel[2]<< "\n";
        kl[kl_inx].rho=Rho0;
        kl[kl_inx].s_rho=RHO_MAX;
    }else if(kl[kl_inx].s_rho<0){
        std::cout<<"\nKL EKF Panic! Neg VRho!"<<vel<<RVel <<kl[kl_inx].rho << kl[kl_inx].s_rho <<"\n";
        kl[kl_inx].rho=Rho0;
        kl[kl_inx].s_rho=RHO_MAX;

    }

    return kl[kl_inx].rho;


}

double edge_tracker::EstimateScaling(){

    if(kn<=0)
        return 1;

    double rho=0,mass=0;
    for(int ikl=0;ikl<kn;ikl++){
        rho+=kl[ikl].rho/(kl[ikl].s_rho*kl[ikl].s_rho);
        mass+=1.0/(kl[ikl].s_rho*kl[ikl].s_rho);
    }

    return rho/mass;

}


double edge_tracker::EstimateReScaling(double &RKp){

    if(kn<=0)
        return 1;

    double rTr=0,rTr0=0;
    for(int ikl=0;ikl<kn;ikl++){

        if(kl[ikl].m_id<0 || kl[ikl].s_rho0<=0 )
            continue;
        rTr+=kl[ikl].rho*kl[ikl].rho/(kl[ikl].s_rho0*kl[ikl].s_rho0);
        rTr0+=kl[ikl].rho0*kl[ikl].rho0/(kl[ikl].s_rho0*kl[ikl].s_rho0);
        if(isnan(kl[ikl].rho0 || kl[ikl].rho)){
           printf("\n NAN: %f %f %f",kl[ikl].s_rho0,kl[ikl].rho0,kl[ikl].rho);
        }
    }

    RKp=1/rTr;
    if(rTr0>0)
        return sqrt(rTr/rTr0);
    else
        return 1;

}


double edge_tracker::EstimateQuantile(double s_rho_min,double s_RHO_MIN,double percentile,int n){


    int histo[n];


    for(int i=0;i<n;i++){
        histo[i]=0;
    }

    for(int ikl=0;ikl<kn;ikl++){
        int i=n*(kl[ikl].s_rho-s_rho_min)/(s_RHO_MIN-s_rho_min);

        i=i>n-1?n-1:i;
        i=i<0?0:i;

        histo[i]++;
    }

    double s_rho=1e3;

    for(int i=0,a=0;i<n;i++){

        if(a>percentile*kn){

            s_rho=(double)i*(s_RHO_MIN-s_rho_min)/(double)n+s_rho_min;
            break;

        }

        a+=histo[i];
    }

    return s_rho;

}

void edge_tracker::ExtVel(Vector <3> &vel,Matrix <3,3> &RVel){


    int nmatch1D=nmatch;


    Matrix<> Phi=Zeros(nmatch1D,3);
    Matrix<> PhiM=Zeros(nmatch1D,3);
    Matrix <1,1> M;
    Matrix <1,1> Mi;
    Vector<> Y=Zeros(nmatch1D);
    int nok=0;


    int j=0;

    for(int i=0;i<kn;i++){

        if(kl[i].m_id<0)
            continue;

        double a,b,n_ab;

        Point2DF p_m=kl[i].p_m,p_m_0=kl[i].p_m_0;

        //        cam_mod.UndistortHom2Hom(p_m,3);
        //        cam_mod.UndistortHom2Hom(p_m_0,3);


        a=kl[i].m_m0.x;
        b=kl[i].m_m0.y;
        n_ab=util::norm(a,b);
        a/=n_ab;
        b/=n_ab;


        Phi(j+0,0) = a*kl[i].rho*cam_mod.zfm;
        Phi(j+0,1) = b*kl[i].rho*cam_mod.zfm;
        Phi(j+0,2) =a*(-kl[i].rho*p_m.x)+b*(-kl[i].rho*p_m.y);

        Y[j+0]=a*(p_m.x-p_m_0.x)+b*(p_m.y-p_m_0.y);


        //M(0,0)=(EKF_DQ*EKF_DQ)+Y[j+0]*Y[j+0]*kl[i].s_rho*kl[i].s_rho/(kl[i].s_rho*kl[i].s_rho+kl[i].rho*kl[i].rho)/3;


        M(0,0)=(kl[i].s_rho*kl[i].s_rho);

        Mi(0,0)=(1/M(0,0));


        if(isnan(Mi(0,0))){
            std::cout << "\nNan Mi "<<i<<" "<< j<< " "<<kl[i].rho<< " " <<kl[i].s_rho <<" "<<M(0,0)<< "\n";
            //exit(0);
            Mi=Zeros;
            nok--;
        }


        if(isinf(Mi(0,0))){
            //std::cout << "\nInf Mi\n";
            //exit(0);
            Mi=Zeros;
        }

        PhiM.slice(j,0,1,3)=Mi.T()*Phi.slice(j,0,1,3);



        j++;



        nok++;


    }

    if(j!=nmatch1D)
        printf("\nExtVel2D j err!!!\n");

    //std::cout << "\n"<<nmatch<<" "<< j<< "\n";

    if(nok<10){

        std::cout << "\nnok<10!!!\n";

        RVel=Identity*1e3;
        vel=Zeros;
        return;
    }

    RVel=util::Matrix3x3Inv(PhiM.T()*Phi);


 //   std::cout<<"\nPhi:\n"<<PhiM.T()*Phi <<"\n"<<PhiM.T()*Y<<"\n";

    vel=RVel*(PhiM.T()*Y);


}

void edge_tracker::PrintTrails(RGB24Pixel *data,char r, char g,char b){

    for (int ikl=0;ikl<kn;ikl++){


        int np=kl[ikl].plist.Size();

        for(int ip=0;ip<np-1;ip++){

            KLPos *pos=&kl[ikl].plist.GetRelative(ip), *pos0=&kl[ikl].plist.GetRelative(ip+1);

            Point2DF p1=cam_mod.Hom2Img(pos->p);
            Point2DF p0=cam_mod.Hom2Img(pos0->p);

            double dx=p1.x-p0.x;
            double dy=p1.y-p0.y;
            double norm=util::norm(dx,dy);
            dx/=norm;dy/=norm;

            for(int t=0;t<=norm;t++){
                double x=(double)t*dx+p0.x;
                double y=(double)t*dy+p0.y;

                if(x<0||x>=cam_mod.sz.w|| y<0||y>=cam_mod.sz.h)
                    continue;

                data[(int)y*cam_mod.sz.w+(int)x].pix.b=b;
                data[(int)y*cam_mod.sz.w+(int)x].pix.g=g;
                data[(int)y*cam_mod.sz.w+(int)x].pix.r=r;
            }

        }
    }

}

void edge_tracker::saveTrays(const char *fname){

    FILE *f=fopen(fname,"w");

    printf("\nEdge Tracker: Guardando logs %s\n",fname);

    fprintf(f,"\nTraysX=[");

    for(int ikl=0;ikl<kn;ikl++){

        int np=kl[ikl].plist.Size(),ip;
        double v=0;
        for(ip=0;ip<np;ip++){
            v=kl[ikl].plist.GetRelative(ip).p.x;
            if(ip>0)
                fprintf(f,",%e",v);
            else
                fprintf(f,"%e",v);
        }
        for(;ip<kl[ikl].plist.ListSize();ip++){
            fprintf(f,",%e",v);
        }

        fprintf(f,";...\n");

    }

    fprintf(f,"];\n");

    fprintf(f,"\nTraysY=[");

    for(int ikl=0;ikl<kn;ikl++){

        int np=kl[ikl].plist.Size(),ip;
        double v=0;
        for(ip=0;ip<np;ip++){
            v=kl[ikl].plist.GetRelative(ip).p.y;
            if(ip>0)
                fprintf(f,",%e",v);
            else
                fprintf(f,"%e",v);
        }
        for(;ip<kl[ikl].plist.ListSize();ip++){
            fprintf(f,",%e",v);
        }

        fprintf(f,";...\n");

    }

    fprintf(f,"];\n");

    fprintf(f,"\nTraysInfo=[");

    for(int ikl=0;ikl<kn;ikl++){

        int np=kl[ikl].plist.Size();

        fprintf(f,"%d,%lu,%e,%e",np,kl[ikl].tag,kl[ikl].rho,kl[ikl].s_rho);

        fprintf(f,";...\n");

    }

    fprintf(f,"];\n");


    fprintf(f,"\nVelTray=[");

    for(int ip=0;ip<vlist.Size();ip++){


        fprintf(f,"%e,%e,%e;...\n",vlist.GetRelative(ip)[0],vlist.GetRelative(ip)[1],vlist.GetRelative(ip)[2]);


    }

    fprintf(f,"];\n");



    fclose(f);

}




