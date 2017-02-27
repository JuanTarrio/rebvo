#include "UtilLib/linefitting.h"
#include "UtilLib/util.h"

#include <TooN/TooN.h>

namespace rebvo{
LineFitting::LineFitting()
{
}


template <class T1,class T2>
void LineFitting::Fit2DLine(PPoint3D<T1> *plist, const int size, Line2Dequ<T2> &fp){

    T2 m_x=0,m_y=0;
    T1 x[size];
    T1 y[size];
    T2 Sxx=0,Syy=0,Sxy=0,a;

    for(int i=0;i<size;i++){
        m_x+=plist[i].x;
        m_y+=plist[i].y;
    }
    m_x/=size;
    m_y/=size;

    for(int i=0;i<size;i++){
        x[i]=plist[i].x-m_x;
        y[i]=plist[i].y-m_y;

        Sxx+=x[i]*x[i];
        Sxy+=x[i]*y[i];
        Syy+=y[i]*y[i];
    }

    a=0.5*atan2(-2*Sxy,Sxx-Syy);

    fp.m1=sin(a);
    fp.m2=cos(a);
    fp.m3=-(fp.m1*m_x+fp.m2*m_y);


}

template <class T>
double LineFitting::DummyFit3DLine(PPoint3D<T> *plist, const int size, cam_model &cam, PPoint3D<T> &p0, PPoint3D<T> &p1){

    p0=plist[0];
    p1=plist[size-1];
    return 0;

}

template double LineFitting::DummyFit3DLine<float>(PPoint3D<float> *plist,const int size,  cam_model &cam,PPoint3D<float> &p0,PPoint3D<float> &p1);

template <class T>
double LineFitting::Fit3DLine(PPoint3D<T> *plist, const int size, cam_model &cam, PPoint3D<T> &p0, PPoint3D<T> &p1){


    using namespace TooN;
    Line2Dequ <double> equ2d;

    for(int i=0;i<size;i++)
        cam.Img2Hom(plist[i].x,plist[i].y,plist[i].x,plist[i].y);

    Fit2DLine(plist,size,equ2d);

    //T ox=-equ2d.m3*equ2d.m1;
    //T oy=-equ2d.m3*equ2d.m2;
    T tx= equ2d.m2;
    T ty=-equ2d.m1;

    T zfo=sqrt(cam.zfm*cam.zfm+equ2d.m3*equ2d.m3);

    T ql[size];

    Matrix <Dynamic,2> Phi(size,2);
    Vector <Dynamic> Y(size);

    for(int i=0;i<size;i++){
        ql[i]=plist[i].x*tx+plist[i].y*ty;

        Phi(i,0)=ql[i]/cam.zfm/plist[i].s_rho;
        Phi(i,1)=zfo/cam.zfm/plist[i].s_rho;
        Y[i]=plist[i].rho/plist[i].s_rho;

    }

    Matrix <2,2> P=TooN::inv(Phi.T()*Phi);
    Vector <2> R=P * (Phi.T()*Y);

    Vector <2> Q=makeVector(ql[0]/cam.zfm,zfo/cam.zfm);
    p0.rho=Q*R;
    p0.s_rho=plist[0].s_rho;//sqrt((Q.as_row()*P*Q.as_col())(0,0));

    Q[0]=ql[size-1]/cam.zfm;
    p1.rho=Q*R;
    p1.s_rho=plist[size-1].s_rho;//sqrt((Q.as_row()*P*Q.as_col())(0,0));

    cam.Hom2Img(p0.x,p0.y,plist[0].x,plist[0].y);
    cam.Hom2Img(p1.x,p1.y,plist[size-1].x,plist[size-1].y);

    return sqrt((Q.as_row()*P*Q.as_col())(0,0));

}


template double LineFitting::Fit3DLine<float>(PPoint3D<float> *plist,const int size,  cam_model &cam,PPoint3D<float> &p0,PPoint3D<float> &p1);


template <class T>
double LineFitting::RobustFit3DLine(PPoint3D<T> *plist, const int size, cam_model &cam, PPoint3D<T> &p0, PPoint3D<T> &p1, T s_thr){


    using namespace TooN;
    Line2Dequ <double> equ2d;

    for(int i=0;i<size;i++)
        cam.Img2Hom(plist[i].x,plist[i].y,plist[i].x,plist[i].y);

    Fit2DLine(plist,size,equ2d);

    //T ox=-equ2d.m3*equ2d.m1;
    //T oy=-equ2d.m3*equ2d.m2;
    T tx= equ2d.m2;
    T ty=-equ2d.m1;

    T zfo=sqrt(cam.zfm*cam.zfm+equ2d.m3*equ2d.m3);

    T ql[size];

    Matrix <Dynamic,2> Phi(size,2);
    Vector <Dynamic> Y(size);
    Matrix <2,2> P;
    Matrix <2,2> Pi;
    Vector <2> R;

    Vector <Dynamic> r(size);
    Vector <Dynamic> w(size);

    T mean_s=0;

    for(int i=0;i<size;i++){
        ql[i]=plist[i].x*tx+plist[i].y*ty;
        mean_s+=plist[i].s_rho;
         w[i]=1;
    }
    mean_s/=size;

    for(int it=0;it<1;it++){

        for(int i=0;i<size;i++){

            Phi(i,0)=ql[i]/cam.zfm/plist[i].s_rho;
            Phi(i,1)=zfo/cam.zfm/plist[i].s_rho;
            Y[i]=plist[i].rho/plist[i].s_rho;

        }

        Pi=Phi.T()*w.as_diagonal()*Phi;
        if(TooN::determinant(Pi)<1e-10){

            p0=plist[0];
            p1=plist[size-1];
            return 1e10;
        }
        P=TooN::inv(Pi);
        R=P * (Phi.T()*w.as_diagonal()*Y);


        for(int i=0;i<size;i++){
            r[i]=fabs(plist[i].rho-(ql[i]/cam.zfm*R[0]+zfo/cam.zfm*R[1]));
            if(r[i]<plist[i].s_rho)
                w[i]=1;
            else
                w[i]=plist[i].s_rho/(r[i]+1e-10);
        }



    }

    Vector <2> Q=makeVector(ql[0]/cam.zfm,zfo/cam.zfm);
    p0.rho=Q*R;
    p0.s_rho=plist[0].s_rho;//sqrt((Q.as_row()*P*Q.as_col())(0,0));

    Q[0]=ql[size-1]/cam.zfm;
    p1.rho=Q*R;
    p1.s_rho=plist[size-1].s_rho;//sqrt((Q.as_row()*P*Q.as_col())(0,0));

    cam.Hom2Img(p0.x,p0.y,plist[0].x,plist[0].y);
    cam.Hom2Img(p1.x,p1.y,plist[size-1].x,plist[size-1].y);

    for(int i=0;i<size;i++)
        if(r[i]>plist[i].s_rho)
        return 1e10;

    for(int i=0;i<size-1;i++)
        if(fabs(plist[i].rho-plist[i+1].rho)>std::min(plist[i].s_rho,plist[i+1].s_rho))
            return 1e10;

    return mean_s;//sqrt((Q.as_row()*P*Q.as_col())(0,0));

}


template double LineFitting::RobustFit3DLine<float>(PPoint3D<float> *plist,const int size,  cam_model &cam,PPoint3D<float> &p0,PPoint3D<float> &p1, float s_thr);
}
