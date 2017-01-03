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

#include "scaleestimator.h"
#include "minimizer.h"
#include <TooN/Cholesky.h>
#include <TooN/so3.h>
#include "util.h"
using namespace TooN;
namespace  rebvo{
ScaleEstimator::ScaleEstimator()
{
}



void ScaleEstimator::EstAcelLsq4(const TooN::Vector <3> &vel,TooN::Vector <3>  &acel, \
                               const TooN::Matrix<3,3> &R,\
                               const double &dt){

    static Vector <3> V=Zeros,V0=Zeros,V1=Zeros,V2=Zeros,V3=Zeros;
    static double T[5]={0};
    static double Dt[4]={0};

   // static double kG[4]={1};



    V3=R.T()*V2;
    V2=R.T()*V1;
    V1=R.T()*V0;
    V0=R.T()*V;
    V=vel;

    for(int i=0;i<3;i++){
        Dt[i]=Dt[i+1];
    }
    Dt[3]=dt;

    T[0]=0;
    double mt=0;
    for(int i=0;i<4;i++){
        T[i+1]=T[i]+Dt[i];
        mt+=T[i+1];
    }
    mt/=5;

    double num=0,den=0,vm;

    for(int i=0;i<5;i++)
        den+=(T[i]-mt)*(T[i]-mt);

    for(int i=0;i<3;i++){

        vm=(V[i]+V0[i]+V1[i]+V2[i]+V[3])/5.0;

        num =(V[i]-vm)*(T[4]-mt);
        num+=(V0[i]-vm)*(T[3]-mt);
        num+=(V1[i]-vm)*(T[2]-mt);
        num+=(V2[i]-vm)*(T[1]-mt);
        num+=(V3[i]-vm)*(T[0]-mt);

        if(den>0)
            acel[i]=num/den;

    }

}

void ScaleEstimator::MeanAcel4(const TooN::Vector <3> &s_acel,TooN::Vector <3>& acel, \
                               const TooN::Matrix<3,3> &R){

    static Vector <3> A=Zeros,A0=Zeros,A1=Zeros,A2=Zeros;



    A2=R.T()*A1;
    A1=R.T()*A0;
    A0=R.T()*A;
    A=makeVector(s_acel[0],s_acel[1],s_acel[2]);

    acel=(A+A0+A1+A2)/4;


}





struct FunParams_KaGMEKBias{
    Vector <3> a_v;
    Vector <3> a_s;
    double G;
    Vector <7> x_p;
    Matrix <3,3> Rv;
    Matrix <3,3> Rs;
    double Rg;
    Matrix <7,7> Pp;
};

bool Problem_KaGMEKBias(TooN::Matrix<7,7>&JtJ,TooN::Vector<7>&JtF,const Vector<7> &x,FunParams_KaGMEKBias * p){

    double a=x[0];
    Vector <3> g=x.slice<1,3>();
    Vector <3> b=x.slice<4,3>();
    Vector <3> &a_s=p->a_s;
    Vector <3> &a_v=p->a_v;

    Vector <11> F=Zeros;
    F.slice<0,3>()=(a_s+g)*cos(a)-a_v*sin(a);
    F[3]=g*g-p->G*p->G;

    F[4]=x[0]-p->x_p[0];                        //Error function Angle

    if(F[4]>M_PI)                               //Error correction on the angle (cyclicity)
        F[4]-=2*M_PI;
    else if(F[4]<-M_PI)
        F[4]+=2*M_PI;


    SO3<> Rb(b);

    F.slice<5,3>()=Rb*g-p->x_p.slice<1,3>();       //Error function G
    F.slice<8,3>()=b-p->x_p.slice<4,3>();           //Error function Bias

    Vector <11> dFda=Zeros;                          //
    dFda.slice<0,3>()=-(a_s+g)*sin(a)-a_v*cos(a);
    dFda[4]=1;


    Vector <3> Rg=Rb*g;
    Matrix <3,3> Gx=Data(  0  , Rg[2], -Rg[1], \
                         -Rg[2],  0  ,  Rg[0], \
                          Rg[1],-Rg[0],   0   );


    Matrix <11,6> dFdx1=Zeros;
    dFdx1.slice<0,0,3,3>()=Identity*cos(a);         //dF(0:2)/dG    measurement
    dFdx1.slice<3,0,1,3>()=2*g.as_row();            //dF(3)/dG      G module
    dFdx1.slice<5,0,3,3>()=Rb.get_matrix();         //dF(5:7)/dG    G prior with bias rotation
    dFdx1.slice<5,3,3,3>()=Gx;                      //dF(5:7)/db    derivative of G prior with rotation
    dFdx1.slice<8,3,3,3>()=Identity;                //dF(8:10)/db   b prior

    Matrix <3,3> Pz=sin(a)*sin(a)*p->Rv+cos(a)*cos(a)*p->Rs;

    Matrix <11,11> P=Zeros;
    P.slice<0,0,3,3>()=Pz;
    P(3,3)=p->Rg;
    P.slice<4,4,7,7>()=p->Pp;

    Matrix <11,11> W=Zeros;
    W.slice<0,0,3,3>()=Cholesky<3>(Pz).get_inverse();
    W(3,3)=1/p->Rg;
    W.slice<4,4,7,7>()=Cholesky<7>(p->Pp).get_inverse();

    Matrix <11,11> dPda=Zeros;
    dPda.slice<0,0,3,3>()=2*sin(a)*cos(a)*(p->Rv-p->Rs);

    Matrix <11,11> dWda=-W*dPda*W;

    JtJ(0,0)=0.25*F*dWda*P*dWda*F+dFda*dWda*F+dFda*W*dFda;
    JtJ.slice<1,0,6,1>()=(0.5*dFdx1.T()*dWda*F+dFdx1.T()*W*dFda).as_col();
    JtJ.slice<0,1,1,6>()=JtJ.slice<1,0,6,1>().T();
    JtJ.slice<1,1,6,6>()=dFdx1.T()*W*dFdx1;

    JtF[0]=0.5*F*dWda*F+dFda*W*F;
    JtF.slice<1,6>()=dFdx1.T()*W*F;

    return true;
}

Vector<7> FunT_KaGMEKBias(const Vector<7> &x,FunParams_KaGMEKBias * p){

    return makeVector(atan2(sin(x[0]),cos(x[0])),x[1],x[2],x[3],util::saturate(x[4],5e-1/25),util::saturate(x[5],5e-1/25),util::saturate(x[6],5e-1/25));
}



double ScaleEstimator::estKaGMEKBias(   const TooN::Vector<3> &s_acel,
                                        const TooN::Vector<3> &f_acel,
                                        double kP,
                                        TooN::Matrix<3,3> Rot,
                                        TooN::Vector <7> &X,
                                        TooN::Matrix <7,7> &P,
                                        const TooN::Matrix <3,3> &Qg,
                                        const TooN::Matrix <3,3> &Qrot,
                                        const TooN::Matrix <3,3> &Qbias,
                                        const double &QKp,
                                        const double &Rg,
                                        const Matrix <3,3> &Rs,
                                        const TooN::Matrix <3,3> &Rf,
                                        TooN::Vector<3> & g_est,
                                        TooN::Vector<3> &b_est,
                                        const TooN::Matrix <6,6> &Wvw,
                                        TooN::Vector <6> &Xvw,
                                        double g_gravit
                                        )
{


  //  static Vector <7> X=makeVector(M_PI/4,0,g_gravit,0,0,0,0);
  //  static Matrix <7,7> P=makeVector(M_PI/16*M_PI/16*1e-2,100,100,100,1e-10,1e-10,1e-10).as_diagonal();


    /*Prior calculation (linear)*/
    Matrix <7,7> F=Zeros;
    F(0,0)=kP;
    F.slice<1,1,3,3>()=Rot.T();
    F.slice<4,4,3,3>()=Identity;


    Vector <3> Gtmp=X.slice<1,3>();
    Matrix <3,3> GProd=Data(  0  , Gtmp[2], -Gtmp[1], \
                         -Gtmp[2],  0  ,  Gtmp[0], \
                          Gtmp[1],-Gtmp[0],   0   );


    Matrix <7,7> Q=Zeros;
    Q(0,0)=QKp;
    Q.slice<1,1,3,3>()=GProd.T()*Qrot*GProd+Qg;
    Q.slice<4,4,3,3>()=Qbias;


    X=F*X;
    Matrix <7,7> Pp=F*P*F.T()+Q;

    /*Add visual bias meassurement*/



    /*Posterior estimation (non linear)*/

    FunParams_KaGMEKBias params;

    params.a_s=s_acel;
    params.a_v=f_acel;


    params.Rs=Rs;
    params.Rv=Rf;
    params.Pp=Pp;
    params.Rg=Rg;
    params.G=g_gravit;
    params.x_p=X;

    //std::cout << "\n"<< Rs <<"\n" << Rf<<"\n";

    Minimizer<7,11,FunParams_KaGMEKBias>::GaussNewton(X,Problem_KaGMEKBias,&params,20,FunT_KaGMEKBias);

    Matrix<7,7>JtJ;
    Vector<7>JtF;
    Problem_KaGMEKBias(JtJ,JtF,X,&params);

    P=Cholesky<7>(JtJ).get_inverse();

    double k=tan(X[0]);

    if(k<0 || std::isnan(k) || std::isinf(k))
        k=0;

    g_est=X.slice<1,3>();
    b_est=X.slice<4,3>();



    /*correct visual meassurement with bias estimation*/

    Matrix <3,3> WVBias=JtJ.slice<4,4,3,3>();

    Matrix <6,6> Wb=Zeros;
    Wb.slice<3,3,3,3>()=WVBias;

    Vector <3> wc=Xvw.slice<3,3>() - b_est;

    Vector <6> WXc=Zeros;
    WXc.slice<3,3>()=WVBias*wc;
    Vector <6> Xc=Cholesky<6>(Wb+Wvw).get_inverse()*( Wvw*Xvw +  WXc);

    Xvw=Xc;

    return k;
}


}






