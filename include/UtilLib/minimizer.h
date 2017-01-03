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

#ifndef MINIMIZER_H
#define MINIMIZER_H

#include <TooN/TooN.h>
#include <TooN/SVD.h>
#include <TooN/Cholesky.h>
namespace  rebvo{
template <int nvars,int neqs,typename TParam>
class Minimizer
{
public:


    typedef TooN::Vector<neqs> (*MinimizerFfun) (const TooN::Vector<nvars>&,TParam *);
    typedef TooN::Matrix<neqs,nvars> (*MinimizerJfun) (const TooN::Vector<nvars>&,TParam *);
    typedef TooN::Vector<nvars> (*MinimizerTfun) (const TooN::Vector<nvars>&,TParam *);


    typedef bool (*MinimizerProblem) (TooN::Matrix<nvars,nvars>&,TooN::Vector<nvars>&,const TooN::Vector<nvars>&,TParam *);


    Minimizer(){}

    static int GaussNewton(TooN::Vector<nvars> &x,MinimizerFfun ffun,MinimizerJfun Jfun,TParam *params,int iter_max,TooN::Matrix<neqs,neqs>W=TooN::Identity,MinimizerTfun Tfun=NULL,double a_tol=0,double r_tol=0){

        using namespace TooN;
        int i=0;

        Vector<nvars> h;
        Vector<neqs> f;
        Matrix<neqs,nvars> J;

       // double E0;

        for(;i<iter_max;i++){

            f=ffun(x,params);
            J=Jfun(x,params);

            //if(i==0)
             //   E0=f*W*f;

            Cholesky <nvars> sol(J.T()*W*J);
            h=sol.backsub(-J.T()*W*f);
            x+=h;

            if(Tfun)
                x=Tfun(x,params);

            if(norm(h)<a_tol || norm(h)/(norm(x)+1e-20)<r_tol)
                break;
        }

        //std::cout <<"\n"<<h<<" E0="<<E0<<" E="<<f*W*f<<"\n";
        return i;

    }


    static int GaussNewton(TooN::Vector<nvars> &x,MinimizerProblem problem,TParam *params,int iter_max,MinimizerTfun Tfun=NULL,double a_tol=0,double r_tol=0){

        using namespace TooN;
        int i=0;

        Vector<nvars> h;
        Matrix<nvars,nvars>JtJ;
        Vector<nvars>JtF;

       // TooN::Vector<nvars> x0=x;

        for(;i<iter_max;i++){

            problem(JtJ,JtF,x,params);

            SVD <nvars> sol(JtJ);
            h=sol.backsub(-JtF);
            x+=h;

            if(Tfun)
                x=Tfun(x,params);

            if(norm(h)<a_tol || norm(h)/(norm(x)+1e-20)<r_tol)
                break;
        }

        //std::cout <<"\n"<<h<<" "<<x-x0<<"\n";
        return i;

    }
};


}

#endif // MINIMIZER_H
