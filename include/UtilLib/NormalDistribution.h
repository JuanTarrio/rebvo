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

#ifndef NORMALDISTRIBUTION_H
#define NORMALDISTRIBUTION_H

#include "util.h"
namespace  rebvo{
namespace util{



template<class T,int size>
class NormalDistribution{

    T table[size];
    T spar;

public:

    NormalDistribution(T sparceness){

        T x;

        spar=sparceness;

        double k=1/(sqrt(2*M_PI));

        for(int i=0;i<size;i++){

            x=(T)i*sparceness/(T)size;

            table[i]=k*exp(-x*x/2);

        }

    }

    T eval(T x,T mu,T sigma){

        T z=fabs(x-mu)/sigma;

        int i=z*size/spar;
        if(i>=size)
            return 0;

        return table[i]/sigma;

    }


    template <bool debug>
    void EvalReciprocal(T &mean,T &dev,double r=1,int n=10){

            double p[n+1],x[n+1],rx[n+1];

            bool zero=false;

            if(debug){
                printf("\n(x,p,rx): ");
            }


            for(int i=0;i<n;i++){

                x[i]=2*dev*r*(T)(i-n/2)/(T)(n-1)+mean;
                p[i]=eval(x[i],mean,dev);

                if(fabs(x[i])>0){
                    rx[i]=1/x[i];
                }else{
                    rx[i]=num_inf;
                    rx[n]=-num_inf;
                    p[n]=p[i];
                    x[n]=x[i];
                    zero=true;
                }

                if(debug){
                    printf("(%f,%f,%f) ",x[i],p[i],rx[i]);
                }

            }

            if(zero){
                if(debug){
                    printf("(%f,%f,%f) ",x[n],p[n],rx[n]);
                }
                n++;
            }

            double mr=0,vr=0,m=0,v=0,mass=0;

            for(int i=0;i<n;i++){
                mr+=rx[i]*p[i];
                m+=x[i]*p[i];
                mass+=p[i];
            }

            mr/=mass;
            m/=mass;

            if(debug){
                printf("\n Mean = %f, CM = %f, RM = %f",mean,m,mr);
            }


            for(int i=0;i<n;i++){
                vr+=(rx[i]-mr)*(rx[i]-mr)*p[i];
                v+=(x[i]-m)*(x[i]-m)*p[i];
            }

            vr/=mass;
            v/=mass;

            if(debug){
                printf("\n Dev = %f, CM = %f, RM = %f\n",dev,sqrt(v),sqrt(vr));
            }

            dev=sqrt(vr);
            mean=mr;

        }

};


}
}

#endif // NORMALDISTRIBUTION_H
