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


#include "crash_detector.h"

using namespace TooN;

crash_detector::crash_detector(depth_filler *df,int df_num, Point2DF pp, float zf)
{

    df_n=df_num;
    this->df=df;
    this->pp=pp;
    this->zf=zf;

    min_dist=1e20;
    time2impact=1e20;
    dist2impact=1e20;
    int_num=0;
}


double crash_detector::EstimateDistAndRange_BP(const std::vector <Point4DD> &nav_tray, double &time2impact,int npoints,double bubble_size){


    V=FilterVel(nav_tray,npoints);

    double normV=norm(V);

    int_num=0;

    time2impact=1e20;

    if(normV<1e-6 || isnan(normV)){

        return 1e20;
    }

    Vector <3> Vnorm=V/normV;

    double dist=1e20;

    double min_dist=1e20;
    int min_x=-1,min_y=-1;


    for(int y=0;y<df->s.h;y++){
        for(int x=0;x<df->s.w;x++){

            int inx=y*df->s.w+x;


            double z0=df->data[inx].depth;

            if(isnan(z0) || isinf(z0))
                continue;

            double ix0=((x)*df->scl_w-(double)pp.x)/zf;
            double iy0=((y)*df->scl_h-(double)pp.y)/zf;

            Vector <3> p0=makeVector(ix0*z0, iy0*z0, z0);

            df->data[inx].intersected=false;

            if(BubblePointInt(p0,Zeros,Vnorm,bubble_size,dist)){



                if(util::keep_min(min_dist,dist)){
                    min_x=x;
                    min_y=y;
                }
                int_num++;
            }


        }
    }

    if(int_num>0){

        time2impact=min_dist/normV;

        if(min_x>0 && min_y>0)
            df->data[(min_y-1)*df->s.w+min_x-1].intersected=true;

        if(min_x>0)
            df->data[(min_y)*df->s.w+min_x-1].intersected=true;

        if(min_y>0)
            df->data[(min_y-1)*df->s.w+min_x].intersected=true;

        df->data[(min_y)*df->s.w+min_x].intersected=true;

    }else{
        min_dist=1e20;
        time2impact=1e20;
    }

    this->time2impact=time2impact;

    return min_dist;

}

double crash_detector::EstimateDistAndRange_MT(const std::vector <Point4DD> &nav_tray, double &time2impact,int npoints){


    V=FilterVel(nav_tray,npoints);

    double normV=norm(V);

    int_num=0;

    time2impact=1e20;

    if(normV<1e-6){

        return 1e20;
    }
   // printf("\n%e\n",normV);

    Vector <3> Vnorm=V/normV;

    double dist=1e20;

    for(int y=0;y<df->s.h-1;y++){
        for(int x=0;x<df->s.w-1;x++){

            int inx=y*df->s.w+x;


            double z0=df->data[inx].depth;
            double z1=df->data[inx+1].depth;
            double z2=df->data[inx+df->s.w+1].depth;
            double z3=df->data[inx+df->s.w].depth;

            double ix0=((x)*df->scl_w-(double)pp.x)/zf;
            double iy0=((y)*df->scl_h-(double)pp.y)/zf;
            double ix1=((x+1)*df->scl_w-(double)pp.x)/zf;
            double iy1=((y+1)*df->scl_h-(double)pp.y)/zf;

            Vector <3> p0=makeVector(ix0*z0, iy0*z0, z0);
            Vector <3> p1=makeVector(ix1*z1, iy0*z1, z1);
            Vector <3> p2=makeVector(ix1*z2, iy1*z2, z2);
            Vector <3> p3=makeVector(ix0*z3, iy1*z3, z3);

            df->data[inx].intersected=false;

            if(TriangleRayInt_MollerTrum(p0,p1,p3,Zeros,Vnorm,dist)){
                time2impact=dist/normV;
                df->data[inx].intersected=true;
                int_num++;
            }


            if(TriangleRayInt_MollerTrum(p2,p3,p1,Zeros,Vnorm,dist)){
                time2impact=dist/normV;
                df->data[inx].intersected=true;
                int_num++;
            }

        }
    }

    if(int_num>1){
        printf("\nChash Det: Warning, mas de un triangulo intersectado %d\n",int_num);
    }else if(int_num==0){
        dist=1e20;
        time2impact=1e20;
    }

    this->time2impact=time2impact;

    return dist;

}

bool crash_detector::TriangleRayInt_MollerTrum(const Vector <3> &p0,const Vector <3> &p1,const Vector <3> &p2,const Vector <3> &o,const Vector <3> &d,double &t){


        Vector <3> e0=p1-p0;
        Vector <3> e1=p2-p0;
        Vector <3> T=o-p0;

        Vector <3> P=d^e1;

        double det=e0*P;

        if(fabs(det)<1e-10)
            return false;

        double i_det=1/det;

        double u=(T*P)*i_det;

        if(u<0 || u>1)
            return false;

        Vector <3> Q=T^e0;

        double v=(d*Q)*i_det;

        if(v<0 || (u+v)>1)
            return false;

        t=(e1*Q)*i_det;

        if(t<=0)
            return false;

        return true;

}



bool crash_detector::BubblePointInt(const Vector <3> &p0,const Vector <3> &o,const Vector <3> &d,const double &s,double &t){

    Vector <3> T=p0-o;

    double u=(T*d);

    if(u<0)
        return false;

    Vector <3> D=d*u;

    Vector <3> Q=T-D;

    if(norm(Q)>s){
        return false;
    }

    t=norm(D);
    return true;

}

TooN::Vector<3> crash_detector::FilterVel(const std::vector <Point4DD> &nav_tray, int npoints){

    Vector<3> Dp=Zeros;

    int n;

    if((n=nav_tray.size())<npoints)
        return V;

    double Dt=0;

    for(int i=0;i<npoints;i++){

        Dp+=makeVector(nav_tray[n-i-1].x,nav_tray[n-i-1].y,nav_tray[n-i-1].z);
        Dt+=nav_tray[n-i-1].t;

    }

    return Dp/Dt;

}

double crash_detector::EstimateRefMinDist(){


    min_dist=1e20;

    for(int i=0;i<df_n;i++)
        util::keep_min(min_dist,df[i].GetMinDist());

    return min_dist;


}
