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

#ifndef CAM_MODEL_H
#define CAM_MODEL_H

#include "video_io.h"
#include "common_types.h"
#include <math.h>

struct cam_model{

public:
    Point2DF pp;
    Point2DF zf;
    double zfm;
    double Kc[2];
    Size2D sz;

    cam_model(Point2DF prin_point,Point2DF focal_dist,double DistKc[2],Size2D ImageSize)
        :pp(prin_point),zf(focal_dist),zfm((focal_dist.x+focal_dist.y)/2),Kc{DistKc[0],DistKc[1]},sz(ImageSize){ }

    cam_model(){}

    void UndistortHom2Hom(Point2DF &p,int newton_it){

        double rn,rd;

        p.x+=sz.w/2-pp.x;
        p.y+=sz.h/2-pp.y;

        rd=sqrt((p.x/zf.x)*(p.x/zf.x)+(p.y/zf.y)*(p.y/zf.y));

        rn=rd;
        for(int i=0;i<newton_it;i++){

            rn = rn - (rn*(1+rn*rn*(Kc[0]+Kc[1]*rn*rn))-rd)/(1+rn*rn*(3*Kc[0]+5*Kc[1]*rn*rn));

        }


        p.x*=rn/rd*zfm/zf.x;
        p.y*=rn/rd*zfm/zf.y;


    }

    template <typename PointType>
    inline PointType Hom2Img(const PointType &ph){
        return {ph.x+pp.x,ph.y+pp.y};
    }
    inline Point2DF Img2Hom(const Point2DF &pi){
        return {pi.x-pp.x,pi.y-pp.y};
    }

    template <class T>
    void Hom2Img(T &ix,T &iy,const T hx, const T hy){
        ix=hx+pp.x;
        iy=hy+pp.y;
    }

    template <class T>
    void Img2Hom(T &hx,T &hy,const T ix, const T iy){
        hx=ix-pp.x;
        hy=iy-pp.y;
    }

};

#endif // CAM_MODEL_H
