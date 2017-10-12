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

#ifndef SURFACE_INTEGRATOR_H
#define SURFACE_INTEGRATOR_H

#include "mtracklib/keyframe.h"

namespace rebvo {

namespace SurfaceInt{
TooN::Vector<3>  analizeSpaceSize(std::vector <keyframe> &kf_list, TooN::Vector<3> *origin=nullptr);

void checkDFRayCrossExaustive(keyframe &target,keyframe &hidder);

}

struct OGridPoint{
    std::vector <df_point*> surf_p;
};

class OcGrid{

    Point3D<u_int> size;
    TooN::Vector<3> wordl_size;
    u_int lin_size;
    TooN::Vector<3> origin;
    TooN::Vector<3> block_size;
    double min_block_size;
    OGridPoint** data;

public:

    OcGrid(TooN::Vector<3> orig_point,TooN::Vector<3> space_size,Point3D<u_int> grid_size);
    ~OcGrid();

    bool addDFPoint(OGridPoint* &pos, df_point *point);

    u_int fillKFList(std::vector <keyframe> &kf_list);
    u_int rayCutSurface(std::vector <keyframe> &kf_list);

    inline u_int linearIndex(const u_int &x,const u_int & y,const u_int & z){
        return z*(size.x*size.y)+y*size.x+x;
    }

    inline u_int wordl2Index(const TooN::Vector<3> &p){
        TooN::Vector<3> pw=p-origin;
        return linearIndex(pw[0]/block_size[0],pw[1]/block_size[1],pw[2]/block_size[2]);
    }

    inline TooN::Vector<3>  index2wordl(const u_int &x,const u_int & y,const u_int & z){
        return TooN::makeVector((double)x*block_size[0],(double)y*block_size[1],(double)z*block_size[2])+origin;
    }

    inline OGridPoint* operator () (const u_int &x,const u_int & y,const u_int & z){
        return data[linearIndex(x,y,z)];
    }
    inline OGridPoint* operator () (const TooN::Vector<3> &p){
        return data[wordl2Index(p)];
    }


    void hideAll(OGridPoint *&pos, df_point *point);
    u_int rayCutSurface(keyframe &kf);
};


}

#endif // SURFACE_INTEGRATOR_H
