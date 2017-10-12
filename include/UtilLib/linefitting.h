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

#ifndef LINEFITTING_H
#define LINEFITTING_H

#include "UtilLib/cam_model.h"
namespace rebvo{

template <class T>
struct PPoint3D{
    T x;
    T y;
    T rho;
    T s_rho;
};


template <class T>
struct Line2Dequ{       // m1*x+m2*y+m3=0;
    T m1;
    T m2;
    T m3;
};

class LineFitting
{
public:
    LineFitting();


    template  <class T1,class T2>
    void static Fit2DLine(PPoint3D<T1> *plist,const int size,Line2Dequ<T2> &fp);

    template <class T>
    double static DummyFit3DLine(PPoint3D<T> *plist,const int size, cam_model &cam,PPoint3D<T> &p0,PPoint3D<T> &p1);
    template <class T>
    double static Fit3DLine(PPoint3D<T> *plist,const int size, cam_model &cam,PPoint3D<T> &p0,PPoint3D<T> &p1);
    template <class T>
    double static RobustFit3DLine(PPoint3D<T> *plist,const int size, cam_model &cam,PPoint3D<T> &p0,PPoint3D<T> &p1,T s_thr);
};
}
#endif // LINEFITTING_H
