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

#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

//****** Some common types used all across the library ******

#include "video_io.h"

namespace  rebvo{
template <typename T>
struct Point2D{
    T x;
    T y;
    Point2D():x(0),y(0) {}
    template <typename T1> Point2D(const T1 & xi,const T1 & yi):x(xi),y(yi) {}


};

template <typename T>
struct Point3D{
    T x;
    T y;
    T z;
    Point3D():x(0),y(0),z(0){}
    template <typename T1> Point3D(const T1 &xi,const T1 & yi,const T1 & zi):x(xi),y(yi),z(zi) {}

};

template <typename T>
struct Point4D{
    T x;
    T y;
    T z;
    T t;
    Point4D():x(0),y(0),z(0),t(0){}
    template <typename T1> Point4D(const T1 & xi,const T1 & yi,const T1 & zi,const T1 & ti):x(xi),y(yi),z(zi),t(ti) {}
};


typedef Point4D<double> Point4DD;
}
#endif // COMMON_TYPES_H
