#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

//****** Some common types used all across the library ******

#include "video_io.h"

template <typename T>
struct Point2D{
    T x;
    T y;
    Point2D():x(0),y(0) {}
    template <typename T1> Point2D(T1 xi,T1 yi):x(xi),y(yi) {}


};

template <typename T>
struct Point3D{
    T x;
    T y;
    T z;
    Point3D():x(0),y(0),z(0){}
    template <typename T1> Point3D(T1 xi,T1 yi,T1 zi):x(xi),y(yi),z(zi) {}
};

template <typename T>
struct Point4D{
    T x;
    T y;
    T z;
    T t;
    Point4D():x(0),y(0),z(0),t(0){}
    template <typename T1> Point4D(T1 xi,T1 yi,T1 zi,T1 ti):x(xi),y(yi),z(zi),t(ti) {}
};


typedef Point4D<double> Point4DD;

#endif // COMMON_TYPES_H
