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
