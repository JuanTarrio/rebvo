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

#ifndef TOON_UTIL_H
#define TOON_UTIL_H

#include <TooN/TooN.h>
#include <fstream>
namespace  rebvo{
namespace util{

inline TooN::Matrix <3,3> Matrix3x3Inv(const TooN::Matrix <3,3> &A){
    TooN::Matrix <3,3> B;

    B(0,0)=  A(2,2)*A(1,1)-A(2,1)*A(1,2); B(0,1)=-(A(2,2)*A(0,1)-A(2,1)*A(0,2));B(0,2)=  A(1,2)*A(0,1)-A(1,1)*A(0,2);
    B(1,0)=-(A(2,2)*A(1,0)-A(2,0)*A(1,2));B(1,1)=  A(2,2)*A(0,0)-A(2,0)*A(0,2); B(1,2)=-(A(1,2)*A(0,0)-A(1,0)*A(0,2));
    B(2,0)=  A(2,1)*A(1,0)-A(2,0)*A(1,1); B(2,1)=-(A(2,1)*A(0,0)-A(2,0)*A(0,1));B(2,2)=  A(1,1)*A(0,0)-A(1,0)*A(0,1);


    return B/TooN::determinant(A);
}



template <int Ti,int Tj>
inline bool isNaN(const TooN::Matrix<Ti,Tj> M){
    for(int i=0;i<Ti;i++)
        for(int j=0;j<Tj;j++)
            if(std::isnan(M(i,j)))
                return true;
    return false;

}
template <int Ti>
inline bool isNaN(const TooN::Vector<Ti> M){
    for(int i=0;i<Ti;i++)
            if(std::isnan(M[i]))
                return true;
    return false;

}

inline TooN::Vector<4> LieRot2Quaternion(TooN::Vector<3>W){
    TooN::Vector<4> q;
    double angle=norm(W);
    if(angle>0)
        q.slice<0,3>()=W/angle*sin(angle/2);
    else
        q.slice<0,3>()=TooN::Zeros;
    q[3]=cos(angle/2);
    return q;
}


inline TooN::Matrix<3,3> Quaternion2RotMat(TooN::Vector<4>q){

    normalize(q);

    return TooN::Data( 1 - 2* util::square(q[2]) - 2* util::square(q[3]),
        2*q[1]*q[2] - 2*q[0]*q[3],
        2*q[3]*q[1] + 2*q[0]*q[2],

        2*q[1]*q[2] + 2*q[0]*q[3],
        1 - 2* util::square(q[1]) - 2* util::square(q[3]),
        2*q[2]*q[3] - 2*q[0]*q[1],

        2*q[3]*q[1] - 2*q[0]*q[2],
        2*q[2]*q[3] + 2*q[0]*q[1],
        1 - 2* util::square(q[1]) - 2* util::square(q[2]));
}


inline TooN::Matrix <3,3> crossMatrix(TooN::Vector <3> v){    //[v]x*u = v x u
    return TooN::Data(   0 ,-v[2], v[1],\
                       v[2],  0  ,-v[0],\
                      -v[1], v[0],  0  );
}

template <typename T>
inline void dumpElement(std::ofstream &file,const T &e){

    file.write((const char*)&e,sizeof(T));

}

template <int size>
inline void dumpMatrix(std::ofstream &file,const TooN::Matrix <size,size> &M){

    for(int i=0;i<size;i++){
        for(int j=0;j<size;j++){
            double d=M(i,j);
            file.write((const char*)&d,sizeof(d));
        }
    }

}

template <int size>
inline void dumpVector(std::ofstream &file,const TooN::Vector <size> &V){

    for(int i=0;i<size;i++){
            double d=V[i];
            file.write((const char*)&d,sizeof(d));
    }

}
template <typename T>
inline void readElement(std::ifstream &file,const T &e){

    file.read((char*)&e,sizeof(T));

}

template <int size>
inline void readMatrix(std::ifstream &file,TooN::Matrix <size,size> &M){

    for(int i=0;i<size;i++){
        for(int j=0;j<size;j++){
            double d;
            file.read((char*)&d,sizeof(d));
            M(i,j)=d;
        }
    }

}

template <int size>
inline void readVector(std::ifstream &file,TooN::Vector <size> &V){

    for(int i=0;i<size;i++){
            double d;
            file.read((char*)&d,sizeof(d));
            V[i]=d;
    }

}




}
}

#endif // TOON_UTIL_H
