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

 
#ifndef FVECTOR_H
#define FVECTOR_H
#include <string.h>

#include <TooN/TooN.h>
namespace  rebvo{
template <int size, typename T=double>
class fvector{

    T data[size];

public:

    fvector(T value){
        for(T &d:data)
            d=value;
    }
    fvector(){fvector(0.0);}
    fvector(T* data_ptr):data(data_ptr){}

    T& operator [](uint inx){
        return data[inx];
    }

    uint Size(){
        return size;
    }

    T* Data(){
        return data;
    }

    fvector<size,T>& operator = (const fvector<size>& copy_from){
        memcpy(data,copy_from.Data(),sizeof(T)*size);
        return *this;
    }

    fvector<size,T>& operator = (const TooN::Vector<size>& copy_from){
        for(int i=0;i<size;i++)
            data[i]=copy_from[i];
        return *this;
    }

    //Conversion to TooN

    template <typename Tv>
    operator TooN::Vector<size,Tv> (){

        TooN::Vector<size,Tv> &&t=TooN::Vector<size,Tv>();
        for(int i=0;i<size;i++)
            t[i]=data[i];
        return t;
    }

    TooN::Vector<size,T> toTooN(){
        return (TooN::Vector<size,T>)*this;
    }

    //Iterators


    typedef T * iterator;
    typedef const T * const_iterator;

    iterator begin(){return &data[0];}
    iterator end(){return &data[size];}

};

}
#endif // FVECTOR_H
