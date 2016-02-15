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

#ifndef CIRCLIST_H
#define CIRCLIST_H


namespace util{


template <class T,int size> class FixedCircList{

private:
    int inx;
    int back_inx;

    int num;
    int data_size;

    T data[size];

public:

    FixedCircList(){
        inx=size-1;
        num=0;
        data_size=size;
        back_inx=inx;
    }

    void Init(const T&v0){
        data[0]=v0;
        inx=0;
        back_inx=0;
        num=1;
    }

    void Put(const T&v){
        if((++inx)>=size)
            inx=0;
        if(num<size)
            num++;
        data[inx]=v;
    }

    void PutBack(const T&v){
        if((--back_inx)<0){
            back_inx+=size;
        }
        if(num<size)
            num++;

        data[back_inx]=v;
    }

    T& GetRelative(int i){
        i=inx-(i%size);
        if(i<0)
            i+=size;

        return data[i];

    }


    T& Get(int i){

        if(i>num)
            return 0;

        return data[i];

    }

    T& GetNewest(){
        return data[inx];
    }

    T& GetOldest(){
        if(num<size)
            return data[0];
        else
            return data[(inx+1)%size];
    }

    T& operator [](int i)
    {
        return data[i%size];
    }

    void CopyTo(FixedCircList<T,size> * to){

        to->num=num;
        to->inx=inx;

        for(int i=0;i<num;i++)
            to->data[i]=data[i];

    }

    void CopyToOrdered(FixedCircList<T,size> * to){

        if(num<size){
            CopyTo(to);
            return;
        }

        int i_from=inx+2;


        for(int i=0;i<num;i++,i_from++){
            if(i_from>=size)
                i_from-=size;
            to->data[i]=data[i_from];
        }

        to->num=num;
        to->inx=size-2;


    }

    const int&Size(){
        return num;
    }

    const int&ListSize(){
        return data_size;
    }


};



class CircListIndexer{

    int inx;
    int num;
    const uint size;

public:

    CircListIndexer(uint list_size) : inx(0),num(0),size(list_size){}

    int operator+(const int &r){
        return (inx+(r%size)+size)%size;
    }

    int operator-(const int &r){
        return (inx-(r%size)+size)%size;
    }

    CircListIndexer& operator++(int){

        inx=(inx+1)%size;

        return *this;
    }


    CircListIndexer& operator--(int){

        inx=(inx-1+size)%size;

        return *this;
    }

    operator int()
    {
        return inx;
    }

    CircListIndexer& AddOne(){
        if(num<size)
            num++;
        inx=(inx+1)%size;
        return *this;
    }

    const int& NumElem(){
        return num;
    }

    uint Size(){
        return size;
    }
};




}

#endif // CIRCLIST_H
