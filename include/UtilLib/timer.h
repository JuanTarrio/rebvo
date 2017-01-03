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

#ifndef TIMER_H
#define TIMER_H


#include <chrono>
#include <ctime>
#include <vector>
namespace  rebvo{
namespace util{



class timer{

     std::chrono::time_point<std::chrono::steady_clock> tstart=std::chrono::steady_clock::now();
     std::chrono::time_point<std::chrono::steady_clock> tstop=std::chrono::steady_clock::now();

     bool running=false;

public:

     template <typename Prec=double>
     Prec start(){
         Prec elap=elapsed<Prec>();
         tstart=std::chrono::steady_clock::now();
         running=true;
         return elap;
     }


     template <typename Prec=double>
     Prec stop(){
         tstop=std::chrono::steady_clock::now();
         running=false;
         return elapsed<Prec>();
     }


     template <typename Prec=double>
     Prec elapsed(){
         std::chrono::duration<Prec> diff = running?std::chrono::steady_clock::now()-tstart:tstop-tstart;
         return diff.count();
     }

     operator double() {
         return elapsed();
     }

};


class interval_list{


    std::vector <std::chrono::time_point<std::chrono::steady_clock> > tlist;


public:

    void clear (){
        tlist.clear();
    }

    void push_new(){
        tlist.push_back(std::chrono::steady_clock::now());
    }

    int size(){
        return tlist.size()-1;
    }

    template <typename Prec=double>
    Prec operator () (uint i_end,uint i_start){

        if(i_start>=tlist.size() || i_end>=tlist.size())
            return 0;

        std::chrono::duration<Prec> && diff=tlist[i_end] - tlist[i_start];
        return diff.count();
    }

    template <typename Prec=double>
    Prec operator [] (uint inx){
        if(inx+1>=tlist.size())
            return 0;
        std::chrono::duration<Prec> && diff=tlist[inx+1] - tlist[inx];
        return diff.count();
    }


    template <typename Prec=double>
    Prec total (){
        return tlist.size()>0 ? (*this)(tlist.size()-1,0) : 0;
    }

};

inline double dift(timeval &tv1, timeval &tv2)
{

return (tv1.tv_sec - tv2.tv_sec) + (tv1.tv_usec - tv2.tv_usec)/1.0e6;

}

inline double dift(timespec &tv1, timespec &tv2)
{

return (tv1.tv_sec - tv2.tv_sec) + (tv1.tv_nsec - tv2.tv_nsec)/1.0e9;

}


}
}
#endif // TIMER_H
