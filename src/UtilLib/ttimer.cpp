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

#include "UtilLib/ttimer.h"
#include "UtilLib/util.h"
#include <stdio.h>
namespace  rebvo{
TTimer GlobalTimer;

TTimer::TTimer()
{
    init=false;

    simu_time_on=false;
    timer_thread=0;

    pthread_mutex_init(&mutex,NULL);
}

TTimer::~TTimer(){

    if(simu_time_on)
        TurnSimuOff();
}

bool TTimer::TurnSimuOff(){

    if(timer_thread==0 && !simu_time_on)
        return false;

    simu_time_on=false;

    pthread_mutex_lock(&t_mutex);
    t_quit=true;
    pthread_mutex_unlock(&t_mutex);

    void *r;
    pthread_join(timer_thread,&r);



    return true;
}


double TTimer::GetTimer()
{

    double t=0;
    pthread_mutex_lock(&mutex);

    if(!init){

        init=true;
        ClockGetTime(CLOCK_MONOTONIC,&t0);

    }else{

        timespec t1;
        ClockGetTime(CLOCK_MONOTONIC,&t1);
        t=util::dift(t1,t0);

    }


    pthread_mutex_unlock(&mutex);

    return t;

}


int TTimer::ClockGetTime(clockid_t clk_id, struct timespec *res){

    if(simu_time_on){


        pthread_mutex_lock(&t_mutex);
        *res=simu_time;

        pthread_mutex_unlock(&t_mutex);
        return 0;

    }else{
        return clock_gettime(clk_id,res);
    }

}


bool TTimer::TurnSimuOn(long ns_t_step,double time_sweep,double t_start){

    clock_gettime(CLOCK_MONOTONIC,&simu_time);

    pthread_mutex_init(&t_mutex,NULL);

    t_quit=false;
    simu_t_step=ns_t_step;
    simu_t_sweep=time_sweep;

    simu_time.tv_sec=0;
    simu_time.tv_nsec=0;

    simu_time_on=true;

    this->GetTimer();

    simu_time.tv_sec=t_start;
    simu_time.tv_nsec=(t_start-(double)simu_time.tv_sec)*1.0e9;

    if (pthread_create(&timer_thread, NULL,&TimerThread, this))
    {
        return false;
    }


    return true;

}


void* TTimer::TimerThread(void *arg){

    TTimer *tt=(TTimer*)arg;

    bool quit=false;

    pthread_mutex_lock(&tt->t_mutex);
    long int t_step=tt->simu_t_step;
    double t_sweep=tt->simu_t_sweep;
    pthread_mutex_unlock(&tt->t_mutex);

    timespec t_sleep;
    sched_param sp;
    sp.__sched_priority=1;
    if(pthread_setschedparam(pthread_self(),SCHED_FIFO,&sp)!=0)
        printf(" \nGlobal Timer: error setting simu-thread prioroty\n");



    while(!quit){


        pthread_mutex_lock(&tt->t_mutex);

        tt->simu_time.tv_nsec+=t_step;

        if(tt->simu_time.tv_nsec>=1e9){
            tt->simu_time.tv_sec++;
            tt->simu_time.tv_nsec-=1e9;
        }

        quit=tt->t_quit;
        pthread_mutex_unlock(&tt->t_mutex);

        t_sleep.tv_sec=0;
        t_sleep.tv_nsec=t_step*t_sweep;
        nanosleep(&t_sleep,NULL);

    }


    return NULL;


}
}
