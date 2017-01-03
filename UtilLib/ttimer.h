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

#ifndef TTIMER_H
#define TTIMER_H

#include <pthread.h>
#include <time.h>
namespace  rebvo{

class TTimer
{

    timespec t0;
    bool init;
    pthread_mutex_t mutex;

    bool simu_time_on;

    timespec simu_time;

    pthread_t timer_thread;

    bool t_quit;


    pthread_mutex_t t_mutex;

    long int simu_t_step;
    double simu_t_sweep;

public:
    TTimer();
    ~TTimer();

    double GetTimer();

    int ClockGetTime(clockid_t clk_id, timespec *res);

    bool TurnSimuOn(long int ns_t_step, double time_sweep, double t_start);


    bool TurnSimuOff();


    static void	*TimerThread(void *arg);

};


extern TTimer GlobalTimer;
}
#endif // TTIMER_H
