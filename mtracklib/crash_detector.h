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


#ifndef CRASH_DETECTOR_H
#define CRASH_DETECTOR_H

#include "video_io.h"

#include "depth_filler.h"




class crash_detector
{

    depth_filler *df;
    int df_n;

    float zf;
    Point2DF pp;
    double min_dist;
public:

    TooN::Vector<3> V;
    double time2impact;
    double dist2impact;

    int int_num;

    crash_detector(depth_filler *df,int df_num,Point2DF pp,float zf);


    double EstimateDistAndRange_BP(const std::vector <Point4DD> &nav_tray, double &time2impact,int npoints=10,double bubble_size=1);
    double EstimateDistAndRange_MT(const std::vector <Point4DD> &nav_tray, double &time2impact,int npoints=10);

    TooN::Vector<3> FilterVel(const std::vector <Point4DD> &nav_tray, int npoints);

    bool TriangleRayInt_MollerTrum(const TooN::Vector <3> &p0,const TooN::Vector <3> &p1,const TooN::Vector <3> &p2,const TooN::Vector <3> &o,const TooN::Vector <3> &d,double &t);

    bool BubblePointInt(const TooN::Vector <3> &p0,const TooN::Vector <3> &o,const TooN::Vector <3> &d,const double &s,double &t);

    double EstimateRefMinDist();

    double GetMinDist(){return min_dist;}
};

#endif // CRASH_DETECTOR_H
