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

#ifndef KEYFRAME_H
#define KEYFRAME_H


#include "edge_tracker.h"
#include "global_tracker.h"
#include "visualizer/depth_filler.h"
#include <memory>
namespace  rebvo{
class keyframe
{



    std::shared_ptr<edge_tracker> et;
    std::shared_ptr<global_tracker>gt;
    std::shared_ptr<depth_filler>df;




public:
    double t;
    double K;
    TooN::Matrix <3,3> Rot;
    TooN::Vector <3> RotLie;
    TooN::Vector <3> Vel;


    TooN::Matrix <3,3> Pose;
    TooN::Vector <3> PoseLie;
    TooN::Vector <3> Pos;

    cam_model camera;


public:
    keyframe(edge_tracker& edges, const global_tracker &gtracker, double frame_t,double scale,
             TooN::Matrix <3,3> _Rot, TooN::Vector <3> _RotLie, TooN::Vector <3> _Vel,
             TooN::Matrix <3,3> _Pose, TooN::Vector <3> _PoseLie, TooN::Vector <3> _Pos);

    keyframe(edge_tracker& edges, double frame_t,double scale,
             TooN::Matrix <3,3> _Rot, TooN::Vector <3> _RotLie, TooN::Vector <3> _Vel,
             TooN::Matrix <3,3> _Pose, TooN::Vector <3> _PoseLie, TooN::Vector <3> _Pos);


    keyframe();
    ~keyframe();

    void dumpToBinaryFile(std::ofstream &file);
    void loadFromBinaryFile(std::ifstream &file);

    static bool saveKeyframes2File(const char *name,std::vector<keyframe> &kf_list);
    static bool loadKeyframesFromFile(const char *name,std::vector<keyframe> &kf_list);


    edge_tracker & edges(){return *et;}
    global_tracker & tracker(){return *gt;}

    void initDepthFiller(Size2D blockSize, int iter_num, double error_thresh, double m_num_thresh, depth_filler::bound_modes bound_mode, bool discart=true);
    bool depthFillerAval(){
        return df!=nullptr;
    }
    depth_filler& depthFill(){
        return *df;
    }


    TooN::Vector<3> World2Local(const TooN::Vector<3> &w) const {
        return Pose.T()*(w-Pos);
    }
    TooN::Vector<3> Local2World(const TooN::Vector<3> &p) const {
        return Pose*p+Pos;
    }
    TooN::Vector<3> World2LocalScaled(const TooN::Vector<3> &w) const {
        return Pose.T()*(w-Pos)/K;
    }
    TooN::Vector<3> Local2WorldScaled(const TooN::Vector<3> &p) const {
        return Pose*p*K+Pos;
    }

    TooN::Vector<3> transformTo(const keyframe &kfto,const TooN::Vector<3> &p) const {

        return kfto.Pose.T()*(Pose*p+Pos-kfto.Pos);

    }
    TooN::Vector<3> reProjectTo(const keyframe &kfto,const TooN::Vector<3> &q) const {

        return kfto.camera.projectHomCordVec(kfto.World2LocalScaled(Local2WorldScaled(camera.unprojectHomCordVec(q))));

    }

};
}
#endif // KEYFRAME_H
