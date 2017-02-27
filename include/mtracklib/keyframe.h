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

    void initDepthFiller(Size2D blockSize, int iter_num, double error_thresh, double m_num_thresh, depth_filler::bound_modes bound_mode);
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
};
}
#endif // KEYFRAME_H
