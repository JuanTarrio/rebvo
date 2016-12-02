#ifndef KEYFRAME_H
#define KEYFRAME_H


#include "edge_tracker.h"
#include "global_tracker.h"
#include <memory>

class keyframe
{



    std::shared_ptr<edge_tracker> et;
    std::shared_ptr<global_tracker>gt;
    double t;

    TooN::Matrix <3,3> Rot;
    TooN::Vector <3> RotLie;
    TooN::Vector <3> Vel;


    TooN::Matrix <3,3> Pose;
    TooN::Vector <3> PoseLie;
    TooN::Vector <3> Pos;

    cam_model camera;


public:
    keyframe(edge_tracker& edges, const global_tracker &gtracker, double frame_t,
             TooN::Matrix <3,3> _Rot, TooN::Vector <3> _RotLie, TooN::Vector <3> _Vel,
             TooN::Matrix <3,3> _Pose, TooN::Vector <3> _PoseLie, TooN::Vector <3> _Pos);

    keyframe();
    ~keyframe();

    void dumpToBinaryFile(std::ofstream &file);
    void loadFromBinaryFile(std::ifstream &file);

    static bool saveKeyframes2File(const char *name,std::vector<keyframe> &kf_list);
    static bool loadKeyframesFromFile(const char *name,std::vector<keyframe> &kf_list);
};

#endif // KEYFRAME_H
