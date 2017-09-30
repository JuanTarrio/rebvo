#ifndef LOCAL_PG_H
#define LOCAL_PG_H

#include "mtracklib/keyframe.h"

namespace  rebvo {


struct VI_Odometry{

    TooN::Vector<6> relPosPose;
    TooN::Matrix<6,6> WrelPosPose;
};

struct LocalPGOptimizer_isam;

class LocalPGOptimizer{

    std::vector <VI_Odometry> frame_2_frame_odo;
    std::vector <VI_Odometry> frame_2_kf_odo;

    LocalPGOptimizer_isam *isam_graph;

public:
    LocalPGOptimizer(TooN::Vector<6> priorPose, TooN::Matrix<6, 6> priorPoseW);
    ~LocalPGOptimizer();

    void pushFrame2FrameOdo(const VI_Odometry &odo);
    void addKF2FrameOdo(const VI_Odometry &odo);
    void addFrame2KFOdo(const VI_Odometry &odo);

    void solvePoses();

    int numPoses();

    void getPose(int poseId, TooN::Matrix<3, 3> &Pose, TooN::Vector<3> &Pos);
};
}
#endif // LOCAL_PG_H
