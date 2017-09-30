
#include "mtracklib/isam_defs.h"
#include "mtracklib/local_pg.h"
#include "mtracklib/pose_graph.h"

namespace  rebvo {
using namespace isam;
struct LocalPGOptimizer_isam{
    Slam slam;
    std::vector <Pose3d_Node*>FramePoses;
};


LocalPGOptimizer::~LocalPGOptimizer(){
    delete isam_graph;
}

LocalPGOptimizer::LocalPGOptimizer(TooN::Vector<6> priorPose,TooN::Matrix<6,6> priorPoseW)
    :isam_graph(new LocalPGOptimizer_isam)
{


    Pose3d_Node* KfPose = new Pose3d_Node();
    isam_graph->FramePoses.push_back(KfPose);

    isam_graph->slam.add_node(KfPose);

    // create a prior on the keyframe position
    isam_graph->slam.add_factor(new Pose3d_Factor(KfPose,
                                                  revboRP2isam(priorPose),
                                                  Information(matrix_toon2eigen(priorPoseW))));
}

void LocalPGOptimizer::pushFrame2FrameOdo(const VI_Odometry &odo){
    frame_2_frame_odo.push_back(odo);

    isam_graph->FramePoses.push_back(new Pose3d_Node());


    Pose3d_Node* pose0=isam_graph->FramePoses[isam_graph->FramePoses.size()-2];
    Pose3d_Node* pose1=isam_graph->FramePoses[isam_graph->FramePoses.size()-1];
    isam_graph->slam.add_node(pose1);

    Noise rel_noise = Information(matrix_toon2eigen(odo.WrelPosPose));
    Pose3d delta=revboRP2isam(odo.relPosPose);

    isam_graph->slam.add_factor(new Pose3d_Pose3d_Factor(pose0, pose1, delta, rel_noise));

}
void LocalPGOptimizer::addKF2FrameOdo(const VI_Odometry &odo){
    frame_2_kf_odo.push_back(odo);

    Pose3d_Node* pose0=isam_graph->FramePoses[0];
    Pose3d_Node* pose1=isam_graph->FramePoses[isam_graph->FramePoses.size()-1];

    Noise rel_noise = Information(matrix_toon2eigen(odo.WrelPosPose));
    Pose3d delta=revboRP2isam(odo.relPosPose);

    isam_graph->slam.add_factor(new Pose3d_Pose3d_Factor(pose1, pose0, delta, rel_noise));

}

void LocalPGOptimizer::addFrame2KFOdo(const VI_Odometry &odo){
    frame_2_kf_odo.push_back(odo);

    Pose3d_Node* pose0=isam_graph->FramePoses[0];
    Pose3d_Node* pose1=isam_graph->FramePoses[isam_graph->FramePoses.size()-1];

    Noise rel_noise = Information(matrix_toon2eigen(odo.WrelPosPose));
    Pose3d delta=revboRP2isam(odo.relPosPose);

    isam_graph->slam.add_factor(new Pose3d_Pose3d_Factor(pose0,pose1, delta, rel_noise));

}

void LocalPGOptimizer::solvePoses()
{
    Properties prop = isam_graph->slam.properties();
    prop.method = DOG_LEG;
    prop.max_iterations=100;
    isam_graph->slam.set_properties(prop);
    isam_graph->slam.batch_optimization();
}


int LocalPGOptimizer::numPoses(){
    return isam_graph->FramePoses.size();
}

void LocalPGOptimizer::getPose(int poseId, TooN::Matrix<3, 3> &Pose, TooN::Vector<3> &Pos)
{
    if(poseId<isam_graph->FramePoses.size() ){
        const Pose3d &pose=isam_graph->FramePoses[poseId]->value();
        TooN::Matrix<4,4> t44=matrix_eigen2toon(pose.wTo());
        Pose=t44.slice<0,0,3,3>();
        Pos=t44.T()[3].slice<0,3>();
    }else{
        std::cout<<"\n LocalPGOptimizer::getPose(): poseId out of size!";
    }

}

}
