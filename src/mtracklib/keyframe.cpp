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



#include "mtracklib/keyframe.h"
namespace  rebvo{
keyframe::keyframe(edge_tracker& edges,const global_tracker &gtracker,double frame_t,double scale
                   ,TooN::Matrix <3,3> _Rot,TooN::Vector <3> _RotLie,TooN::Vector <3> _Vel,
                   TooN::Matrix <3,3> _Pose,TooN::Vector <3> _PoseLie,TooN::Vector <3> _Pos)
    :et(new edge_tracker(edges)),gt(new global_tracker(gtracker)),df(nullptr),t(frame_t),K(scale),camera(edges.GetCam())
{

    gt->SetEdgeTracker(et.get());


    Rot=_Rot;
    RotLie=_RotLie;
    Vel=_Vel;
    Pose=_Pose;
    PoseLie=_PoseLie;
    Pos=_Pos;
}

keyframe::keyframe(edge_tracker& edges,double frame_t,double scale
                   ,TooN::Matrix <3,3> _Rot,TooN::Vector <3> _RotLie,TooN::Vector <3> _Vel,
                   TooN::Matrix <3,3> _Pose,TooN::Vector <3> _PoseLie,TooN::Vector <3> _Pos)
    :et(new edge_tracker(edges)),gt(nullptr),df(nullptr),t(frame_t),K(scale),camera(edges.GetCam())
{

    //gt->SetEdgeTracker(et.get());


    Rot=_Rot;
    RotLie=_RotLie;
    Vel=_Vel;
    Pose=_Pose;
    PoseLie=_PoseLie;
    Pos=_Pos;
}

keyframe::keyframe()
    :et(nullptr),gt(nullptr),df(nullptr)
{
}


keyframe::~keyframe(){
}



void keyframe::dumpToBinaryFile(std::ofstream &file){

    file.write((const char*)&t,sizeof(t));
    file.write((const char*)&K,sizeof(K));

    util::dumpMatrix(file,Rot);
    util::dumpVector(file,RotLie);
    util::dumpVector(file,Vel);
    util::dumpMatrix(file,Pose);
    util::dumpVector(file,PoseLie);
    util::dumpVector(file,Pos);

    double d=gt->getMaxSRadius();
    file.write((const char*)&d,sizeof(d));

    file.write((const char*)&camera,sizeof(camera));

    et->dumpToBinaryFile(file);


    //std::cout <<"\nDump"<<Pose<<"\n"<<Pos<<"\n";

}

void keyframe::loadFromBinaryFile(std::ifstream &file){

    file.read((char*)&t,sizeof(t));
    file.read((char*)&K,sizeof(K));

    util::readMatrix(file,Rot);
    util::readVector(file,RotLie);
    util::readVector(file,Vel);
    util::readMatrix(file,Pose);
    util::readVector(file,PoseLie);
    util::readVector(file,Pos);



    double max_r;
    file.read((char*)&max_r,sizeof(max_r));

    //std::cout <<"\nRead"<<Pose<<"\n"<<Pos<<"\n"<<max_r;

    file.read((char*)&camera,sizeof(camera));


    et=std::shared_ptr<edge_tracker>(new edge_tracker(camera,0,0));
    gt=std::shared_ptr<global_tracker>(new global_tracker(et->GetCam()));

    et->readFromBinaryFile(file);
    gt->build_field(*et,max_r);


}


bool keyframe::saveKeyframes2File(const char *name,std::vector<keyframe> &kf_list){

    std::ofstream f(name);


    if(!f.is_open())
        return false;

    int kfnum=kf_list.size();
    f.write((const char*)&kfnum,sizeof(kfnum));

    for(keyframe &kf:kf_list){
        kf.dumpToBinaryFile(f);
    }

    f.close();
    return true;

}

bool keyframe::loadKeyframesFromFile(const char *name,std::vector<keyframe> &kf_list){

    std::ifstream f(name);

    if(!f.is_open())
        return false;

    int kfnum;
    f.read((char*)&kfnum,sizeof(kfnum));

    for(int i=0;i<kfnum;i++){

        kf_list.push_back(keyframe());
        kf_list.back().loadFromBinaryFile(f);

    }

    f.close();
    return true;

}

void keyframe::initDepthFiller(Size2D blockSize, int iter_num, double error_thresh, double m_num_thresh,depth_filler::bound_modes bound_mode, bool discart)
{
    df=std::shared_ptr<depth_filler>(new depth_filler(camera,blockSize,bound_mode));

    if(iter_num>0){

        (*df).ResetData();
        (*df).FillEdgeData(*et,error_thresh,m_num_thresh,discart);
        (*df).InitCoarseFine();
        (*df).Integrate(iter_num);
        (*df).computeDistance(TooN::Zeros);
        (*df).kf_ptr=this;
    }
}


}
