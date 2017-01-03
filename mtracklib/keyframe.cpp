#include "keyframe.h"
namespace  rebvo{
keyframe::keyframe(edge_tracker& edges,const global_tracker &gtracker,double frame_t
                   ,TooN::Matrix <3,3> _Rot,TooN::Vector <3> _RotLie,TooN::Vector <3> _Vel,
                   TooN::Matrix <3,3> _Pose,TooN::Vector <3> _PoseLie,TooN::Vector <3> _Pos)
    :et(new edge_tracker(edges)),gt(new global_tracker(gtracker)),t(frame_t),camera(edges.GetCam())
{

    gt->SetEdgeTracker(et.get());


    Rot=_Rot;
    RotLie=_RotLie;
    Vel=_Vel;
    Pose=_Pose;
    PoseLie=_PoseLie;
    Pos=_Pos;
}

keyframe::keyframe()
    :et(nullptr),gt(nullptr)
{
}


keyframe::~keyframe(){
}

inline void dumpMatrix(std::ofstream &file,const TooN::Matrix <3,3> &M){

    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            double d=M(i,j);
            file.write((const char*)&d,sizeof(d));
        }
    }

}

inline void dumpVector(std::ofstream &file,const TooN::Vector <3> &V){

    for(int i=0;i<3;i++){
            double d=V[i];
            file.write((const char*)&d,sizeof(d));
    }

}

inline void readMatrix(std::ifstream &file,TooN::Matrix <3,3> &M){

    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            double d;
            file.read((char*)&d,sizeof(d));
            M(i,j)=d;
        }
    }

}

inline void readVector(std::ifstream &file,TooN::Vector <3> &V){

    for(int i=0;i<3;i++){
            double d;
            file.read((char*)&d,sizeof(d));
            d=V[i];
    }

}



void keyframe::dumpToBinaryFile(std::ofstream &file){

    file.write((const char*)&t,sizeof(t));

    dumpMatrix(file,Rot);
    dumpVector(file,RotLie);
    dumpVector(file,Vel);
    dumpMatrix(file,Pose);
    dumpVector(file,PoseLie);
    dumpVector(file,Pos);

    double d=gt->getMaxSRadius();
    file.write((const char*)&d,sizeof(d));

    file.write((const char*)&camera,sizeof(camera));

    et->dumpToBinaryFile(file);

}

void keyframe::loadFromBinaryFile(std::ifstream &file){

    double t;
    file.read((char*)&t,sizeof(t));

    TooN::Matrix <3,3> Rot;
    TooN::Vector <3> RotLie;
    TooN::Vector <3> Vel;


    TooN::Matrix <3,3> Pose;
    TooN::Vector <3> PoseLie;
    TooN::Vector <3> Pos;

    readMatrix(file,Rot);
    readVector(file,RotLie);
    readVector(file,Vel);
    readMatrix(file,Pose);
    readVector(file,PoseLie);
    readVector(file,Pos);

    double max_r;
    file.read((char*)&max_r,sizeof(max_r));

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
}
