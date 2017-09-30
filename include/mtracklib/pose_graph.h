#ifndef POSE_GRAPH_H
#define POSE_GRAPH_H
#include "mtracklib/keyframe.h"

namespace  rebvo {


struct OdometryMeas{

    TooN::Vector<6> relPosPose;
    TooN::Matrix<6,6> WrelPosPose;


    TooN::Vector<3> AcelMeas;
    TooN::Vector<3> VisualMeas;
    TooN::Vector<3> GMeas;
    TooN::Matrix<3,3> Rs;
    TooN::Matrix<3,3> Rv;
    TooN::Matrix<3,3> Rg;
    double K;
    double WK;
    double QK;
    int KF_ID;

    OdometryMeas(const TooN::Vector<6> &relPosPose_,const TooN::Matrix<6> &WrelPosPose_,const TooN::Vector<3> &AcelMeas_,const TooN::Vector<3> &VisualMeas_,const TooN::Vector<3> &GMeas_,const TooN::Matrix<3,3> &Rs_,const TooN::Matrix<3,3> &Rv_,const TooN::Matrix<3,3> &Rg_,const double &K_,const double &WK_,const double &QK_,int KF_ID_):
        relPosPose(relPosPose_),WrelPosPose(WrelPosPose_),AcelMeas(AcelMeas_),VisualMeas(VisualMeas_),GMeas(GMeas_),Rs(Rs_),Rv(Rv_),Rg(Rg_),K(K_),WK(WK_),QK(QK_),KF_ID(KF_ID_){}

    OdometryMeas(){}


//****** Serialization Funcions ***************

    void dumpToFile(std::ofstream &file){
        util::dumpVector(file,relPosPose);
        util::dumpMatrix(file,WrelPosPose);
        util::dumpVector(file,AcelMeas);
        util::dumpVector(file,VisualMeas);
        util::dumpVector(file,GMeas);
        util::dumpMatrix(file,Rs);
        util::dumpMatrix(file,Rv);
        util::dumpMatrix(file,Rg);
        util::dumpElement(file,K);
        util::dumpElement(file,WK);
        util::dumpElement(file,QK);
        util::dumpElement(file,KF_ID);
    }
    void readFromFile(std::ifstream &file){
        util::readVector(file,relPosPose);
        util::readMatrix(file,WrelPosPose);
        util::readVector(file,AcelMeas);
        util::readVector(file,VisualMeas);
        util::readVector(file,GMeas);
        util::readMatrix(file,Rs);
        util::readMatrix(file,Rv);
        util::readMatrix(file,Rg);
        util::readElement(file,K);
        util::readElement(file,WK);
        util::readElement(file,QK);
        util::readElement(file,KF_ID);
    }

};
class isam_data;

class pose_graph
{
    std::vector <OdometryMeas> frame_meas;
    std::vector <std::pair<double,double>> smooth_scale;
    std::vector <std::pair<double,double>> smooth_scale_z;

    isam_data * isam_graph=nullptr;

public:
    pose_graph();

    void addFrameMeas(const OdometryMeas & m){
        frame_meas.push_back(m);

    }

    void reset(){
        frame_meas.resize(0);
    }

    void saveToFile(std::ofstream &file);
    void loadFromFile(std::ifstream &file);
    bool saveToFile(const char *fname){
        std::ofstream file(fname);

        if(!file.is_open())
            return false;
        saveToFile(file);
        file.close();
        return true;

    }

    bool  loadFromFile(const char *fname){
        std::ifstream file(fname);

        if(!file.is_open())
            return false;
        loadFromFile(file);
        file.close();
        return true;
    }


    std::vector <OdometryMeas> & odoMeas(){return frame_meas;}
    void loadIsam(std::vector<keyframe> &kf_list);

    void addOdoMeasScaled(TooN::Vector<6> relPose, TooN::Matrix<6,6> relPosCov, double K, int kf_from, int kf_to);
    void addOdoMeasWithScale(TooN::Vector <6> relPose, TooN::Matrix<6, 6> relPosCov,int kf_from,int kf_to);
    void addOdoMeasWithAngScale(TooN::Vector <6> relPose, TooN::Matrix<6, 6> relPosCov,int kf_from,int kf_to);

    void optimizeAndPrint(bool update);

    void getKfPose(int kf_id, TooN::Matrix<3,3> &Pose, TooN::Vector<3> &Pos, double &K);
    void loadIsamWithScale();
    void loadIsamWithScaleDecoupled();
    void loadIsamWithScaleMeas();
    void smoothScale();
    void buildScale();
    void buildAngScale();
    void loadIsamWithScaleMonster();
    void loadIsamWithScaleMonocular();
    void addScaleOdometry(double Kr, double WKr, int kf_from, int kf_to);
};
}
#endif // POSE_GRAPH_H
