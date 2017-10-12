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


class pose_graph
{

protected:
    std::vector <OdometryMeas> frame_meas;

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

};
}
#endif // POSE_GRAPH_H
