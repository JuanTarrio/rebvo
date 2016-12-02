#ifndef SCALEESTIMATOR_H
#define SCALEESTIMATOR_H

#include "CircList.h"
#include "TooN/Cholesky.h"
#include <iostream>
#include <fstream>

class ScaleEstimator
{
public:
    ScaleEstimator();

    static void EstAcelLsq4(const TooN::Vector <3> &vel,TooN::Vector <3>  &acel, \
                            const TooN::Matrix<3,3> &R,\
                            const double &dt);

    static void MeanAcel4(const TooN::Vector <3> &s_acel,TooN::Vector <3> &acel, \
                                   const TooN::Matrix<3,3> &R);




    static double estKaGMEKBias(const TooN::Vector<3> &s_acel, const TooN::Vector<3> &f_acel, double kP, TooN::Matrix<3,3> Rot, TooN::Vector <7> &X, TooN::Matrix <7,7> &P, const TooN::Matrix<3, 3> &Qrot, const TooN::Matrix<3, 3> &Qbias, const double &QKp, \
                            const double &Rg, const TooN::Matrix<3, 3> &Rs, const TooN::Matrix<3, 3> &Rf, TooN::Vector<3> & g_est, TooN::Vector<3> &b_est, const TooN::Matrix<6, 6> &Wvb, TooN::Vector <6> &Xvw, double g_gravit);



};



#endif // SCALEESTIMATOR_H
