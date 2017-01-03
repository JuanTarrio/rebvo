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

#ifndef SCALEESTIMATOR_H
#define SCALEESTIMATOR_H

#include "UtilLib/CircList.h"
#include "TooN/Cholesky.h"
#include <iostream>
#include <fstream>
namespace  rebvo{
class ScaleEstimator
{
public:
    ScaleEstimator();

    static void EstAcelLsq4(const TooN::Vector <3> &vel,TooN::Vector <3>  &acel, \
                            const TooN::Matrix<3,3> &R,\
                            const double &dt);

    static void MeanAcel4(const TooN::Vector <3> &s_acel,TooN::Vector <3> &acel, \
                                   const TooN::Matrix<3,3> &R);




    static double estKaGMEKBias(const TooN::Vector<3> &s_acel, const TooN::Vector<3> &f_acel, double kP, TooN::Matrix<3,3> Rot, TooN::Vector <7> &X, TooN::Matrix <7,7> &P,
                                const TooN::Matrix <3,3> &Qg, const TooN::Matrix<3, 3> &Qrot, const TooN::Matrix<3, 3> &Qbias, const double &QKp, \
                            const double &Rg, const TooN::Matrix<3, 3> &Rs, const TooN::Matrix<3, 3> &Rf, TooN::Vector<3> & g_est, TooN::Vector<3> &b_est, const TooN::Matrix<6, 6> &Wvb, TooN::Vector <6> &Xvw, double g_gravit);



};


}
#endif // SCALEESTIMATOR_H
