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

#ifndef IIGAUSS_H
#define IIGAUSS_H

#include "iimage.h"
namespace  rebvo{
class iigauss : public iimage
{
public:
    int box_n;			    //cantidad de filtros caja
    double sigma;		    //sigma objetivo
    double sigma_r;		    //sigma obtenido
    int *box_d;			    //ancho de cada filtro caja
    Image<DetectorImgType> *div;	//los divisores para cada filtro

    iigauss(const Size2D &size,double sigma,int box_num);
    ~iigauss();

    void smooth(Image<DetectorImgType>& in, Image<DetectorImgType>& out);
    void iismooth(Image<DetectorImgType>& iimg,Image<DetectorImgType>& out);
};
}
#endif // IIGAUSS_H
