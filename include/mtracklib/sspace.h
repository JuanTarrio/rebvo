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

#ifndef SSPACE_H
#define SSPACE_H

#include "mtracklib/iigauss.h"
#include "VideoLib/video_io.h"
namespace  rebvo{
class sspace
{

private:

    int bf_n;


    iigauss filter0;
    iigauss filter1;

    Image<DetectorImgType> img[2];
    Image<DetectorImgType> dog;
    Image<DetectorImgType> img_dx;
    Image<DetectorImgType> img_dy;

public:




    sspace(double sigma0, double k_sigma, const Size2D &size, int bf_num);

    void calc_gradient();

    void build(Image<DetectorImgType> &data);
    void build_dog();

    Image<DetectorImgType> &Img(int inx){return img[inx];}
    Image<DetectorImgType> &ImgDx(){return img_dx;}
    Image<DetectorImgType> &ImgDy(){return img_dy;}
    Image<DetectorImgType> &ImgDOG(){return dog;}


};
}
#endif // SSPACE_H
