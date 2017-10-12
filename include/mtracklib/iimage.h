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

#ifndef IIMAGE_H
#define IIMAGE_H

#include <math.h>

#include "UtilLib/util.h"
#include "VideoLib/image.h"
namespace  rebvo{
typedef float DetectorImgType;

class iimage
{

protected:
    Image<DetectorImgType> img_data;

    const uint w;
    const uint h;

public:

    iimage(const Size2D &size, Image<DetectorImgType> *i_load=0);
    ~iimage();
    void load(Image<DetectorImgType> &data);

    static void average(Image<DetectorImgType> & buf, Image<DetectorImgType> &img_data, int d, Image<DetectorImgType> &div);
    static void build_average(int d,Image<DetectorImgType> &div);

};
}
#endif // IIMAGE_H
