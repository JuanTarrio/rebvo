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

#include "mtracklib/sspace.h"
#include <iostream>
namespace  rebvo{
/*

  Esta clase implementa el espacio de escalas (Lindenberg 94,98 y otros...) a la Lowe et. al. (SIFT)
  Cada octaba subsamplea por dos la octaba anterior, para ahorrar tiempo de cálculo.
  Sin embargo, si se van a buscar features en s_n escalas, deben calcularse s_n+3 escalas por octaba
  ya que debe contarse con s_n+2 DoG para buscar maximos en espacio de escala

*/

sspace::sspace(double sigma0,
               double k_sigma,
               const Size2D &size,
               int bf_num)
    :bf_n(bf_num),
      filter0(size,sigma0,bf_num),filter1(size,filter0.sigma_r*k_sigma,bf_num),
      img{Image<DetectorImgType>(size),Image<DetectorImgType>(size)},dog(size),img_dx(size),img_dy(size)
{




}



void sspace::build(Image<DetectorImgType> &data){


    filter0.smooth(data,img[0]);    //calcula los filtros usando la imagen integral
    filter1.smooth(data,img[1]);    //calcula los filtros usando la imagen integral

    build_dog();                        //build DoG
    calc_gradient();                    //Calc gradient
}


void sspace::build_dog(){


    for(int k=0;k<dog.bSize();k++){
        dog[k]=img[1][k]-img[0][k];
    }

}


// calc_gradient() calcula el gradiante con respecto a x e y para una escala determinada

void sspace::calc_gradient(){


    for(int y=1;y<img_dx.Size().h-1;y++){
        for(int x=1;x<img_dx.Size().w-1;x++){
            img_dx(x,y)=img[0](x+1,y)-img[0](x-1,y);
            img_dy(x,y)=img[0](x,y+1)-img[0](x,y-1);
        }
    }

}
}
