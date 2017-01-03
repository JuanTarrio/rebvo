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



#include "mtracklib/iigauss.h"

#include <iostream>
namespace  rebvo{
/*
    Siguiendo la idea de Kovesi, utilizo aplicaciones sucesivas de filtros de caja, con
    imagenes integrales, para aproximar una filtrao gausseano de gran eficiencia

*/


//El constructor calcula los filtros de caja nesesarios para aproximar un cierto sigma
//(deviacion gausseana), dado la cantintidad de filtros a utilizar ( box_num )
//para mejorar la aproximacion Kovesi propone utilizar dos tamaños de filtro distindo:
//m filtros de tamaño wl y bon_num-m de tamaño wl+2. Ademas, para que el filtro este
//centrado sobre un punto wl debe ser impar (y entero..).

iigauss::iigauss(const Size2D &size, double sigma, int box_num)
    :iimage(size)
{
    this->sigma=sigma;
    box_n=box_num;
    box_d=new int[box_n];	//Tamaño de cada filtro caja, se supone que cada uno puede
    //ser diferente, pero solo se usan solo 2 tamaños.

    double wideal=sqrt(12*sigma*sigma/box_n+1);	//tamaño ideal del filtro caja a utilizar

    int wl=wideal;
    int tmp=wl/2;
    if(tmp*2==wl)
        wl--;

    //Ahora wl es el menor valor impar entero proximo a wl, de esta diferencia viene la aprox.

    int m=round((3*box_num+4*box_num*wl+box_num*wl*wl-12*sigma*sigma)/(4+4*wl));   //Ver Kovesi...

    int i;

    for(i=0;i<m;i++){
        box_d[i]=wl;		//m filtros de tamaño wl
    }
    for(;i<box_num;i++){
        box_d[i]=wl+2;		//box_num-m de tamaño wl+2
    }

    sigma_r=sqrt((m*wl*wl+(box_num-m)*(wl+2.0)*(wl+2.0)-box_num)/12.0);   //Esta es la desviacion lograda..

    /* std::cout << "wl = " << wl << " m = " << m << " n = " << box_num << "s = " << sigma << " sr = " << sigma_r \
        << " %e = "<< fabs(sigma_r-sigma)/sigma*100 << std::endl;*/

    div=new Image<DetectorImgType>*[box_n];	  //Cada tamaño de filtro requiere un cierto conjunto de divisores (ver average()...)
    for(int i=0;i<box_n;i++){
        div[i]=new Image<DetectorImgType>(size);
        build_average(box_d[i],*div[i]); //lo calculo para cada tamaño
    }
}

// smooth() realiza el filtrado propiamente dicho, aplicando sucesivamente los filtros de caja

void iigauss::smooth(Image<DetectorImgType> &in, Image<DetectorImgType> &out){

    load(in);						//genera la primera imagen integral

    for(int i=0;i<box_n-1;i++){
        average(out,img_data,box_d[i],*div[i]);		//filtro de caja
        load(out);					//regenera la imagen integral
    }

    average(out,img_data,box_d[box_n-1],*div[box_n-1]);	//ultimo filtrado
}

// Esta funcion es igual a smooth() salvo que toma como dato la primera imagen integral,
// permite ahorrar redundancias en el calculo del espacio de escalas (ver sscale::build_v2()...)

void iigauss::iismooth(Image<DetectorImgType>& iimg,Image<DetectorImgType>& out){


    average(out,iimg,box_d[0],*div[0]);
    load(out);

    for(int i=1;i<box_n-1;i++){
        average(out,img_data,box_d[i],*div[i]);
        load(out);
    }

    average(out,img_data,box_d[box_n-1],*div[box_n-1]);
}
}
