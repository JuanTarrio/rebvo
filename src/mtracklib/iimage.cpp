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



#include "mtracklib/iimage.h"
namespace  rebvo{
/*

  iimage: clase basica para el manejo de imagenes integrales

*/


//Inicializa la clase con el tamaño de la imagen, el buffer
// es opcional

iimage::iimage(const Size2D &size,Image<DetectorImgType> *i_load)
    :img_data(size),w(size.w),h(size.h)
{
    if(i_load)
        load(*i_load);
}

iimage::~iimage(){
}

//Genera la imagen integral a partir de una imagen normal
//Se utiliza como entrada un buffer de 32 bits, ya que
//permite aumentar la presicion de los filtros resultantes
//sin costo computacional

void iimage::load(Image<DetectorImgType> &l_data){


    for(int y=0;y<h;y++){
        img_data(0,y)=l_data(0,y);
        for(int x=1;x<w;x++){
            img_data(x,y)=img_data(x-1,y)+l_data(x,y);    //suma sobre las filas
        }
    }

    for(int x=0;x<w;x++){
        for(int y=1;y<h;y++){
            img_data(x,y)+=img_data(x,y-1);		    //suma sobre las cols
        }
    }

    //img(x,y)=sum_0^y sum_0^x data(i,j)

}

//Reliza un filtrado tipo caja usando la imagen integral:
//fil(x,y) = sum_(y-d/2)^(y+d/2) sum_(x-d/2)^(x+d/2) data(i,j) / (d*d)=
//
//	     img(y+d/2,x+d/2)-img(y-d/2,x+d/2)-img(y+d/2,x-d/2)+img(y-d/2,x-d/2)
//	   = -------------------------------------------------------------------
//					    d*d
//
//La complejidad de la funcion proviene del manejo de bordes, donde debido a la
//limitacion en la cantidad de pixels disponibles cambia el divisor en la ecuacion.
//Este divisor depende del tamaño del filtro y de la imagen y no de los valores en sí,
//por esto se precalcula en la inicializacion utilizando la funcion build_average(),
//de esta forma se ahorra tiempo de computacion durante el procesamiento de los frames

void iimage::average(Image<DetectorImgType> &buf, Image<DetectorImgType> &img, int d, Image<DetectorImgType> &div)
{


    auto & w=div.Size().w;
    auto & h=div.Size().h;

    int x,y,d2=d/2;
    float a=1.0/(d*d);
    for(y=0;y<d2+1;y++){
        for(x=0;x<d2+1;x++){
            buf(x,y)=img(x+d2,y+d2)*div(x,y);	//como x-d2 e y-d2 es negativo, se asume 0
        }
        for(;x<w-d2;x++){
            buf(x,y)=(img(x+d2,y+d2)-img(x-d2-1,y+d2))*div(x,y); //idem solo y-d2
        }
        for(;x<w;x++){
            buf(x,y)=(img(w-1,y+d2)-img(x-d2-1,y+d2))*div(x,y); //como x+d2>w se toma el valor en w-1
        }
    }
    for(;y<h-d2;y++){
        for(x=0;x<d2+1;x++){
            buf(x,y)=(img(x+d2,y+d2)-img(x+d2,y-d2-1))*div(x,y); //misma idea ...
        }
        for(;x<w-d2;x++){
            buf(x,y)=(img(x+d2,y+d2)-img(x-d2-1,y+d2)-img(x+d2,y-d2-1)+img(x-d2-1,y-d2-1))*a;//div(x,y);
        }
        for(;x<w;x++){
            buf(x,y)=(img(w-1,y+d2)-img(x-d2-1,y+d2)-img(w-1,y-d2-1)+img(x-d2-1,y-d2-1))*div(x,y);
        }
    }
    for(;y<h;y++){
        for(x=0;x<d2+1;x++){
            buf(x,y)=(img(x+d2,h-1)-img(x+d2,y-d2-1))*div(x,y);
        }
        for(;x<w-d2;x++){
            buf(x,y)=(img(x+d2,h-1)-img(x+d2,y-d2-1)-img(x-d2-1,h-1)+img(x-d2-1,y-d2-1))*div(x,y);
        }
        for(;x<w;x++){
            buf(x,y)=(img(w-1,h-1)-img(w-1,y-d2-1)-img(x-d2-1,h-1)+img(x-d2-1,y-d2-1))*div(x,y);
        }
    }
}

//Esta funcion calcula el divisor en cada pixel (cantidad de pixels del filtro caja)
//teniendo en cuenta las dimensiones finitas de la imagen (bordes)
//(Ver average()...), el calculo de estos divisores es "tricky"...

void iimage::build_average(int d,Image<DetectorImgType> &div)
{

    auto & w=div.Size().w;
    auto & h=div.Size().h;

    int x,y,d2=d/2,a=d*d;
    for(y=0;y<d2+1;y++){
	for(x=0;x<d2+1;x++){
        div(x,y)=(x+d2+1)*(y+d2+1);
	}
	for(;x<w-d2;x++){
        div(x,y)=d*(y+d2+1);
	}
	for(;x<w;x++){
        div(x,y)=(w-x+d2)*(y+d2+1);
	}
    }
    for(;y<h-d2;y++){
	for(x=0;x<d2+1;x++){
        div(x,y)=(x+d2+1)*d;
	}
	for(;x<w-d2;x++){
        div(x,y)=a;   //Esta es la zona central de la imagen (donde el area de la caja es d*d)
	}
	for(;x<w;x++){
        div(x,y)=(w-x+d2)*d;
	}
    }
    for(;y<h;y++){
	for(x=0;x<d2+1;x++){
        div(x,y)=(h-y+d2)*(x+d2+1);
	}
	for(;x<w-d2;x++){
        div(x,y)=(h-y+d2)*d;
	}
	for(;x<w;x++){
        div(x,y)=(h-y+d2)*(w-x+d2);
	}
    }

    for(int y=0;y<h;y++)
	for(int x=0;x<w;x++)
        div(x,y)=1.0/div(x,y);

}


}
