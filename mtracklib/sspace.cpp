#include "sspace.h"
#include <iostream>

/*

  Esta clase implementa el espacio de escalas (Lindenberg 94,98 y otros...) a la Lowe et. al. (SIFT)
  Cada octaba subsamplea por dos la octaba anterior, para ahorrar tiempo de cálculo.
  Sin embargo, si se van a buscar features en s_n escalas, deben calcularse s_n+3 escalas por octaba
  ya que debe contarse con s_n+2 DoG para buscar maximos en espacio de escala

*/

sspace::sspace(double sigma0,
               double sigma1,
               const Size2D &size,
               int bf_num)
    :sigma0(sigma0),sigma1(sigma1),bf_n(bf_num),
      filters{iigauss(size,sigma0,bf_num),iigauss(size,sigma1,bf_num)},
      img{Image<DetectorImgType>(size),Image<DetectorImgType>(size)},dog(size),img_dx(size),img_dy(size)
{




}



void sspace::build(Image<DetectorImgType> &data){


    filters[0].smooth(data,img[0]);    //calcula los filtros usando la imagen integral
    filters[1].smooth(data,img[1]);    //calcula los filtros usando la imagen integral

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
