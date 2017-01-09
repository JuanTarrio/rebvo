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
