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
