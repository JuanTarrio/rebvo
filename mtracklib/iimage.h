#ifndef IIMAGE_H
#define IIMAGE_H

#include "math.h"

#include "util.h"
#include "image.h"
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
