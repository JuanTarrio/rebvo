#ifndef IMAGE_UNDISTORT_H
#define IMAGE_UNDISTORT_H

#include "image.h"
#include "cam_model.h"
#include <exception>
#include <stdexcept>
namespace  rebvo{
//********************************************************************************
//image_undistort: Image undistorter using radial-tangential model and bilinear or NN interpolation
//**********************************************************************************

class image_undistort
{
    typedef struct {
        int num;        //number of valid points
        int inx[4];     //Index of neigboring points
        float w[4];     //Interpolation weights
        int iw[4];      //Integer interpolation weight
        int nn_inx;     //Index of nearest neigbor point
    }undistMapPoint;

    static constexpr int i_shift=16;
    static constexpr float i_mult=(1<<i_shift);

    cam_model cam;
    Image <undistMapPoint> umap;                //undistortion MAP


    //Interpolation funtions for float  and RGB24Pixel (24bit RGB) types

    inline float biInterp(Image<float> &in, const int &inx){
        int r=0;
        undistMapPoint &ump=umap[inx];
        for(int i=0;i<ump.num;i++){
            r+=ump.w[i]*(in[ump.inx[i]]);
        }
        return r;
    }

    inline RGB24Pixel biInterp(Image<RGB24Pixel> &in, const int &inx){

        int r=0,g=0,b=0 ;
        undistMapPoint &ump=umap[inx];
        for(int i=0;i<ump.num;i++){
            r+=ump.iw[i]*(in[ump.inx[i]].pix.r);
            g+=ump.iw[i]*(in[ump.inx[i]].pix.g);
            b+=ump.iw[i]*(in[ump.inx[i]].pix.b);
        }
        r>>=i_shift;
        g>>=i_shift;
        b>>=i_shift;
        return {(__u8)r,(__u8)g,(__u8)b};
    }

    inline float nnInterp(Image<float> &in, const int &inx){

        undistMapPoint &ump=umap[inx];
        if(ump.nn_inx<0)
            return 0;
        return in[ump.nn_inx];
    }

    inline RGB24Pixel nnInterp(Image<RGB24Pixel> &in, const int &inx){
        undistMapPoint &ump=umap[inx];
        if(ump.nn_inx<0)
            return {0,0,0};
        return in[ump.nn_inx];
    }


public:
    image_undistort(const cam_model&camera_model);
    template <bool use_bilin=true,typename DataType>
    void undistort(Image<DataType> &out, Image<DataType> &in);

};


template <bool use_bilin,typename DataType>
void image_undistort::undistort(Image<DataType> &out,Image<DataType> &in){
    if(out.Size().w!=in.Size().w || out.Size().h!=in.Size().h){
        throw std::length_error("image_undistort: images must be the same size!");
        return;
    }
    if(out.Size().w!=umap.Size().w || out.Size().h!=umap.Size().h){
        throw std::length_error("image_undistort: images must be the same size as the umap!");
        return;
    }

    for(int inx=0;inx<umap.bSize();inx++){
            if(use_bilin)
                out[inx]=biInterp(in,inx);
            else
                out[inx]=nnInterp(in,inx);
        }
}

}
#endif // IMAGE_UNDISTORT_H
