#ifndef IMAGE_H
#define IMAGE_H
#include <exception>
#include <stdexcept>
#include <math.h>
#include <string.h>

#include "VideoLib/common_types.h"
#include "VideoLib/video_io.h"
#include <iostream>
#include <cstring>

namespace  rebvo{

//****** Simple class to manage generic 2D images, without the mayor libraries *******//

//#define SAFE_IMAGE_ACCESS         //Flag to check for index errors

template <typename DataType>
class Image{
protected:
    Size2D size={0,0};    //Img size
    uint bsize=0;         //Data size
    DataType *data=nullptr;     //Plain data
    bool data_owned=false;
public:
    Image(){}

    Image(const Size2D &i_size)                      //Initilization from frame
        : size(i_size),bsize(i_size.w*i_size.h),
          data(new DataType[i_size.w*i_size.h]),
          data_owned(true){}

    Image(DataType* i_data,const Size2D &i_size)     //Inialization from data, not copy
        : size(i_size),bsize(i_size.w*i_size.h),
          data(i_data),
          data_owned(false){}

    Image(const Image &img)                      //Copy Initilization from other image
        : size(img.size),bsize(img.bsize),
          data(new DataType[size.w*size.h]),
          data_owned(true){
        std::memcpy(data,img.data,bsize*sizeof(DataType));
    }

    Image(Image &&img)                      //Move initilization from other image
        : size(img.size),bsize(img.bsize),
          data(img.data),
          data_owned(img.data_owned){

        img.data_owned=false;                     //prevent thte other image to delete data on destroy
    }


    ~Image(){
        if(data_owned && data)
            delete [] data;
    }

    void resize(const Size2D &sz){
        if(sz.h==size.h && sz.w==size.w)    //if size are equal, return
            return;
        size=sz;
        bsize=size.w*size.h;
        if(data)
            delete []data;
        data=new DataType[bsize];

    }



    //Image indexing functions
    DataType & operator [](const uint inx){
#ifdef SAFE_IMAGE_ACCESS
        if(inx>=bsize) throw std::out_of_range("Image index error");
#endif
        return data[inx];
    }

    const DataType & operator [](const uint inx) const {
#ifdef SAFE_IMAGE_ACCESS
        if(inx>=bsize) throw std::out_of_range("Image index error");
#endif
        return data[inx];
    }


    //Just get linear index
    uint GetIndex(const uint x,const uint y){
#ifdef SAFE_IMAGE_ACCESS
        if(x>=size.w || y>=size.h) throw std::out_of_range("Image index error");
#endif
        return y*size.w+x;
    }

    //Round real value position and check image boundaries
    inline int GetIndexRC(const float x,const float y){
        const int xi=round(x),yi=round(y);
        if(xi>=size.w || yi>=size.h || xi<0 || yi<0)
            return -1;
        return GetIndex(xi,yi);
    }

    DataType & operator ()(const uint x,const uint y){
        return data[GetIndex(x,y)];
    }


    bool isInxValid (const uint &x,const uint &y){
        return (x>=0&&y>=0&&x<size.w&&y<size.h);
    }

    //Data retreival
    DataType * Data(){return data;}
    const Size2D& Size() const {return size;}
    const uint& bSize()const {return bsize;}


    //data copy
    Image<DataType> & operator = (const Image<DataType> &img){

        resize(img.size);

        std::memcpy(data,img.data,bsize*sizeof(DataType));
        return *this;

    }

    //data copy
    Image<DataType> & operator = ( Image<DataType> &&img){
        size=img.size;
        bsize=img.bSize();
        if(data)
            delete [] data;
        data=img.data;
        data_owned=img.data_owned;
        img.data_owned=false;           //data is no longer owned by img
        return *this;
    }

    Image<DataType> & operator = (DataType *img){
        std::memcpy(data,img,bsize*sizeof(DataType));
        return *this;
    }

    void copyTo (DataType *img){
        std::memcpy(img,data,bsize*sizeof(DataType));
        return;
    }

    void copyFrom (DataType *img){
        std::memcpy(data,img,bsize*sizeof(DataType));
        return;
    }

    template <typename indexableType>
    inline void copyFromRotate180(indexableType from){
        for(int i=0,j=bsize-1;i<bsize;i++,j--)
           data[i]=from[j];
    }

    //Batch data set

    void Reset(DataType d){

        for(int i=0;i<bsize;i++)
            data[i]=d;

    }

    //Simple color convertion functions

    static void ConvertRGB2BW(Image<DataType> &to,Image<RGB24Pixel> &from){
        if(from.Size().h!=to.size.h || from.Size().w!=to.size.w || from.bSize()!=to.bsize)
            throw std::length_error("Error on convert image, must be the same size!");

        for(int i=0;i<to.bSize();i++)
            to[i]=from[i].pix.b+from[i].pix.g+from[i].pix.r;
    }

    static void ConvertBW2RGB(Image<RGB24Pixel> &to,Image<DataType> &from){
        if(from.Size().h!=to.size().h || from.Size().w!=to.size().w || from.bSize()!=to.bsize)
            throw std::length_error("Error on convert image, must be the same size!");

        for(int i=0;i<bSize();i++){
            to[i].pix.b=from[i]/3;
            to[i].pix.g=from[i]/3;
            to[i].pix.r=from[i]/3;
        }
    }


};

}

#endif // IMAGE_H
