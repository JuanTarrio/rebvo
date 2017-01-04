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

 
 #include "VideoLib/videodecoder.h"

namespace  rebvo{
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(55,28,1)
#define av_frame_alloc  avcodec_alloc_frame
#endif


int LibAvInitStatus=0;

VideoDecoder::VideoDecoder(AVCodecID codec_id,int w,int h)
{
    status = 0;

    if(LibAvInitStatus==0){

        printf("\nVideoDecoder: iniciando LibAV!\n");

        /* must be called before using avcodec lib */
        //avcodec_init();

        /* register all the codecs */
        avcodec_register_all();
        LibAvInitStatus=1;

    }


    av_init_packet(&avpkt);


    /* find the mpeg1 video decoder */
    codec = avcodec_find_decoder(codec_id);    //
    if (!codec) {
        printf("\nVideoDecoder: codec not found!\n");
        return;
    }

    c = avcodec_alloc_context3(codec);
    picture= av_frame_alloc();


/*
    if(codec->capabilities&CODEC_CAP_TRUNCATED)
        c->flags|= CODEC_FLAG_TRUNCATED;  we do not send complete frames

       For some codecs, such as msmpeg4 and mpeg4, width and height
       MUST be initialized there because this information is not
       available in the bitstream. */

    c->width=w;
    c->height=h;
    width=w;
    height=h;

    if (avcodec_open2(c, codec,NULL) < 0) {
        printf("\nVideoDecoder: no puedo abrir el codec!\n");
        return;
    }

    printf("\nVideoDecoder: iniciado (%dx%d) OK!\n",width,height);

}

VideoDecoder::~VideoDecoder(){

    avcodec_close(c);
    //av_free(c);
    //av_free(picture);
}

inline float clamp(float s){

    return s<0?0:s>255?255:s;

}

bool VideoDecoder::DecodeFrame(u_char *coded_data,int cd_size,RGB24Pixel *data)
{

    int got_picture,len;

    avpkt.size=cd_size;
    avpkt.data = coded_data;

    len = avcodec_decode_video2(c, picture, &got_picture, &avpkt);
    if (len < 0) {
        printf("\nVideoDecoder: Error while decoding frame!\n");
        return false;
    }

    if (got_picture) {

       // printf ("\n%d\n",picture->format,picture);

        float Y,Cb,Cr;
        for(int y=0,i=0,j=0;y<height;y++,j+=picture->linesize[0]-width)
            for(int x=0;x<width;x++,i++,j++){

                int k=(y/2)*picture->linesize[1]+(x/2);
                Y=picture->data[0][j];
                Cb=picture->data[1][k];
                Cr=picture->data[2][k];
                data[i].pix.r=clamp(Y+1.402*(Cr-128));
                data[i].pix.g=clamp(Y-0.344*(Cb-128)-0.714*(Cr-128));
                data[i].pix.b=clamp(Y+1.772*(Cb-128));

            }


    }

    return false;
}

}
