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

#ifndef VIDEODECODER_H
#define VIDEODECODER_H

#include "video_io.h"



namespace  rebvo{

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAVE_AV_CONFIG_H
#undef HAVE_AV_CONFIG_H
#endif

#ifndef UINT64_C
#define UINT64_C(c) (c ## ULL)
#endif

#include "libavcodec/avcodec.h"
#include "libavutil/mathematics.h"
#include "libavutil/samplefmt.h"

#ifdef __cplusplus
}
#endif

class VideoDecoder
{

    int status;
    AVCodec *codec;
    AVCodecContext *c;
    AVFrame *picture;
    AVPacket avpkt;
    int width;
    int height;
public:
    VideoDecoder(AVCodecID codec_id, int w, int h);
    ~VideoDecoder();
    bool DecodeFrame(u_char *coded_data,int cd_size,RGB24Pixel *data);
};

}
#endif // VIDEODECODER_H
