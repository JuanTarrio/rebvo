#ifndef VIDEODECODER_H
#define VIDEODECODER_H

#include "video_io.h"




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

#if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(54,25,0)
#define CodecID AVCodecID
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
    VideoDecoder(CodecID codec_id, int w, int h);
    ~VideoDecoder();
    bool DecodeFrame(u_char *coded_data,int cd_size,RGB24Pixel *data);
};

#endif // VIDEODECODER_H
