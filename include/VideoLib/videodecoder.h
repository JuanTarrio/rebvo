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
