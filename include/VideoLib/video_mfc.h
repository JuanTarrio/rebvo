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

#ifndef VIDEO_MFC_H
#define VIDEO_MFC_H


#include "video_encoder.h"
namespace  rebvo{


enum MFCState{MFC_ST_ERROR,MFC_ST_OPEN,MFC_ST_OK,MFC_ST_STREAM_ON};

enum MFC_io_type { IO_NONE, IO_FUNC, IO_MMAP, IO_USERPTR };
enum MFC_io_dir { DIR_IN = 0, DIR_OUT = 1};


#define MFC_MAX_STREAM_SIZE (2*1024*1024)

class EncoderMFC: public VideoEncoder{

private:
    int fd;
    MFCState state;

    int width;
    int height;
    int fps;
    int bitrate;


    int BytesPerLine;

    int NBufs[2];
    int NPPBuf[2];

    int *PlanesLen[2];

    void ***BufAddr[2];


    int ReqBufs(MFC_io_dir dir, int nbuf);
    int InitBuffers(MFC_io_dir dir,int nbuf);

    int BufInx[2];

public:
    EncoderMFC(const char *name);
    ~EncoderMFC();

    int SetCodec(int codec, int stream_b_size);

    int SetFormat(int width, int height);

    int SetRate(float rate);

    int SetControl(int id, int value);

    int SetBitRate(int bitrate);

    int StreamOn(bool SetOn);

    int Initialize(int width, int height,int codec,float rate,int bitrate,int in_bufs,int out_bufs,int stream_b_size);

    int PopFrame(char *stream_buf,int b_size);
    int PushFrame(RGB24Pixel *data);
    void Convert_RGB_2_NV12(union RGB24Pixel *data,char **b_plane, int *s_planes);

    int SendStop();

    int GetEncoderType(){return VIDEO_ENCODER_TYPE_MFC;}

    void Autoconfig();
};

}
#endif // VIDEO_MFC_H
