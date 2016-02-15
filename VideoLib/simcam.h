#ifndef SIMCAM_H
#define SIMCAM_H

#include "videocam.h"

class simcam : public VideoCam
{
    bool error;

    FILE *fd;

    RGB24Pixel* buffer;

    VCFrameHdr hdr;

    bool frm_pending;


public:
    simcam(const char *sim_name,Size2D frame_size,const char *log_name=NULL);
    ~simcam();
    int WaitFrame(bool drop_frames=true) override;
    int GrabFrame(RGB24Pixel *data, double &tstamp, bool drop_frames=true) override;
    RGB24Pixel* GrabBuffer(double &tstamp,bool drop_frames=true) override;
    int ReleaseBuffer() override ;
    const bool & Error() override{return error;}
};

#endif // SIMCAM_H
