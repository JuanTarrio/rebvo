#ifndef CUSTOMCAM_H
#define CUSTOMCAM_H

#include "pipeline.h"
#include "videocam.h"
namespace  rebvo{

class customCam : public VideoCam
{

    bool error=false;

public:
    struct CustomCamPipeBuffer{
        std::shared_ptr<Image <RGB24Pixel> > img;
        double timestamp;
    };

private:

    Pipeline <customCam::CustomCamPipeBuffer> &pipe;

public:
    customCam(Pipeline <customCam::CustomCamPipeBuffer> &cam_pipe, Size2D frame_size, const char *log_name);

    int WaitFrame(bool drop_frames=true) override;
    int LoadImage(const std::string &i_name);
    int GrabFrame(RGB24Pixel *data, double &tstamp, bool drop_frames=true) override;
    RGB24Pixel* GrabBuffer(double &tstamp,bool drop_frames=true) override;
    int ReleaseBuffer() override ;
    const bool & Error() override{return error;}
};

}
#endif // CUSTOMCAM_H
