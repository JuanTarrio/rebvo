#include "customcam.h"
namespace  rebvo{
customCam::customCam(Pipeline<CustomCamPipeBuffer> &cam_pipe, Size2D frame_size, const char *log_name)
    :VideoCam(log_name,frame_size),pipe(cam_pipe)
{
}



int customCam::WaitFrame(bool drop_frames){



    return 0;

}

int customCam::GrabFrame(RGB24Pixel *data, double &tstamp, bool drop_frames){


    CustomCamPipeBuffer &pbuf=pipe.RequestBuffer(1);

    tstamp=pbuf.timestamp;
    paknum++;

    (*pbuf.img).copyTo(data);

    pipe.ReleaseBuffer(1);
    return 0;
}

RGB24Pixel* customCam::GrabBuffer(double &tstamp, bool drop_frames){


    CustomCamPipeBuffer &pbuf=pipe.RequestBuffer(1);



    tstamp=pbuf.timestamp;
    paknum++;
    return (*pbuf.img).Data();
}

int customCam::ReleaseBuffer(){
    pipe.ReleaseBuffer(1);
    return 0;
}
}
