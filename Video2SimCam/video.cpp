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


#include <iostream> // for standard I/O
#include <string>   // for strings
#include <fstream>

#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)
#include <opencv2/highgui/highgui.hpp>  // Video write
#include <opencv2/highgui/highgui_c.h>
#include <stdio.h>
#include <iomanip>

#include <videocam.h>
#include <errno.h>

#include "image.h"

using namespace std;
using namespace cv;

static void help()
{
    cout
            << "------------------------------------------------------------------------------" << endl
            << "Convert video files to SimCam files sim_video and sim_time.txt"                 << endl
            << "./video2simcam video <fps>"                                                     << endl
            << "------------------------------------------------------------------------------" << endl
            << endl;
}

int main(int argc, char *argv[])
{

    if (argc <2 )
    {
        help();
        return -1;
    }



    ofstream tstamp_fd("sim_time.txt");

    FILE * sim_fd=fopen("sim_video", "w");




    if(sim_fd==NULL){
        cout << "\nError creating sim-video file\n";
        return false;
    }

    if(!tstamp_fd.is_open()){
        cout << "\nError creating sim-time file\n";
        return false;
    }

    VideoCapture inputVideo;                                        // Open the output

    if(!inputVideo.open(argv[1])){
        cout << "\nError opening video file " <<argv[1]<< "\n";
        return false;
    }



    VCFrameHdr hdr={0,0};
    Size2D size;
    size.w=inputVideo.get(CV_CAP_PROP_FRAME_WIDTH);
    size.h=inputVideo.get(CV_CAP_PROP_FRAME_HEIGHT);
    double fps;

    if(argc>2)
        fps=atoi(argv[2]);
    else
        fps=inputVideo.get(CV_CAP_PROP_FPS);

    Mat image;

    Image <RGB24Pixel> oframe(size);

    cout << "\nStaring video size ("<<size.w<<","<<size.h<<") FPS = "<<fps<<"\n";

    while(inputVideo.read(image)) //Show the image captured in the window and repeat
    {

        for(int y=0;y<size.h;y++)
            for(int x=0;x<size.w;x++){
                oframe(x,y).pix.r=image.at<Vec3b>(y,x)[0];
                oframe(x,y).pix.g=image.at<Vec3b>(y,x)[1];
                oframe(x,y).pix.b=image.at<Vec3b>(y,x)[2];
            }


        hdr.time=(double)hdr.n/fps;

        int r;
        if((r=fwrite(&hdr,sizeof(hdr),1,sim_fd))!=1){
            cout << "\nError writing "<<sizeof(hdr)<< "bytes of data to output video "<<r<<" bytes writen instead, Bye!\n";
            break;
        }

        if((r=fwrite(oframe.Data(),1,sizeof(RGB24Pixel)*size.h*size.w,sim_fd))!=sizeof(RGB24Pixel)*size.h*size.w){
            cout << "\nError writing "<<size.h*size.w*sizeof(RGB24Pixel)<< "bytes of data to output video "<<r<<" bytes writen instead, error: " << strerror(errno)<<"\n Bye!\n";
            break;
        }
        fflush(sim_fd);

        tstamp_fd << std::fixed << std::setprecision(10) <<  hdr.time << "\n";

        hdr.n++;
        cout << "Writed Frame "<< hdr.n << endl;

    }

    tstamp_fd.close();
    fclose(sim_fd);
    cout << "Finished writing "<< hdr.n <<" frames!" << endl;
    return 0;
}
