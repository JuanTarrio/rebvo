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


#ifndef DISPLAYFRONTAL_H
#define DISPLAYFRONTAL_H

#include "VideoLib/video_io.h"
#include "CommLib/net_keypoint.h"
#include "rebvo/rebvo.h"
#include "UtilLib/configurator.h"
#include <iostream>
#include <vector>
namespace rebvo {


class visualizer
{
private:



    bool carga=true;

    std::string Host;
    int Port;

    Size2D ImageSize;

    double pFPS;
    double dFPS;



    bool ShowTrails=false;
    double depth_show_max;

    net_packet_hdr* net_hdr;

    double ZfX;

    bool nFrame=false;
    bool KillRemote=false;


    int TraySource;


    std::vector < TooN::Vector<3> > pos_tray;

    TooN::Matrix<3,3> Pose;

    net_keyline *net_kl;
    int net_kln=0;

    int ShowImg=1;


    double time2impact;
    double dist2impact;

    double KFix=1;


    int ViewNumber;
    int EdgeMapSaveNumber;
    int DF_ThreshMatchNum;
    int DF_ThreshRelRho;
    int DF_IterNum;

    int XLibView;

public:

    int       Run();
    static int OnPaint(XVideoContext *xvc, void *param);
    static int OnPaintDepth(XVideoContext *xvc, void *param);


    visualizer(Configurator &config);


    bool Init();
};
}

#endif // DISPLAYFRONTAL_H
