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

#ifndef NET_KEYPOINT_H
#define NET_KEYPOINT_H

#include "mtracklib/edge_finder.h"
#include "mtracklib/nav_data_defs.h"

namespace  rebvo{

constexpr double NET_RHO_SCALING=(10000.0);
constexpr double NET_POS_SCALING=(64.0);

#pragma pack(push,1)


struct net_keyline{
    u_short qx;
    u_short qy;

    u_short rho;
    u_short s_rho;

    int n_kl;

    u_char m_num;

    union {
        struct {
            u_char x;
            u_char y;
        }flow;

        struct {
            unsigned a:10;
            unsigned m:6;
        }gradient;
    }extra;


};

struct net_packet_hdr{
    int data_size;
    int w;
    int h;
    int key_num;
    int jpeg_size;
    int encoder_type;
    int kline_num;
    int km_num;
    float k;
    float max_rho;

    //Nav data
    compressed_nav_pkg nav;
};

#pragma pack(pop)


struct upk_net_keypoint{

    float qx;
    float qy;
    float z;
    float s;
    float v;
    short int flags;
    char model;

};



int copy_net_keyline(edge_finder &from, edge_finder *from_pair, net_keyline *to, int kl_size, double k_prof);
int copy_net_keyline_nextid(edge_finder &from, net_keyline *to, int kl_size);
}
#endif // NET_KEYPOINT_H
