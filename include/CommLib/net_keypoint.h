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

    short n_kl;

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



int copy_net_keyline(edge_finder &from, net_keyline *to, int kl_size, double k_prof);
int copy_net_keyline_nextid(edge_finder &from, net_keyline *to, int kl_size);
}
#endif // NET_KEYPOINT_H
