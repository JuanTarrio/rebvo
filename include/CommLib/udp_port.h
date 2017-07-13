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


#ifndef UDP_PORT_H
#define UDP_PORT_H
#include <stdlib.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <asm/types.h>

namespace  rebvo{

#pragma pack(push,1)
struct UDPFragmentHeader{
    __u32 tag;
    __u32 frag_pos;
    __u32 frag_num;
    __u32 frag_size;
    __u32 pack_size;
};

#pragma pack(pop)

class udp_port
{
    int sock;
    sockaddr_in si_remote;

    bool error;
    int packet_tag;

    char ** r_pack_buf;
    int *r_pack_n;
    int *r_pack_tag;
    int pp_size;
    int r_pack_inx;

    int max_f_size;
    unsigned char *frag_buffer;
    int max_p_size;

    bool block;
    int sleep_us;

public:
    udp_port(const char *remote_host, int port, bool bind_port=false\
            , int max_pak_size=1e6, int pak_pipe_size=10, int max_fragment_size =65535);
    ~udp_port();

    const bool & Error(){return error;}

    bool SendFragmented(unsigned char *data, int data_size, int fragment_size);
    int  RecvFragmented(unsigned char *buffer, int buf_size, double time_out=2);

    bool SendPacket(unsigned char * data, int data_size);

    void setBlock(bool b);
    void setSleep(float secs){sleep_us=secs*1.0e6;}
};

}
#endif // UDP_PORT_H
