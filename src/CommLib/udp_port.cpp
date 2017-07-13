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


#include "CommLib/udp_port.h"
#include "UtilLib/util.h"
#include <iostream>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace  rebvo{
udp_port::udp_port(const char *remote_host, int port, bool bind_port, \
                   int max_pak_size, int pak_pipe_size, int max_fragment_size)
{

    if ((sock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1){
        printf("\nUDPPort: No puedo iniciar socket!\n");
        error=true;
        return;
    }



    memset((char *) &si_remote, 0, sizeof(si_remote));
    si_remote.sin_family = AF_INET;
    si_remote.sin_port = htons(port);
    si_remote.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind_port && bind(sock, (struct sockaddr*)&si_remote, sizeof(si_remote))==-1){
        printf("\nUDPPort: No puedo bloquear puerto!\n");
        error=true;
        return;
    }


    if (inet_aton(remote_host, &si_remote.sin_addr)==0) {
        printf("\nUDPPort: Error en la dirección remota!\n");
        error=true;
        return;
    }

    error=false;
    packet_tag=0;

    pp_size=pak_pipe_size;
    r_pack_n=new int[pp_size];
    r_pack_tag=new int[pp_size];
    r_pack_buf =new char *[pp_size];
    for(int i=0;i<pp_size;i++){
        r_pack_buf[i]=new char[max_pak_size];
        r_pack_n[i]=0;
        r_pack_tag[i]=-1;
    }

    max_p_size=max_pak_size;

    r_pack_inx=0;
    max_f_size=max_fragment_size;
    frag_buffer=new unsigned char [max_f_size];

    block=false;
    sleep_us=1e3;

}

udp_port::~udp_port(){
    if(error)
        return;

    close (sock);
    delete []frag_buffer;
    delete []r_pack_n;

    for(int i=0;i<pp_size;i++)
        delete [] r_pack_buf[i];

    delete [] r_pack_buf;
}

void udp_port::setBlock(bool b){

    if(block){

        int flags = fcntl(sock, F_GETFL, 0);
        fcntl(sock, F_SETFL, flags && ~O_NONBLOCK);
    }else{
        int flags = fcntl(sock, F_GETFL, 0);
        fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    }

    block=b;
}

int udp_port::RecvFragmented(unsigned char *buffer,int buf_size,double time_out)
{
    int br;
    bool recv_started=false;
    timeval t0,t1;
    while(1){

        br=recvfrom(sock,frag_buffer,max_f_size,0, NULL, NULL);

        if(br<=0){
            if(br<0 && errno  != EAGAIN){
                printf("\nUdpPort: RecvFrom error, %s\n",strerror(errno));
                return -1;
            }


            if(!recv_started)
                return 0;

            gettimeofday(&t1,NULL);
            if(util::dift(t1,t0)>time_out){
                printf("\nUdpPort: RecvFrom timeout\n");
                return -1;
            }

            continue;

        }
        gettimeofday(&t0,NULL);
        recv_started=true;

        UDPFragmentHeader * hdr=(UDPFragmentHeader * )frag_buffer;

      // printf("\nNum = %d Pos=%d FSize=%d PSize=%d Tag=%d br=%d\n",hdr->frag_num,hdr->frag_pos,hdr->frag_size,hdr->pack_size,hdr->tag,br);

        if(hdr->frag_size+(int)sizeof(UDPFragmentHeader) !=br){
            printf("\nUdpPort: Fragment size error %d!=%d\n",(int)(hdr->frag_size+sizeof(UDPFragmentHeader)),br);
            return -1;
        }

        int ipp;
        for(ipp=0;ipp<pp_size;ipp++){

            if(r_pack_tag[ipp]==hdr->tag){
                break;
            }

        }
        if(ipp==pp_size){
            r_pack_inx=(r_pack_inx+1)%pp_size;
            ipp=r_pack_inx;
            r_pack_n[ipp]=0;
            r_pack_tag[ipp]=hdr->tag;
        }

        if(hdr->frag_size+hdr->frag_pos>max_p_size){
            printf("\nUdpPort: Frag buffer error\n");
            return -1;
        }


        memcpy(&r_pack_buf[ipp][hdr->frag_pos],frag_buffer+sizeof(UDPFragmentHeader),hdr->frag_size);
        if((++(r_pack_n[ipp]))>=hdr->frag_num){
            if(hdr->pack_size>buf_size){
                printf("\nUdpPort: Recv buffer error\n");
                return -1;
            }
            memcpy(buffer,r_pack_buf[ipp],hdr->pack_size);
            return hdr->pack_size;
        }


    }
    return br;


}

bool udp_port::SendFragmented(unsigned char * data, int data_size,int fragment_size){

    packet_tag++;

    if(fragment_size+(int)sizeof(UDPFragmentHeader) > max_f_size){
        max_f_size=fragment_size+sizeof(UDPFragmentHeader);
        delete [] frag_buffer;
        frag_buffer=new unsigned char [max_f_size];
    }

    UDPFragmentHeader * hdr=(UDPFragmentHeader * )frag_buffer;

    int frag_num=(data_size+fragment_size-1)/fragment_size;
    int b_remain=data_size;
    int pos=0;
    while(b_remain>0){

        int b_to_send=std::min(fragment_size,b_remain);

        memcpy(frag_buffer+sizeof(UDPFragmentHeader),data,b_to_send);
        data+=b_to_send;

        hdr->frag_pos=pos;
        hdr->frag_num=frag_num;
        hdr->frag_size=b_to_send;
        hdr->pack_size=data_size;

        hdr->tag=packet_tag;

        if(!SendPacket(frag_buffer,b_to_send+sizeof(UDPFragmentHeader))){
            return false;
        }

        b_remain-=b_to_send;
        pos+=b_to_send;


       // printf("\nNum = %d Pos=%d FSize=%d PSize=%d Tag=%d \n",hdr->frag_num,hdr->frag_pos,hdr->frag_size,hdr->pack_size,hdr->tag);



    }


    return true;
}



bool udp_port::SendPacket(unsigned char * data, int data_size){

    int r;
    while((r=sendto(sock, data,data_size, !block?MSG_DONTWAIT:0, (struct sockaddr*)&si_remote,sizeof(si_remote)))!=data_size){
        if(!block)
            return false;
        else
            usleep(sleep_us);
    }


    return true;
}
}
