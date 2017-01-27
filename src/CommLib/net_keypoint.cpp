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


#include "CommLib/net_keypoint.h"
namespace  rebvo{


int copy_net_keyline(edge_finder &from,edge_finder *from_pair, net_keyline *to, int kl_size,double k_prof){

    uint j=0;
    for(auto &kl:from){

        if(j>=kl_size)
            break;


        to[j].qx=round(kl.c_p.x);
        to[j].qy=round(kl.c_p.y);

        to[j].rho=std::max(util::clamp_ushort(NET_RHO_SCALING*kl.rho/k_prof),(u_short)1);
        to[j].s_rho=std::max(util::clamp_ushort(NET_RHO_SCALING*kl.s_rho/k_prof),(u_short)1);

        if(from_pair){
            if(kl.stereo_m_id>=0 && fabs(round(-kl.c_p.x+(*from_pair)[kl.stereo_m_id].c_p.x))<127 && fabs(round(-kl.c_p.y+(*from_pair)[kl.stereo_m_id].c_p.y))<127){

                to[j].extra.flow.x=util::clamp_uchar(round((-kl.c_p.x+(*from_pair)[kl.stereo_m_id].c_p.x)+127.0));
                to[j].extra.flow.y=util::clamp_uchar(round((-kl.c_p.y+(*from_pair)[kl.stereo_m_id].c_p.y)+127.0));


        //        to[j].rho=std::max(util::clamp_ushort(NET_RHO_SCALING*kl.stereo_rho/k_prof),(u_short)1);
        //        to[j].s_rho=std::max(util::clamp_ushort(NET_RHO_SCALING*kl.stereo_s_rho/k_prof),(u_short)1);

            }else{

                to[j].extra.flow.x=127;
                to[j].extra.flow.y=127;
            }
        }else{
            to[j].extra.flow.x=util::clamp_uchar(round((kl.p_m.x-kl.p_m_0.x)*10+127.0));
            to[j].extra.flow.y=util::clamp_uchar(round((kl.p_m.y-kl.p_m_0.y)*10+127.0));
        }

        to[j].n_kl=-1;

        to[j].m_num=util::clamp_uchar(kl.m_num);

        kl.net_id=j;

        j++;

    }
    return j;

}



int copy_net_keyline_nextid(edge_finder &from, net_keyline *to, int kl_size){


    //return 0;
    int j=0;
    for(auto &kl:from){

        if(j>=kl_size)
            break;


        j=kl.net_id;

        if(j<0 && j>kl_size){
            printf("\nCNKLN WTF0 %d %ld\n",j,&kl-from.begin());
        }

        if(kl.n_id>=0){
            to[j].n_kl=from[kl.n_id].net_id;

            if(to[j].n_kl<0){
                printf("\nCNKLN WTF1 %d %d %f %f\n",kl.n_id,to[j].n_kl,kl.c_p.x,kl.c_p.y);
            }
        }


    }
    return j;

}

}
