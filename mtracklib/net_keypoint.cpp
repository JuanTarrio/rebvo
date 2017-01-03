
#include "net_keypoint.h"
namespace  rebvo{


int copy_net_keyline(edge_finder &from, net_keyline *to, int kl_size,double k_prof){

    uint j=0;
    for(auto &kl:from){

        if(j>=kl_size)
            break;


        to[j].qx=round(kl.c_p.x);
        to[j].qy=round(kl.c_p.y);

        to[j].extra.flow.x=util::clamp_uchar(round((kl.p_m.x-kl.p_m_0.x)*10+127.0));
        to[j].extra.flow.y=util::clamp_uchar(round((kl.p_m.y-kl.p_m_0.y)*10+127.0));

        to[j].rho=std::max(util::clamp_ushort(NET_RHO_SCALING*kl.rho/k_prof),(u_short)1);
        to[j].s_rho=std::max(util::clamp_ushort(NET_RHO_SCALING*kl.s_rho/k_prof),(u_short)1);

        to[j].n_kl=-1;

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
