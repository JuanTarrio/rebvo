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

#ifndef EDGEMAP_COM_H
#define EDGEMAP_COM_H

#include <mtracklib/edge_finder.h>
#include <UtilLib/cam_model.h>

#include "UtilLib/linefitting.h"
#include "visualizer/depth_filler.h"

namespace rebvo{

#define CRHO_MAX     20
#define CRHO_MIN     1e-3


#define COMPRESSED_SRHO_SCALING ((float)(255.0/CRHO_MAX))
#define COMPRESSED_RHO_SCALING ((float)(32767.0/CRHO_MAX))
#define COMPRESSED_RHO_MIN (COMPRESSED_RHO_SCALING*CRHO_MIN)
#define COMPRESSED_X_SCALING (0.5)
#define COMPRESSED_Y_SCALING (1)

#pragma pack(push,1)
struct compressed_kl{
    u_char x;
    u_char y;
    short rho;
    u_char s_rho;
};


struct em_compressed_nav_pkg{
    float p[3];
    float w[3];
    float g[3];
    float ps[3];
    float K;
    float ref_err[4];

    em_compressed_nav_pkg(){
        for(int i=0;i<3;i++){
            p[i]=0;
            w[i]=0;
            g[i]=0;
            ps[i]=0;
        }
        for(int i=0;i<4;i++)
            ref_err[i]=0;
        K=1;
    }

    const em_compressed_nav_pkg & operator = (const em_compressed_nav_pkg &f){
        for(int i=0;i<3;i++){
            this->p[i]=f.p[i];
            this->w[i]=f.w[i];
            this->g[i]=f.g[i];
            this->ps[i]=f.ps[i];
        }
        for(int i=0;i<4;i++)
            this->ref_err[i]=f.ref_err[i];
        K=f.K;
        return *this;
    }
};

struct compressed_edgemap_hdr{
    ushort paket_size;
    ushort edgemap_id;
    float rho_scale;
    uint kl_id;
    uint kl_num;
    uint kl_total;
    em_compressed_nav_pkg nav;
    ushort crc_em;
    ushort crc;

};


#pragma pack(pop)


typedef PPoint3D<float> uncompressed_kl;

struct uncompressed_segment{
    uncompressed_kl k0;
    uncompressed_kl k1;
    bool visi_mask;
};

class edgemap_com
{
protected:
    compressed_kl kls[KEYLINE_MAX];
    bool kl_mask[KEYLINE_MAX];
    ushort edge_map_id;
    int kl_num;

    compressed_edgemap_hdr pkg_hdr;
    int pkg_sr_inx;
    u_char *pkg_ptr;

    int pair_inx;

    em_compressed_nav_pkg em_pos;
    em_compressed_nav_pkg new_pos;

    TooN::Matrix <3,3> DPose;
    TooN::Vector <3> DPos;
    double DK;

    int oldness;


    float rho_scale;


public:
    edgemap_com();

    TooN::Matrix <3,3> GetDPose(){return DPose;}
    TooN::Vector <3> GetDPos(){return DPos;}
    double GetDK(){return DK;}

    TooN::Vector <3> GetGEst(){return TooN::makeVector(new_pos.g[0],new_pos.g[1],new_pos.g[2]);}


    void beginUncompress();
    bool getPair(uncompressed_kl &kl0, uncompressed_kl &kl1);

    u_char* GetPkgStr(){return pkg_ptr;}
    u_char* GetHdrStr(){return (u_char*)&pkg_hdr;}
    uint GetHdrSize(){return sizeof(compressed_edgemap_hdr);}

    ushort GetEMId(){return edge_map_id;}

    uint Size(){return kl_num;}

    compressed_kl & operator [] (int inx){return kls[inx];}


    edgemap_com & operator = (const edgemap_com &arg){
        pkg_ptr=NULL;

        edge_map_id=arg.edge_map_id;
        kl_num=arg.kl_num;
        for(int i=0;i<kl_num;i++)
            kls[i]=arg.kls[i];

        em_pos=arg.em_pos;
        oldness=arg.oldness;

        rho_scale=arg.rho_scale;
        return *this;
    }

    void SaveKLS(char *filename);

    uncompressed_kl TransformPos(const uncompressed_kl & iP0, cam_model &cam);
    TooN::Vector<3> UnProjectPos(const uncompressed_kl & iP0, cam_model &cam);


    const em_compressed_nav_pkg &GetNavData(){return pkg_hdr.nav;}

    void UpdatePos(const em_compressed_nav_pkg &nav);

    int GetOldness(){return oldness;}
    int IncOldness(){return ++oldness;}

    const em_compressed_nav_pkg& GetViewPos(){return em_pos;}
};


class edgemap_com_sender: public edgemap_com
{


public:

    edgemap_com_sender():edgemap_com(){}

    int compress_edgemap(edge_finder & ef, double scale, int min_line_len, int max_line_len, double max_angle, cam_model &cam, int match_n_t=0,double rho_rel_max=1e10,float s_thresh=1e10);

    int PreparePkg(const em_compressed_nav_pkg &nav, int max_kl_num);
    uint GetPkgSize(){return pkg_hdr.paket_size-sizeof(compressed_edgemap_hdr);}

};

class edgemap_com_receiver: public edgemap_com
{

    int pkg_size;
public:
    edgemap_com_receiver(uint recv_pkg_size=0);
    ~edgemap_com_receiver();

    bool CheckRecvPak(int brecv,bool &PkgReady,edgemap_com * copy_to=NULL);

    uint GetPkgSize(){return pkg_size;}

};


class edgemap_com_decoder: public edgemap_com
{

    uncompressed_segment seg_list[KEYLINE_MAX];
    int seg_num;


public:
    depth_filler *dfill=nullptr;

    edgemap_com_decoder():edgemap_com(),seg_num(0){}

    int SegSize(){return seg_num;}

    int Decode(){return Decode(*this);}
    int Decode(edgemap_com & em);

    uncompressed_segment & operator [](int inx){return seg_list[inx];}

    int HideVisible(const em_compressed_nav_pkg &pos, cam_model &cam);

    void fillDepthMap(depth_filler &df, cam_model &cam, double v_thresh, double a_thresh);

    double GetLowerKL(const float g[], cam_model &cam, double SigmaThresh,double ProfThresh);

    void dfillerHideVisible(const em_compressed_nav_pkg &pos, cam_model &cam, depth_filler *stitch_to);
};
}

#endif // EDGEMAP_COM_H

