#include "CommLib/edgemap_com.h"
#include <TooN/so3.h>
#include "UtilLib/libcrc.h"

namespace rebvo{
edgemap_com::edgemap_com()
    :edge_map_id(0),kl_num(0),pkg_sr_inx(0),rho_scale(1)
{
    DK=1;
    DPos=TooN::Zeros;
    DPose=TooN::Identity;

    pkg_hdr.paket_size=0;
    oldness=0;

}


void edgemap_com::beginUncompress(){
    pair_inx=0;
}

bool edgemap_com::getPair(uncompressed_kl &kl0,uncompressed_kl &kl1){
    
    
    for(;pair_inx<kl_num-1 && kls[pair_inx].rho==0;pair_inx++);
    if(pair_inx>=kl_num-1)
        return false;
    
    kl0.rho=abs(kls[pair_inx].rho);
    kl0.rho/=(float)COMPRESSED_RHO_SCALING/rho_scale;
    kl0.x=kls[pair_inx].x/COMPRESSED_X_SCALING;
    kl0.y=kls[pair_inx].y/COMPRESSED_Y_SCALING;
    if(kls[pair_inx].s_rho>0)
        kl0.s_rho=kls[pair_inx].s_rho/COMPRESSED_SRHO_SCALING;
    else
        kl0.s_rho=CRHO_MIN;
    
    pair_inx++;
    
    if(kls[pair_inx].rho==0 || kls[pair_inx-1].rho<0){
        kl1=kl0;
        return true;
    }
    
    kl1.rho=kls[pair_inx].rho;
    kl1.rho/=(float)COMPRESSED_RHO_SCALING/rho_scale;
    kl1.x=kls[pair_inx].x/COMPRESSED_X_SCALING;
    kl1.y=kls[pair_inx].y/COMPRESSED_Y_SCALING;
    if(kls[pair_inx].s_rho>0)
        kl1.s_rho=kls[pair_inx].s_rho/COMPRESSED_SRHO_SCALING;
    else
        kl1.s_rho=CRHO_MIN;


    if(kl1.rho<0){
        kl1.rho=-kl1.rho;
        pair_inx++;
    }
    return true;
    
    
}


void edgemap_com::SaveKLS(char *filename){

    FILE *f=fopen(filename,"w");

    if(f==NULL){
        printf("\nEdgeMapComp: Error en SaveKLS\n");
        return;
    }

    for(int i=0;i<kl_num;i++){
        fprintf(f,"%d %d %d\n",kls[i].x,kls[i].y,kls[i].rho);
    }

    fclose(f);

}

uncompressed_kl edgemap_com::TransformPos(const uncompressed_kl & iP0, cam_model &cam){

    using namespace TooN;

    uncompressed_kl iP1;

    float x,y;
    cam.Img2Hom(x,y,iP0.x,iP0.y);
    Vector <3> P0=makeVector(x/cam.zfm,y/cam.zfm,1)/iP0.rho;

    Vector <3> P1=DPose*P0*em_pos.K+DPos;

    iP1.rho=1/P1[2];
    iP1.x=P1[0]*iP1.rho*cam.zfm;
    iP1.y=P1[1]*iP1.rho*cam.zfm;

    iP1.rho*=new_pos.K;

    cam.Hom2Img(iP1.x,iP1.y,iP1.x,iP1.y);
    iP1.s_rho=iP0.s_rho*new_pos.K;
    return iP1;
}


TooN::Vector<3> edgemap_com::UnProjectPos(const uncompressed_kl & iP0, cam_model &cam){

    using namespace TooN;
    float x,y;

    cam.Img2Hom(x,y,iP0.x,iP0.y);
    Vector <3> P0=makeVector(x/cam.zfm,y/cam.zfm,1)/iP0.rho;

    return  (DPose*P0*DK+DPos);  //K!!!!
}


void edgemap_com::UpdatePos(const em_compressed_nav_pkg & nav){

    using namespace TooN;
    new_pos=nav;

    SO3 <>NPose(makeVector(new_pos.w[0],new_pos.w[1],new_pos.w[2]));
    SO3 <>OPose(makeVector(em_pos.w[0],em_pos.w[1],em_pos.w[2]));
    const Vector <3> Vn=makeVector(new_pos.p[0],new_pos.p[1],new_pos.p[2]);
    const Vector <3> Vo=makeVector(em_pos.p[0],em_pos.p[1],em_pos.p[2]);
    const Matrix <3,3>& Rn=NPose.get_matrix();
    const Matrix <3,3>& Ro=OPose.get_matrix();


    DPose=Rn.T()*Ro;
    DPos=Rn.T()*(Vo-Vn);
    //DK=new_pos.K;
    DK=em_pos.K;

}

inline double kl_cos_angle(const KeyLine &k1,const KeyLine &k2){

    return (k1.u_m.x*k2.u_m.x+k1.u_m.y*k2.u_m.y);

}

int edgemap_com_sender::compress_edgemap(edge_finder & ef, double scale, int min_line_len, int max_line_len, double max_angle, cam_model &cam,int match_n_t,double rho_rel_max,float s_thresh){
    
    double max_cangle=cos(max_angle);

    PPoint3D<float> fit_list[max_line_len];
    int fit_ikl[max_line_len];

    
    for(int k_ink=0;k_ink<ef.KNum();k_ink++){
        kl_mask[k_ink]=false;
    }
    
    int line_len=0,c_inx=0,k_next=0,k_break=0,f_inx=0;
    
    for(int k_inx=0;k_inx<ef.KNum();k_inx++){
        
        for(;(kl_mask[k_inx]==true || ef[k_inx].p_id>=0) && k_inx<ef.KNum();k_inx++);
        if(k_inx>=ef.KNum())
            break;
        
        line_len=0;
        bool breaked=false;
        k_break=k_next=k_inx;

        fit_ikl[0]=k_inx;
        fit_list[0].x=ef[k_inx].c_p.x;
        fit_list[0].y=ef[k_inx].c_p.y;
        fit_list[0].rho=ef[k_inx].rho;
        fit_list[0].s_rho=ef[k_inx].m_num>=match_n_t?ef[k_inx].s_rho:1e10;

        f_inx=1;

        int krk_c=0;

        while(ef[k_next].n_id!=-1 && f_inx<max_line_len){
            line_len++;
            int k_prev=k_next;

        //    float z1=1/ef[k_next].rho;
        //    float sz1=z1-1/(ef[k_next].rho+ef[k_next].s_rho);

            k_next=ef[k_next].n_id;
            
            if(kl_mask[k_next])
                break;

        //    float z2=1/ef[k_next].rho;
         //   float sz2=z2-1/(ef[k_next].rho+ef[k_next].s_rho);
        //    if(fabs(z1-z2)>std::min(sz1,sz2))
        //        break;

            if(ef[k_prev].rho/ef[k_next].rho>rho_rel_max || ef[k_next].rho/ef[k_prev].rho>rho_rel_max)
                break;

            fit_ikl[f_inx]=k_next;
            fit_list[f_inx].x=ef[k_next].c_p.x;
            fit_list[f_inx].y=ef[k_next].c_p.y;
            fit_list[f_inx].rho=ef[k_next].rho;
            fit_list[f_inx++].s_rho=ef[k_next].m_num>=match_n_t?ef[k_next].s_rho:1e10;
            
            if(line_len>min_line_len && kl_cos_angle(ef[k_break],ef[k_next])<max_cangle){

                PPoint3D<float> fit_p0,fit_p1;

                double fit_sigma=LineFitting::RobustFit3DLine(fit_list,f_inx,cam,fit_p0,fit_p1,s_thresh);
                //Fit

                if(++krk_c>5)
                    krk_c=0;

                //

                kls[c_inx].x=util::clamp_uchar(fit_p0.x*COMPRESSED_X_SCALING);
                kls[c_inx].y=util::clamp_uchar(fit_p0.y*COMPRESSED_Y_SCALING);
                kls[c_inx].rho=util::clamp_short(std::max(fit_p0.rho*COMPRESSED_RHO_SCALING/scale,COMPRESSED_RHO_MIN));
                kls[c_inx].s_rho=util::clamp_uchar(fit_sigma*COMPRESSED_SRHO_SCALING);

                if((++c_inx)>=KEYLINE_MAX){
                    printf("\nComnpress error 1\n");
                    return kl_num=0;
                }
                
                kls[c_inx].x=util::clamp_uchar(fit_p1.x*COMPRESSED_X_SCALING);
                kls[c_inx].y=util::clamp_uchar(fit_p1.y*COMPRESSED_Y_SCALING);
                kls[c_inx].rho=util::clamp_short(std::max(fit_p1.rho*COMPRESSED_RHO_SCALING/scale,COMPRESSED_RHO_MIN));
                kls[c_inx].s_rho=util::clamp_uchar(fit_sigma*COMPRESSED_SRHO_SCALING);



                fit_ikl[0]=k_next;
                fit_list[0].x=ef[k_next].c_p.x;
                fit_list[0].y=ef[k_next].c_p.y;
                fit_list[0].rho=ef[k_next].rho;
                fit_list[0].s_rho=ef[k_next].m_num>=match_n_t?ef[k_next].s_rho:1e10;
                f_inx=1;

                k_break=k_next;
                line_len=0;
                breaked=true;
            }

            kl_mask[k_next]=true;
        }
        
        
        if(line_len>min_line_len){


            PPoint3D<float> fit_p0,fit_p1;

            double fit_sigma=LineFitting::RobustFit3DLine(fit_list,f_inx,cam,fit_p0,fit_p1,s_thresh);
            //Fit

            //

           // if(fabs(fit_p1.x-fit_p0.x)>200){
         //       printf("\nS\n");
          //  }

            kls[c_inx].x=util::clamp_uchar(fit_p0.x*COMPRESSED_X_SCALING);
            kls[c_inx].y=util::clamp_uchar(fit_p0.y*COMPRESSED_Y_SCALING);
            kls[c_inx].rho=util::clamp_short(std::max(fit_p0.rho*COMPRESSED_RHO_SCALING/scale,COMPRESSED_RHO_MIN));
            kls[c_inx].s_rho=util::clamp_uchar(fit_sigma*COMPRESSED_SRHO_SCALING);

            if((++c_inx)>=KEYLINE_MAX){
                printf("\nComnpress error 1\n");
                return kl_num=0;
            }

            kls[c_inx].x=util::clamp_uchar(fit_p1.x*COMPRESSED_X_SCALING);
            kls[c_inx].y=util::clamp_uchar(fit_p1.y*COMPRESSED_Y_SCALING);
            kls[c_inx].rho=-util::clamp_short(std::max(fit_p1.rho*COMPRESSED_RHO_SCALING/scale,COMPRESSED_RHO_MIN));
            kls[c_inx].s_rho=util::clamp_uchar(fit_sigma*COMPRESSED_SRHO_SCALING);


            c_inx++;
            f_inx=0;


        }else if(breaked){
            kls[c_inx].rho=-kls[c_inx].rho;
            c_inx++;
        }

        for(int i=0;i<f_inx;i++){
            kl_mask[fit_ikl[i]]=false;
        }
        
        
    }
    
    edge_map_id++;
    pkg_sr_inx=0;

    rho_scale=scale;
    
    return kl_num=c_inx;
    
}


int edgemap_com_sender::PreparePkg(const em_compressed_nav_pkg &nav, int max_kl_num) {
    
    pkg_hdr.edgemap_id=edge_map_id;

    pkg_hdr.nav=nav;
    
    int kl_to_send=std::min(max_kl_num,kl_num-pkg_sr_inx);
    
    pkg_hdr.kl_total=kl_num;
    pkg_hdr.kl_num=kl_to_send;
    pkg_hdr.kl_id=pkg_sr_inx;
    pkg_hdr.paket_size=kl_to_send*sizeof(compressed_kl)+sizeof(compressed_edgemap_hdr);

    pkg_hdr.rho_scale=rho_scale;
    
    pkg_ptr=(u_char*)&kls[pkg_sr_inx];
    pkg_sr_inx+=kl_to_send;


    pkg_hdr.crc_em=util::CRC16(pkg_ptr,kl_to_send*sizeof(compressed_kl));
    pkg_hdr.crc=util::CRC16((u_char*)&pkg_hdr,sizeof(compressed_edgemap_hdr)-sizeof(compressed_edgemap_hdr::crc));
    
    return kl_to_send;
    
}


edgemap_com_receiver::edgemap_com_receiver(uint recv_pkg_size)
    :edgemap_com()
{
    pkg_size=recv_pkg_size;
    if(recv_pkg_size>0)
        pkg_ptr=new u_char[recv_pkg_size];
    else
        pkg_ptr=NULL;
    
    for(int i=0;i<KEYLINE_MAX;i++)
        kls[i].rho=0;
}

edgemap_com_receiver::~edgemap_com_receiver(){
    if(pkg_ptr)
        delete pkg_ptr;
}

bool edgemap_com_receiver::CheckRecvPak(int brecv,bool &PkgReady,edgemap_com * copy_to){

    PkgReady=false;

    if(brecv < 0 || brecv!=pkg_hdr.paket_size || pkg_hdr.kl_total>KEYLINE_MAX){
        printf("\nEdgeMapRecv: Size ERROR: %d %d %d %x %x %x\n",brecv,pkg_hdr.paket_size,pkg_hdr.kl_total,(int)(((u_char*)&pkg_hdr)[sizeof(compressed_edgemap_hdr)-1]),(int)pkg_ptr[0],(int)pkg_ptr[1]);
        return false;
    }
    

    if(pkg_hdr.crc!=util::CRC16((u_char*)&pkg_hdr,sizeof(compressed_edgemap_hdr)-sizeof(compressed_edgemap_hdr::crc))){
        printf("\nEdgeMapRecv: CRC error, discarting...\n");
        return false;
    }


    if(pkg_hdr.edgemap_id>edge_map_id){
        
        if(copy_to)
            *copy_to=*this;

        PkgReady=true;
        
        for(int i=0;i<KEYLINE_MAX;i++)
            kls[i].rho=0;

        edge_map_id=pkg_hdr.edgemap_id;
        kl_num=pkg_hdr.kl_total;
        em_pos=pkg_hdr.nav;
        rho_scale=pkg_hdr.rho_scale;
        
    }else if(pkg_hdr.edgemap_id<edge_map_id-1){
        edge_map_id=pkg_hdr.edgemap_id;
        kl_num=pkg_hdr.kl_total;
    }else if(kl_num!=pkg_hdr.kl_total){
        printf("\nEdgeMapRecv: Consistency ERROR: %d!=%d\n",kl_num,pkg_hdr.kl_total);
    }
    
    if(pkg_hdr.kl_id+pkg_hdr.kl_num>pkg_hdr.kl_total){
        printf("\nEdgeMapRecv: Index ERROR I=%d K=%d T=%d\n", pkg_hdr.kl_id,pkg_hdr.kl_num,pkg_hdr.kl_total);
    }else{

        if(pkg_hdr.crc_em==util::CRC16(pkg_ptr,pkg_hdr.kl_num*sizeof(compressed_kl))){

            for(int i=0;i<pkg_hdr.kl_num;i++){
                kls[pkg_hdr.kl_id+i]=((compressed_kl*)pkg_ptr)[i];
            }
        }else{
            printf("\nEdgeMapRecv: CRC data error, discarting edges...\n");
        }
    }
    
    
    return true;
    
}

int edgemap_com_decoder::Decode(edgemap_com &em){

    seg_num=0;

    em.beginUncompress();
    while(seg_num<KEYLINE_MAX && em.getPair(seg_list[seg_num].k0,seg_list[seg_num].k1)){
        seg_list[seg_num++].visi_mask=true;
    }
    return seg_num;
}


int edgemap_com_decoder::HideVisible(const em_compressed_nav_pkg &pos, cam_model &cam){

    em_compressed_nav_pkg bk_pos=new_pos;

    UpdatePos(pos);

    int s_num=0;

    for(int i=0;i<seg_num;i++){

        if(!seg_list[i].visi_mask)
            continue;

        uncompressed_kl tkl0=TransformPos(seg_list[i].k0,cam);
        uncompressed_kl tkl1=TransformPos(seg_list[i].k1,cam);

        if((tkl0.x>=0 && tkl0.y >=0 && tkl0.x<cam.sz.w && tkl0.y<cam.sz.h && tkl0.rho>0) ||
                (tkl1.x>=0 && tkl1.y >=0 && tkl1.x<cam.sz.w && tkl1.y<cam.sz.h && tkl1.rho>0)){
            seg_list[i].visi_mask=false;
        }else{
            s_num++;
        }
    }


    UpdatePos(bk_pos);

    return s_num;


}

void edgemap_com_decoder::fillDepthMap(depth_filler &df,cam_model &cam,double v_thresh,double a_thresh)
{
    dfill=&df;


    using namespace TooN;
    beginUncompress();
    uncompressed_kl kl0,kl1;
    while(getPair(kl0,kl1)){

        if(kl0.s_rho*COMPRESSED_SRHO_SCALING>254.0 || kl1.s_rho*COMPRESSED_SRHO_SCALING>254.0 || kl0.s_rho+kl1.s_rho>kl0.rho+kl1.rho)
            continue;

        if(kl0.rho/kl0.s_rho<v_thresh || kl1.rho/kl1.s_rho<v_thresh )
            continue;


        TooN::Vector <3> p0=cam.unprojectImgCordVec(TooN::makeVector(kl0.x,kl0.y,kl0.rho));
        TooN::Vector <3> p1=cam.unprojectImgCordVec(TooN::makeVector(kl1.x,kl1.y,kl0.rho));
        double cangle= (fabs((p0-p1)*p0)/TooN::norm(p0)/TooN::norm(p0-p1));
        if(cangle>cos(a_thresh*M_PI/180.0))
            continue;

        Vector <2> t=makeVector(kl1.x-kl0.x,kl1.y-kl0.y);
        Vector <2> q0=makeVector(kl0.x,kl0.y);

        double nt=norm(t);
        t/=nt;

        for(int i=0;i<nt;i++){
            Vector <2> q=q0+t*i;

            double r1=(kl1.rho-kl0.rho)/nt;
            double rho=r1*i+kl0.rho;
            double s_rho=(kl0.s_rho+kl1.s_rho)/2;

            df_point &data=df.getImgPoint(q[0],q[1]);

            double i_rho=data.I_rho*data.rho;
            double kl_I_rho=1/(s_rho*s_rho);
            i_rho+=rho*kl_I_rho;
            data.I_rho+=kl_I_rho;
            double v_rho=data.I_rho>0?1.0/data.I_rho:1e20;
            data.rho=i_rho*v_rho;
            data.s_rho=sqrt(v_rho);
            data.fixed=true;


        }


    }

}


void edgemap_com_decoder::dfillerHideVisible(const em_compressed_nav_pkg &pos, cam_model &cam,depth_filler *stitch_to){

    using namespace TooN;

    if(!dfill)
        return;

    em_compressed_nav_pkg bk_pos=new_pos;

    UpdatePos(pos);


    for(int y=0;y<dfill->gridSize().h;y++)
        for(int x=0;x<dfill->gridSize().w;x++){

            df_point &data=dfill->data(x,y);

            if(!data.visibility)
                continue;


            Vector <3> P1=DPose*dfill->get3DPos(x,y)*em_pos.K+DPos;

            Vector <3> P1u=cam.projectImgCordVec(P1);

            if((P1u[0]>=0 && P1u[1] >=0 && P1u[0]<cam.sz.w && P1u[1]<cam.sz.h && P1u[2]>0)){
                data.visibility=false;

                if(stitch_to){
                    Vector <3> Pn=stitch_to->getImg3DPos(P1u[0],P1u[1])*new_pos.K;
                    Vector <3> P0=DPose.T()*(Pn-DPos)/em_pos.K;
                    data.rho=1.0/P0[2];
                    data.depth=P0[2];
                }

            }



    }


    UpdatePos(bk_pos);


}




double edgemap_com_decoder::GetLowerKL(const float g[3], cam_model &cam,double SigmaThresh,double ProfThresh){

    double max_dist=0,min_dist=1e20;
    using namespace TooN;

    double dists[seg_num*2];
    double sigmas[seg_num*2];
    int pnum=0;
    for(int i=0;i<seg_num;i++){


        {   //Cerro cocodrilo
            uncompressed_kl &kl=seg_list[i].k0;
            if(kl.rho/kl.s_rho>SigmaThresh){
                Vector <3> P=UnProjectPos(kl,cam);

                if(P[2]<ProfThresh){

                    dists[pnum]=(P[0]*g[0]+P[1]*g[1]+P[2]*g[2])/util::norm(g[0],g[1],g[2]);
                    sigmas[pnum]=kl.s_rho;
                    util::keep_min(min_dist,dists[pnum]);
                    util::keep_max(max_dist,dists[pnum]);
                    pnum++;
                }
            }
        }

        {
            uncompressed_kl & kl=seg_list[i].k1;
            if(kl.rho/kl.s_rho>SigmaThresh){
                Vector <3> P=UnProjectPos(kl,cam);


                if(P[2]<ProfThresh){
                    dists[pnum]=(P[0]*g[0]+P[1]*g[1]+P[2]*g[2])/util::norm(g[0],g[1],g[2]);
                    sigmas[pnum]=kl.s_rho;
                    util::keep_min(min_dist,dists[pnum]);
                    util::keep_max(max_dist,dists[pnum]);
                    pnum++;
                }

            }
        }

    }

    if(pnum==0)
        return -1;
    const int bin_num=100;
    int histo[bin_num];
    for(int i=0;i<0;i++)
        histo[i]=0;

    for(int i=0;i<pnum;i++){
        int inx=(dists[i]-min_dist)/(max_dist-min_dist)*(double)(bin_num-1);
        util::keep_max(inx,0);
        util::keep_min(inx,bin_num-1);
        histo[inx]++;
    }

    int icut=bin_num-1;
    int count=0;
    for(;icut>=0;icut--){
        count+=histo[icut];
        if(count>=pnum/10)
            break;
    }

    double dist_cut=((max_dist-min_dist)*(double)(icut-1)/(double)(bin_num-1)+min_dist);

    double dprom=0;
    double mass=0;
    for(int i=0;i<pnum;i++){
        if(dists[i]>=dist_cut){
            dprom+=dists[i]/sigmas[i];
            mass+=1/sigmas[i];;
        }
    }

    return dprom/mass;

}
}
