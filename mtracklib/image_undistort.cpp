#include "image_undistort.h"


image_undistort::image_undistort(const cam_model&camera_model)
    :cam(camera_model),umap(cam.sz)
{

    for(int x=0;x<cam.sz.w;x++)
        for(int y=0;y<cam.sz.h;y++){

            undistMapPoint & ump=umap(x,y);                     //For each point in the map

            Point2D<float> qd=cam.Img2Hom(Point2D<float>(x,y));
            cam.distortHom2Hom(qd);                             //Distort-it

            Point2D<float> id=cam.Hom2Img(qd);

            ump.nn_inx=umap.GetIndexRC(id.x,id.y);

            Point2D<float> p00(floor(id.x)  ,floor(id.y)  );
            Point2D<float> p01(floor(id.x)+1,floor(id.y)  );
            Point2D<float> p10(floor(id.x)  ,floor(id.y)+1);
            Point2D<float> p11(floor(id.x)+1,floor(id.y)+1);

            ump.num=0;

            if(umap.isInxValid(p00.x,p00.y)){
                ump.w[ump.num]=(p11.x-id.x)*(p11.y-id.y);
                ump.inx[ump.num]=umap.GetIndexRC(p00.x,p00.y);
                ump.num++;
            }

            if(umap.isInxValid(p01.x,p01.y)){
                ump.w[ump.num]=(id.x-p00.x)*(p11.y-id.y);
                ump.inx[ump.num]=umap.GetIndexRC(p01.x,p01.y);
                ump.num++;
            }

            if(umap.isInxValid(p10.x,p10.y)){
                ump.w[ump.num]=(p11.x-id.x)*(id.y-p00.y);
                ump.inx[ump.num]=umap.GetIndexRC(p10.x,p10.y);
                ump.num++;
            }

            if(umap.isInxValid(p11.x,p11.y)){
                ump.w[ump.num]=(id.x-p00.x)*(id.y-p00.y);
                ump.inx[ump.num]=umap.GetIndexRC(p11.x,p11.y);
                ump.num++;
            }


            if(ump.num>0){
                float sum_w=0;

                for(int i=0;i<ump.num;i++){
                    sum_w+=ump.w[i];
                }

                for(int i=0;i<ump.num;i++){
                    ump.w[i]/=sum_w;

                    ump.iw[i]=ump.w[i]*i_mult;
                }
            }


        }


}




