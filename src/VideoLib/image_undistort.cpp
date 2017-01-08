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



#include "VideoLib/image_undistort.h"

namespace  rebvo{
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


}
