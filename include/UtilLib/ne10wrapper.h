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

#ifndef NE10WRAPPER_H
#define NE10WRAPPER_H

#include <assert.h>
#include <typeinfo>

//***************** NE10 Wrapper ****************************
//Interface Library to use NE10 lib for Vector Calculations
//And C implementations for non library usage, compile with
//-DUSE_NE10 for library use
//***********************************************************




namespace Ne10 {
//Static linear indexing of 2D fixed size vectors
template <int cols>
inline int Index2Dto1D(int row,int col){
    return row*cols+col;
}

int InitNe10();


template <class T>
inline T PairWiseVAdd(T* __restrict src,int pnum);

#ifdef USE_NE10

#pragma message "Compiling using NE10"
//If compile using NE10, all  the functions wrap to NE10 equivalents, 
//adjusting the number of points to the corresponding vectorial operation

#include <NE10/NE10.h>

// dst=Mat[3][3]*src[3][pnum]
template <class T>
inline void MulMatVect3(T* __restrict dst,T * __restrict Mat,T* __restrict src,int pnum){

    assert(typeid(T)==typeid(float));
    ne10_mulcmatvec_cm3x3f_v3f_neon((ne10_vec3f_t *)dst,(ne10_mat3x3f_t *)Mat,(ne10_vec3f_t *)src,pnum);

}

//dst[pnum]=Vec+src[pnum]
template <class T>
inline void AddVect3(T* __restrict dst,T * __restrict Vec,T* __restrict src,int pnum){

    assert(typeid(T)==typeid(float));
    ne10_addc_vec3f_neon((ne10_vec3f_t *)dst,(ne10_vec3f_t *)src,(ne10_vec3f_t *)Vec,pnum);

}

//dst[pnum]=dst[pnum]+src[pnum]
template <class T>
inline void AddVect3InPlace(T* dst,T * Vec,int pnum){

    assert(typeid(T)==typeid(float));
    ne10_addc_vec3f_neon((ne10_vec3f_t *)dst,(ne10_vec3f_t *)dst,(ne10_vec3f_t *)Vec,pnum);

}


//dst[pnum]=src1[pnum]+src2[pnum]
template <class T>
inline void AddVect(T* __restrict dst,T* __restrict src1,T* __restrict src2,int pnum){

    assert(typeid(T)==typeid(float));
    //assert((pnum&0x03)==0);
    int pnum4=pnum>>2;	//Divide by 4 the number of points and use neon SIMD

    ne10_add_vec4f_neon((ne10_vec4f_t *)dst,(ne10_vec4f_t *)src1,(ne10_vec4f_t *)src2,pnum4);

    for(int i=(pnum4<<2);i<pnum;i++){	
        dst[i]=src1[i]+src2[i];
    }

}

//dst[pnum]=src[pnum]+c

template <class T>
inline void AddCVect(T* dst,const T & c,T* src,int pnum){

    assert(typeid(T)==typeid(float));
    assert((pnum&0x03)==0);
    pnum=pnum>>2;

    ne10_vec4f_t  c_v;
    c_v.x=c;
    c_v.y=c;
    c_v.z=c;
    c_v.w=c;

    ne10_addc_vec4f_neon((ne10_vec4f_t *)dst,(ne10_vec4f_t *)src,&c_v,pnum);


}

//dst[pnum]=c*src1[pnum]

template <class T>
inline void MulCVect(T* dst,const T & c,T* src,int pnum){

    assert(typeid(T)==typeid(float));
    assert((pnum&0x03)==0);
    pnum=pnum>>2;

    ne10_vec4f_t  c_v;
    c_v.x=c;
    c_v.y=c;
    c_v.z=c;
    c_v.w=c;

    ne10_mulc_vec4f_neon((ne10_vec4f_t *)dst,(ne10_vec4f_t *)src,&c_v,pnum);


}

//dst[pnum]=dst[pnum]+c*src[pnum]

template <class T>
inline void MlAcCVect(T* dst,const T & c,T* src,int pnum){

    assert(typeid(T)==typeid(float));
    assert((pnum&0x03)==0);
    pnum=pnum>>2;

    ne10_vec4f_t  c_v;
    c_v.x=c;
    c_v.y=c;
    c_v.z=c;
    c_v.w=c;

    ne10_mlac_vec4f_neon((ne10_vec4f_t *)dst,(ne10_vec4f_t *)dst,(ne10_vec4f_t *)src,&c_v,pnum);
}
/*mm
template <class T>
inline void MulVect(T* dst,T* src1,T* src2,int pnum){


    assert(typeid(T)==typeid(float));
    assert((pnum&0x03)==0);

    ne10_vmul_vec4f_neon((ne10_vec4f_t *)dst,(ne10_vec4f_t *)src1,(ne10_vec4f_t *)src2,pnum>>2);
}


template <class T>
inline void MulVect(T* dst,T* src1,T* src2,int pnum){


    assert(typeid(T)==typeid(float));
    assert((pnum&0x03)==0);
    ne10_vmul_vec2f_neon((ne10_vec2f_t  *)dst,(ne10_vec2f_t  *)src1,(ne10_vec2f_t  *)src2,pnum/2);
}*/


//dst[pnum]=src1[pnum].*src2[pnum] (element wise)

template <class T>
inline void MulVect(T* dst,T* src1,T* src2,int pnum){


    assert(typeid(T)==typeid(float));
    assert((pnum&0x03)==0);
    ne10_mul_float_neon((ne10_float32_t  *)dst,(ne10_float32_t  *)src1,(ne10_float32_t  *)src2,pnum);
}


//dst[pnum]=dst[pnum]+src1[pnum].*src2[pnum]
template <class T>
inline void MlAcVect(T* __restrict dst,T* __restrict src1,T* __restrict src2,int pnum){

    assert(typeid(T)==typeid(float));
    assert((pnum&0x03)==0);
    ne10_vmla_vec4f_neon((ne10_vec4f_t *)dst,(ne10_vec4f_t *)dst,(ne10_vec4f_t *)src1,(ne10_vec4f_t *)src2,pnum>>2);
}

//dst=src1[pnum].*src2[pnum]

template <class T>
inline T DotProduct(T* __restrict src1,T* __restrict src2,int pnum){

    assert(typeid(T)==typeid(float));
    assert((pnum&0x03)==0);
    pnum=pnum>>2;

    T prod[pnum];
    ne10_dot_vec4f_neon((ne10_float32_t *)prod,(ne10_vec4f_t *)src1,(ne10_vec4f_t *)src2,pnum);	//Dot product of 4-vectors in src1 and src2

    return PairWiseVAdd(prod,pnum);	//PairWise add the individual products for improved presition
}


#else //Compile not using NE10

//******* C implementations of NE10 functions ********* 
 
template <class T>
inline void MulMatVect3(T* __restrict dst,T * __restrict Mat,T* __restrict src,int pnum){

    for(int i=0;i<3;i++){

        for(int k=0;k<pnum;k++){
            dst[Index2Dto1D<3>(k,i)]=0;
            for(int j=0;j<3;j++){
                dst[Index2Dto1D<3>(k,i)]+=Mat[Index2Dto1D<3>(j,i)]*src[Index2Dto1D<3>(k,j)];
            }
        }

    }

}

template <class T>
inline void AddVect(T* __restrict dst,T* __restrict src1,T* __restrict src2,int pnum){

    for(int k=0;k<pnum;k++){
        dst[k]=src1[k]+src2[k];
    }

}


template <class T>
inline void AddCVect(T* __restrict dst,const T & __restrict c,T* __restrict src,int pnum){

    for(int k=0;k<pnum;k++){
        dst[k]=c+src[k];
    }

}

template <class T>
inline void MulCVect(T* __restrict dst,const T & __restrict c,T* __restrict src,int pnum){

    for(int k=0;k<pnum;k++){
        dst[k]=c*src[k];
    }
}


template <class T>
inline void MlAcCVect(T* __restrict dst,const T & __restrict c,T* __restrict src,int pnum){

    for(int k=0;k<pnum;k++){
        dst[k]+=c*src[k];
    }
}

template <class T>
inline void MulVect(T* __restrict dst,T* __restrict src1,T* __restrict src2,int pnum){

    for(int k=0;k<pnum;k++){
        dst[k]=src1[k]*src2[k];
    }
}

template <class T>
inline void MlAcVect(T* __restrict dst,T* __restrict src1,T* __restrict src2,int pnum){

    for(int k=0;k<pnum;k++){
        dst[k]+=src1[k]*src2[k];
    }
}




template <class T>
inline void AddVect3(T* __restrict dst,T * __restrict Vec,T* __restrict src,int pnum){

        for(int k=0;k<pnum;k++){
            for(int i=0;i<3;i++){
                dst[Index2Dto1D<3>(k,i)]=Vec[i]+src[Index2Dto1D<3>(k,i)];
            }
        }

}

template <class T>
inline void AddVect3InPlace(T* __restrict dst,T * __restrict Vec,int pnum){

        for(int k=0;k<pnum;k++){
            for(int i=0;i<3;i++){
                dst[Index2Dto1D<3>(k,i)]+=Vec[i];
            }
        }

}

//dst=src1[pnum].*src2[pnum]
template <class T>
inline T DotProduct(T* __restrict src1,T* __restrict src2,int pnum){

    T prod[pnum];
    MulVect(prod,src1,src2,pnum);

    return PairWiseVAdd(prod,pnum);
}

#endif


//Other functions for vector procesing

//Pair Wise sum of the elements of src using Neon SIMD if implemented 
template <class T>
inline T PairWiseVAdd(T* __restrict src,int pnum){


    T dst0[pnum>>1];
    T dst1[pnum>>1];
    T *p_in=src,*p_out=dst0;
    T odd=0;
    bool swap=true;
    while(pnum>3){	//Iterate until is below 4

        odd+=pnum&0x01?p_in[pnum-1]:0;			//pnum is odd? sum the odd elements
        pnum>>=1;
        AddVect(p_out,p_in,&p_in[pnum],pnum);	//Neon add of two halfs

        if(swap){			//Swap between 2 buffers for storing
            p_in=dst0;
            p_out=dst1;
        }else{
            p_in=dst1;
            p_out=dst0;
        }
        swap=!swap;
    }
    for(int i=0;i<pnum;i++){
        odd+=p_in[i];			//add 
    }
    return odd;
}


// dst=Mat*src
// dst and src had to be in the formar [x1 x2 x3 ... xn y1 y2 y3 ... yn z1 z2 z3 ... zn]

template <class T>
inline void MulMat3Vect(T* __restrict dst,T * __restrict Mat,T* __restrict src,int pnum){

    MulCVect (&dst[pnum*0],Mat[0*3+0],&src[pnum*0],pnum);
    MlAcCVect(&dst[pnum*0],Mat[1*3+0],&src[pnum*1],pnum);
    MlAcCVect(&dst[pnum*0],Mat[2*3+0],&src[pnum*2],pnum);

    MulCVect (&dst[pnum*1],Mat[0*3+1],&src[pnum*0],pnum);
    MlAcCVect(&dst[pnum*1],Mat[1*3+1],&src[pnum*1],pnum);
    MlAcCVect(&dst[pnum*1],Mat[2*3+1],&src[pnum*2],pnum);

    MulCVect (&dst[pnum*2],Mat[0*3+2],&src[pnum*0],pnum);
    MlAcCVect(&dst[pnum*2],Mat[1*3+2],&src[pnum*1],pnum);
    MlAcCVect(&dst[pnum*2],Mat[2*3+2],&src[pnum*2],pnum);

}

template <class T>
inline void SE3onP3Matrix(T * __restrict Rot,T* __restrict Vel,T* __restrict Pos,T* __restrict Pos0,int pnum){

    if(pnum<=0)
        return;
    T Ptmp[pnum*3];
    MulMatVect3<T>(Ptmp,Rot,Pos0,pnum);
    AddVect3<T>(Pos,Vel,Ptmp,pnum);

}

//Pos = Rot*Pos0 + Vel
template <class T>
inline void SE3on3PMatrix(T * __restrict Rot,T* __restrict Vel,T* __restrict Pos,T* __restrict Pos0,int pnum){

    if(pnum<=0)
        return;

    MulMat3Vect<T>(Pos,Rot,Pos0,pnum);

    AddCVect<T>(&Pos[pnum*0],Vel[0],&Pos[pnum*0],pnum);
    AddCVect<T>(&Pos[pnum*1],Vel[1],&Pos[pnum*1],pnum);
    AddCVect<T>(&Pos[pnum*2],Vel[2],&Pos[pnum*2],pnum);

}

//Project a point from image plane to 3d space, using zf as focal length
//I3P is a vector with components (x_image,y_image,i_depth)

template <class T>
inline void ProyI3Pto3PMatrix(T* dst,T* I3P,T zf,const int pnum){

    for(int ikl=0;ikl<pnum;ikl++){
        dst[pnum*2+ikl]=1/I3P[pnum*2+ikl];	//Calc depth from IDepth
    }
    T Pz_zf[pnum];
    Ne10::MulCVect(Pz_zf,1/zf,&dst[pnum*2],pnum);	//Multiply by the reciprocal of the focal length

    Ne10::MulVect(&dst[pnum*0],Pz_zf,&I3P[pnum*0],pnum); //Multiply with image coordinate to obtint 3D x,y
    Ne10::MulVect(&dst[pnum*1],Pz_zf,&I3P[pnum*1],pnum);
}

//Project a point from  3d space to image plane, using zf as focal length
//dst is vector with components (x_image,y_image,i_depth)

template <class T>
inline void ProyP3toI3PMatrix(T* dst,T* P3,T zf,const int pnum){

    T*d,*s;

    d=&dst[pnum*2];s=&P3[pnum*2];
    for(int ikl=0;ikl<pnum;ikl++,d++,s++){
        (*d)=1/(*s);								//IDepth is the reciprocal of the z-component
    }

    T Pz_zf[pnum];
    Ne10::MulCVect(Pz_zf,zf,&dst[pnum*2],pnum);		 
    Ne10::MulVect(&dst[pnum*0],Pz_zf,&P3[pnum*0],pnum);
    Ne10::MulVect(&dst[pnum*1],Pz_zf,&P3[pnum*1],pnum);


}

}




#endif // NE10WRAPPER_H
