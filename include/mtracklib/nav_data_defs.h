#ifndef NAV_DATA_DEFS_H
#define NAV_DATA_DEFS_H

#include "UtilLib/util.h"
namespace  rebvo{
struct compressed_nav_pkg{
    fvector<3,float> Pos;
    fvector<3,float> Pose;
    fvector<3,float> Rot;
    fvector<3,float> Vel;
    fvector<3,float> G;
    float K;
    float dt;
    //fvector<4,float> ref_err;
};
}

#endif // NAV_DATA_DEFS_H
