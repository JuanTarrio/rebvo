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

#include "UtilLib/ne10wrapper.h"

namespace Ne10 {

#ifdef USE_NE10

static ne10_result_t Ne10Stated =NE10_ERR;

int InitNe10(){
    if(Ne10Stated==NE10_OK)
        return 0;

    Ne10Stated = ne10_init();
    if (Ne10Stated != NE10_OK){
        printf ("\nNE10 init failed.\n");
        return -1;
    }

    printf ("\nNE10 has been initialized.\n");
    return 0;

}



#else



int InitNe10(){
    return 0;

}



#endif



}
