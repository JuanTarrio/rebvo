



#ifndef _REENTRANT
#define _REENTRANT
#endif


#include "incfiles.h"

#include <iostream>

#include "displayFrontal.h"




int main(int argn,char ** argv)
{


    Configurator GlobalConfig;

    if(!GlobalConfig.ParseConfigFile("GlobalConfig",false))
        return false;

    displayFrontal df(GlobalConfig);



    df.Run();


    return 0;
}
