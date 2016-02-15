



#ifndef _REENTRANT
#define _REENTRANT
#endif


#include <iostream>

#include "visualizer.h"




int main(int argn,char ** argv)
{


    Configurator GlobalConfig;

    if(!GlobalConfig.ParseConfigFile("GlobalConfig",false))
        return false;

    displayFrontal df(GlobalConfig);



    df.Run();


    return 0;
}
