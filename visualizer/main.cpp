



#ifndef _REENTRANT
#define _REENTRANT
#endif


#include <iostream>

#include "visualizer.h"




int main(int argn,char ** argv)
{


    std::string ConfigName(argn>1?argv[1]:"GlobalConfig");
    Configurator GlobalConfig;

    if(!GlobalConfig.ParseConfigFile(ConfigName.data(),false))
        return false;

    visualizer df(GlobalConfig);



    df.Run();


    return 0;
}
