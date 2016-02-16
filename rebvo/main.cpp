

#include <iostream>

#include "rebvo.h"

#include "util.h"



void PrintHelp(){

    std::cout<<R"help(
               REBVO Commands:
               q: Quit
               r: Reset
               s: Start Save Video
               p: Take Snapshot
               )help";

}

int main(int argn,char ** argv)
{


    Configurator GlobalConfig;

    //Leo el archivo de configuracion
    if(!GlobalConfig.ParseConfigFile("GlobalConfig",false))
        return -1;


    REBVO cf(GlobalConfig);
	
    if(!cf.Init())
		return -1;

    PrintHelp();

    bool run=true;
    while(run){

        char c;
        std::cin >> c;

        switch(c){
        case 'q':
            run=false;
            std::cout << "\nExiting ...\n";
            break;
        case 's':
            cf.StartSimSave();
            break;
        case 'p':
            cf.TakeSnapshot();
            break;
        case 'r':
            cf.Reset();
            break;
        default:
            PrintHelp();
            break;
        }


    }

    cf.CleanUp();
    return 0;
}
