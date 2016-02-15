

#include <iostream>

#include "rebvo.h"

#include "util.h"
#include "ttimer.h"

bool ConfigGlobalTimer(Configurator &config){
    bool carga=true;
    int simulation_on,simu_time_on,simu_time_step;
    double simu_time_sweep,simu_time_start;
    carga&=config.GetConfigByName("SimuMode","SimulationOn",simulation_on,true);
    carga&=config.GetConfigByName("SimuMode","SimuTimeOn",simu_time_on,true);
    carga&=config.GetConfigByName("SimuMode","SimuTimeSweep",simu_time_sweep,true);
    carga&=config.GetConfigByName("SimuMode","SimuTimeStep",simu_time_step,true);
    carga&=config.GetConfigByName("SimuMode","SimuTimeStart",simu_time_start,true);
    if(!carga){
        return false;
    }
    if(simulation_on && simu_time_on && !GlobalTimer.TurnSimuOn(simu_time_step,simu_time_sweep,simu_time_start)){
            return false;
    }
    return true;
}

void PrintHelp(){

    std::cout<<R"help(
               REBVO Commands:
               q: Quit
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

    if(!ConfigGlobalTimer(GlobalConfig)){
        std::cout<<"Main: error loading Simulation-Timer!\n";
        return -1;
    }

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
        default:
            PrintHelp();
            break;
        }


    }

    cf.CleanUp();
    return 0;
}
