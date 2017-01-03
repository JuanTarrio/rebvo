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


#ifndef CONFIGURATOR_H
#define CONFIGURATOR_H


#ifndef _REENTRANT
#define _REENTRANT
#endif


#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <stdlib.h>

namespace  rebvo{

template <class T>
struct ConfigParam{
    std::string name;
    T value;
};

class ConfigClass{
public:
    std::string name;
    std::vector <ConfigParam<std::string> > params;
    ConfigClass(){name="";}
    ConfigClass(std::string n){name=n;}
};

class Configurator
{

    std::vector <ConfigClass*> config;
public:
    Configurator();
    bool ParseConfigFile(const char *filename, bool verbose=false);

    ConfigClass* FindClassByName(std::string n);

    static std::string ShrinkWS(std::string s);
    static std::string ShrinkNV(std::string s);

    bool GetConfigByName(const char * class_name,const char *param_name,std::string & param,bool vervose=false);

    template <class T> bool GetConfigByName(const char * class_name,const char *param_name,T & param,bool vervose=false);


};

template <class T> bool Configurator::GetConfigByName(const char * class_name,const char *param_name,T & param,bool vervose){
    using namespace std;
    string s;
    if(GetConfigByName(class_name,param_name,s,false)){
    param=atof(s.data());
    if(vervose)
        cout << endl <<"Configurator: encontrado parametro "<< class_name << "::" << param_name <<" = "<< param <<endl;
    return true;
    }
    return false;
}

}
#endif // CONFIGURATOR_H
