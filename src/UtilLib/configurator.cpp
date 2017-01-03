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


#include "UtilLib/configurator.h"

using namespace std;
namespace  rebvo{
Configurator::Configurator()
{
}

string Configurator::ShrinkWS(string s){
    const char *p=s.data();
    int p1,p2;

    for(p1=0;p1<s.size();p1++)
	if(p[p1]!=' ' && p[p1]!=0x09)
	    break;

    if(p1==s.size())
	return string("");

    for(p2=s.size()-1;p2>=0;p2--)
	if(p[p2]!=' ' && p[p2]!=0x09)
	    break;
    return s.substr(p1,p2-p1+1);
}

string Configurator::ShrinkNV(string s){
    const char *p=s.data();
    int p1,p2;

    for(p1=0;p1<s.size();p1++)
    if(p[p1]!=10 && p[p1]!=13)
        break;

    if(p1==s.size())
    return string("");

    for(p2=s.size()-1;p2>=0;p2--)
    if(p[p2]!=10 && p[p2]!=13)
        break;
    return s.substr(p1,p2-p1+1);
}


ConfigClass* Configurator::FindClassByName(string n){

    for(int i=0;i<config.size();i++){
	if(n.compare(config.at(i)->name)==0)
	    return config.at(i);
    }
    return 0;
}

bool Configurator::ParseConfigFile(const char * filename,bool vervose){
    string s,n,v;
    ifstream file(filename,ifstream::in);
    int pos;
    int linea=0;

    ConfigClass* clase_actual=new ConfigClass("");
    config.push_back(clase_actual);

    ConfigParam<string> param;

    if(!file.is_open()){
	cout << endl<< "Configurator: No puedo abrir el archivo de configuracion!" <<endl;
	return false;
    }

    if(vervose)
	cout << endl<<"Configurator: archivo abierto... leyendo configuracion"<<endl;

    while(!file.eof()){
	linea++;
	getline(file,s);	//Leo una linea


    if((pos=s.find("//",0))!=string::npos){	//elimino comentarios
	    s.resize(pos);
	}

	s=ShrinkWS(s);

	if(s.size()==0)		//Linea vacia? continuo...
	    continue;

	if(s.at(0)=='&')    //Inicio una subseccion de clase
	{
	    s=ShrinkWS(s.substr(1,s.size()));

	    if(vervose)
		cout <<"Configurator: entrando a la subseccion "<<s<<" OK!"<<endl;


	    if((clase_actual=FindClassByName(s))==0){	//la clase no esta en la lista debo agregarla
		clase_actual=new ConfigClass(s);
		config.push_back(clase_actual);

		if(vervose)
		    cout <<"Configurator: debo crearla!"<<endl;

	    }
	    continue;
	}

	pos=s.find('=');
	if(pos==0){
	    cout << "Configurator: Error de sintaxis linea "<< linea << ", nombre vacio!"<<endl;
	    return false;
	}else if(pos==string::npos){
	    cout << "Configurator: Error de sintaxis linea "<< linea << ", use nombre = valor"<<endl;
	    return false;
	}

	n=ShrinkWS(s.substr(0,pos));

	if(pos+1==s.size())
	    v="";
	else
	    v=s.substr(pos+1,s.size());


	param.name=n;
	param.value=v;

	clase_actual->params.push_back(param);

	if(vervose)
	    cout << "Configurator: leido parametro " << n <<" : " << v << " en la subseccion " << clase_actual->name<< " OK!" << endl;

    }

    file.close();
    return true;

}
bool Configurator::GetConfigByName(const char * class_name,const char *param_name,string & param,bool vervose){

    string cn(class_name);
    string pn(param_name);
    ConfigClass* clase_actual;

    cn=ShrinkWS(cn);
    pn=ShrinkWS(pn);

    if((clase_actual=FindClassByName(cn))==0){
	cout << endl <<"Configurator: error, no puedo encontrar la clase " << cn << " en el archivo de configuracion"<<endl;
	return false;
    }

    for(int i=0;i<clase_actual->params.size();i++){
	if(pn.compare(clase_actual->params[i].name)==0){
	    param=clase_actual->params[i].value;
	    if(vervose)
		cout << endl <<"Configurator: encontrado parametro " << cn << "::" << pn <<" = "<< param <<endl;
	    return true;
	}
    }

    cout << endl <<"Configurator: error, no puedo encontrar el parametro " << pn << " dentro de la clase "<< cn <<endl;
    return false;

}


}
