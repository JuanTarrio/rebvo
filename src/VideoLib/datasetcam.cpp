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

#include "UtilLib/ttimer.h"
#include <string.h>
#include "VideoLib/datasetcam.h"
#include <iostream>
#include <fstream>
#include <UtilLib/configurator.h>
#include <gd.h>
namespace  rebvo{
DataSetCam::DataSetCam(const char *DataSetDir,const char *DataSetFile, Size2D frame_size,double time_scale, const char *log_name)
    :VideoCam(log_name,frame_size),buffer(frame_size),strDir(DataSetDir)
{

    //Loads Dataset file data

    std::string dfilen=std::string(DataSetFile);



    std::ifstream ifile(dfilen);

    if(!ifile.is_open()){
        std::cout << "\nDataSetCamera: Failed to open file "<<dfilen<<"\n";
        error=true;
        return;
    }

    int linea =0;
    while(!ifile.eof()){

        std::string line;
        getline(ifile,line);


        line=Configurator::ShrinkWS(line);


        if(line.size()==0)
            continue;

        if(line[0]=='#')
            continue;


        size_t pos;

        img_time.push_back(std::stod(line,&pos)*time_scale);

        if(pos==0 || pos==std::string::npos || pos==line.size()){
            std::cout << "\nDataSetCamera: EDataFile sintax error line "<< linea <<"String:"<<line <<"\n";
            error=true;
            return;
        }

        if(line.at(pos)==',')
            pos++;

        img_list.push_back(DataSetDir+Configurator::ShrinkNV(Configurator::ShrinkWS(line.substr(pos,line.size()))));


        linea++;

    }

    std::cout << "\nLoaded "<<linea << " File names\n";

    error=false;
    paknum=0;
}

DataSetCam::~DataSetCam(){



}


inline  std::string ToLower(std::string s){

    for(auto &c:s){
        c=std::tolower(c);
    }
    return s;

}

int DataSetCam::LoadImage(const std::string &i_name){

    int pos;
    if((pos=i_name.find_last_of('.'))==std::string::npos){
        std::cout << "\nDataSetCam: Image "<<i_name  <<" File format error\n";
        return -1;
    }

    std::string ending=ToLower(i_name.substr(pos+1,i_name.size()));

    FILE *i_file=fopen(i_name.data(),"r");

    if(i_file==nullptr){
        std::cout << "\nDataSetCam: Image "<<i_name  <<" error on opening, file exist?\n"<<strerror(errno);
        return -1;
    }

    gdImagePtr gdi;

    if(ending.compare("jpg")==0 || ending.compare("jpeg")==0){
        gdi=gdImageCreateFromJpeg(i_file);
    }else if(ending.compare("png")==0){
        gdi=gdImageCreateFromPng(i_file);
    }else if(ending.compare("bmp")==0){
        gdi=gdImageCreateFromWBMP(i_file);
    }else if(ending.compare("png")==0){
        gdi=gdImageCreateFromGif(i_file);
    }else{
        std::cout << "\nDataSetCam: Image "<<i_name  <<" File format (" <<ending<<") not recognized\n";
        fclose (i_file);
        return -1;
    }

    fclose(i_file);

    if(gdi==NULL){
        std::cout << "\nDataSetCam: Image "<<i_name  <<" error on reading with gd\n";
        return -1;
    }


    if(gdi->sx!=buffer.Size().w || gdi->sy!=buffer.Size().h){

        std::cout << "\nDataSetCam: Error the image size ("<<
                     gdi->sx<<","<< gdi->sy<< ") doesn't match the configures size ("<<
                     buffer.Size().w <<","<< buffer.Size().h<< ")\n";
        gdImageDestroy(gdi);
        return -1;
    }

    for(int y=0;y<buffer.Size().h;y++)
        for(int x=0;x<buffer.Size().w;x++){
            auto pix=gdImageGetTrueColorPixel(gdi,x,y);
            buffer(x,y).pix.r=gdTrueColorGetRed(pix);
            buffer(x,y).pix.g=gdTrueColorGetGreen(pix);
            buffer(x,y).pix.b=gdTrueColorGetBlue(pix);
        }


    gdImageDestroy(gdi);
    return 0;

}

int DataSetCam::WaitFrame(bool drop_frames){
    if(error)
        return -1;

    if(img_inx>=std::min(img_time.size(),img_list.size())){
        std::cout << "\nDataSetCamera: End of file list after "<<img_inx<<" Images\n";
        error=true;
        return -1;
    }


    if(LoadImage(img_list[img_inx])<0)
        return -1;
    time=img_time[img_inx];

    img_inx++;
    frm_pending=true;

    return 0;

}

int DataSetCam::GrabFrame(RGB24Pixel *data, double &tstamp, bool drop_frames){

    if(!frm_pending)
        if(WaitFrame(drop_frames)<0)
            return -1;

    buffer.copyTo(data);
    tstamp=time;
    frm_pending=false;
    paknum++;
    return 0;
}

RGB24Pixel* DataSetCam::GrabBuffer(double &tstamp, bool drop_frames){

    if(!frm_pending)
        if(WaitFrame(drop_frames)<0)
            return 0;


    frm_pending=false;

    tstamp=time;
    paknum++;
    return buffer.Data();
}

int DataSetCam::ReleaseBuffer(){
    return 0;
}

bool DataSetCam::SearchFrame(RGB24Pixel *data, const double &tstamp)
{
    int i;
    for(i=0;i<img_time.size()-1;i++){

        if(fabs(tstamp-img_time[i])<fabs(img_time[i+1]-img_time[i])/2.0)
            break;
    }
    if(i==img_time.size()-1 && fabs(tstamp-img_time[i])>fabs(img_time[i]-img_time[i-1])/2.0)
        return false;

    if(LoadImage(img_list[i])<0)
        return false;


    buffer.copyTo(data);
    return true;
}
}
