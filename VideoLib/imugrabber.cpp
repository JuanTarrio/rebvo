#include "imugrabber.h"
#include "configurator.h"
#include <TooN/so3.h>




ImuGrabber::ImuGrabber(int list_size,double tsamp)
    :write_inx(list_size+1),read_inx(list_size+1),size(list_size+1),tsample(tsamp)
{
    imu=new ImuData[list_size+1];
    --read_inx;
}

ImuGrabber::ImuGrabber(const std::vector<ImuData> &data_set_data)
    :write_inx(data_set_data.size()+1),read_inx(data_set_data.size()+1),size(data_set_data.size()+1)
{

    imu=new ImuData[size];
    --read_inx;

    for(int i=0;i<size-1;i++){
        imu[i]=data_set_data[i];
        ++write_inx;
    }

    if(data_set_data.size()>1){
        tsample=(data_set_data[data_set_data.size()-1].tstamp-data_set_data[0].tstamp)/(data_set_data.size()-1);
        std::cout<<"\nImuGraber: tsample:"<<tsample<<"\n";
    }else{
        tsample=0;
        std::cout<<"\nImuGraber: coud not set tsample\n";
    }

}


ImuGrabber::~ImuGrabber(){
    delete [] imu;
}

std::vector <ImuData> ImuGrabber::LoadDataSet(const char *data_file, bool comp_data,double time_scale,bool &error){

    std::string dfilen=std::string(data_file);
    std::ifstream ifile(dfilen);

    error=false;
    if(!ifile.is_open()){
        std::cout << "\nImuGrabber: Failed to open file "<<dfilen<<"\n";
        error=true;
        return std::vector <ImuData> ();
    }

    std::vector <ImuData> vector_data;

    int linea =0;
    while(!ifile.eof()){

        std::string line;
        getline(ifile,line);


        line=Configurator::ShrinkWS(line);      //Shrink white spaces

        if(line.size()==0)
            continue;
        if(line[0]=='#')
            continue;

        ImuData data;


        if(comp_data){
            sscanf(line.c_str(),"%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",&data.tstamp,&data.giro[0],&data.giro[1],&data.giro[2],\
                    &data.acel[0],&data.acel[1],&data.acel[2],&data.comp[0],&data.comp[1],&data.comp[2]);
        }else{
            sscanf(line.c_str(),"%lf,%lf,%lf,%lf,%lf,%lf,%lf",&data.tstamp,&data.giro[0],&data.giro[1],&data.giro[2],\
                    &data.acel[0],&data.acel[1],&data.acel[2]);
        }

        data.tstamp*=time_scale;

        linea++;
        vector_data.push_back(data);

    }

    std::cout << "\nImugrabber: Loaded "<<linea << " datums\n";

    return  vector_data;
}

bool ImuGrabber::LoadCamImuSE3(const char *se3_file)
{
     std::ifstream file(se3_file);

     if(!file.is_open()){
         std::cout << "LoadCamImuSE3: could not open SE3 file\n";
         return false;
     }

    std::string num;
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            std::getline(file,num,',');
            RDataSetCam2IMU(i,j)=std::stod(num);
        }
        std::getline(file,num,',');
        TDataSetCam2IMU[i]=std::stod(num);
    }

    std::cout<<"Cam-Imu Rotation:\n"<<RDataSetCam2IMU<<"\nCam-IMU Translation:\n"<<TDataSetCam2IMU<<"\n";

    file.close();
    return true;

}


std::pair <util::CircListIndexer,util::CircListIndexer> ImuGrabber::SeachByTimeStamp(double tstart,double tend){

    util::CircListIndexer inx=read_inx;

  //  std::cout <<"\n1:"<< tstart <<" "<<tend <<" " <<inx <<" "<<read_inx.Size()<<" "<<inx.Size()<<"\n";

    do{
        if((++inx)==write_inx)
            return std::pair<util::CircListIndexer,util::CircListIndexer>(inx,inx); //return empty list
    }while(imu[inx].tstamp<=tstart);

    //if here, there is more data and imu[inx].tstamp>tstart
 //   std::cout <<"2:"<< tstart <<" "<<tend <<" " <<inx <<" "<<read_inx.Size()<<" "<<inx.Size()<<"\n";

    util::CircListIndexer begin=inx;

    do{
        if((++inx)==write_inx)
            return std::pair<util::CircListIndexer,util::CircListIndexer>(begin,begin); //return empty list
    }while(imu[inx].tstamp<tend);


    if(imu[inx].tstamp-tend<1e-12)
        ++inx;                      //if equal, add the last value

   // std::cout <<"3:"<< tstart <<" "<<tend <<" " <<inx <<" "<<read_inx.Size()<<" "<<inx.Size()<<"\n";


    //if here, imu[inx].tstamp<=tend

    return std::pair<util::CircListIndexer,util::CircListIndexer>(begin,inx);   //the value at inx must not be used
}


IntegratedImuData ImuGrabber::GrabAndIntegrate(double tstart,double tend){

    std::pair <util::CircListIndexer,util::CircListIndexer> search_range=SeachByTimeStamp(tstart,tend);


    read_inx=search_range.second-1;                                                                             //grab the buffer

    IntegratedImuData i_data;
    for(util::CircListIndexer inx=search_range.first;inx!=search_range.second;++inx){
        i_data.giro+=RDataSetCam2IMU.T()*imu[inx].giro;
        i_data.acel+=RDataSetCam2IMU.T()*imu[inx].acel;
        i_data.comp+=RDataSetCam2IMU.T()*imu[inx].comp;
        i_data.Rot=i_data.Rot*TooN::SO3<>(RDataSetCam2IMU.T()*imu[inx].giro*tsample).get_matrix();
        i_data.n++;
    }


    i_data.dt=i_data.n*tsample;
    if(i_data.n>1){
        i_data.giro/=i_data.n;
        i_data.acel/=i_data.n;
        i_data.comp/=i_data.n;

        i_data.dgiro=RDataSetCam2IMU.T()*(imu[search_range.second-1].giro-imu[search_range.first].giro);
        i_data.dgiro/=i_data.dt;
    }

    i_data.cacel=i_data.acel+(i_data.dgiro^(-(RDataSetCam2IMU.T()*TDataSetCam2IMU)));


    return i_data;
}
