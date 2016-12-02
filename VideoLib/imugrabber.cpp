#include "imugrabber.h"
#include "configurator.h"
#include <TooN/so3.h>


const TooN::Matrix<3,3> RDataSetCam2IMU=TooN::Data(0.0148655429818, -0.999880929698, 0.00414029679422,
                                                   0.999557249008, 0.0149672133247, 0.025715529948,
                                                  -0.0257744366974, 0.00375618835797, 0.999660727178);

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

    if(i_data.n>1){
        i_data.giro/=i_data.n;
        i_data.acel/=i_data.n;
        i_data.comp/=i_data.n;
    }
    i_data.dt=i_data.n*tsample;

    return i_data;
}
