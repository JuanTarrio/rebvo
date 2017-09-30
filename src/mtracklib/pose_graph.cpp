#include "mtracklib/isam_defs.h"
#include "mtracklib/pose_graph.h"
#include <TooN/Cholesky.h>
#include <isam/robust.h>

namespace  rebvo {
using namespace isam;

struct isam_data{
    Slam slam;


    std::vector <Pose3d_Node*> kf_nodes;
    std::vector <Scale_Node*> kf_scale_nodes;
    std::vector <int> kf_pose_inx;
};



pose_graph::pose_graph()
{
}

void pose_graph::saveToFile(std::ofstream &file)
{
    uint frame_num=frame_meas.size();
    util::dumpElement(file,frame_num);

    for(OdometryMeas &m:frame_meas)
        m.dumpToFile(file);

}

void pose_graph::loadFromFile(std::ifstream &file)
{

    uint frame_num;
    util::readElement(file,frame_num);

    for(int i=0;i<frame_num;i++){
        OdometryMeas m;
        m.readFromFile(file);
        frame_meas.push_back(m);
    }
}


void pose_graph::smoothScale(){

    smooth_scale.resize(frame_meas.size());

    smooth_scale[frame_meas.size()-1].first=frame_meas.back().K;
    smooth_scale[frame_meas.size()-1].second=frame_meas.back().WK;
    for (int i=frame_meas.size()-2;i>=0;i--){

        double P_k_k=1/frame_meas[i].WK;
        double Q_k=frame_meas[i].QK/(1+util::square(frame_meas[i].K));
        double C=P_k_k/(P_k_k+Q_k);

        double x_kp1_n=atan(smooth_scale[i+1].first);
        double x_k_k=atan(frame_meas[i].K);
        double x_k_n=x_k_k+C*(x_kp1_n-x_k_k);

        smooth_scale[i].first=tan(x_k_n);

        double P_k_n=P_k_k+C*(1/smooth_scale[i+1].second-(P_k_k+Q_k))*C;
        smooth_scale[i].second=1/P_k_n;

    }
}
void pose_graph::buildScale(){

    smooth_scale.resize(frame_meas.size());

    smooth_scale[0].first=frame_meas[0].K;
    smooth_scale[0].second=frame_meas[0].WK;

    for (int i=1;i<frame_meas.size();i++){

        double P_k_k=1/frame_meas[i].WK;
        double Q_k=frame_meas[i].QK/(1+util::square(frame_meas[i-1].K));
        double P_k_km1=1/frame_meas[i-1].WK+Q_k;
        double P_k_z=1/(1/P_k_k-1/P_k_km1);


        double x_k_k=atan(frame_meas[i].K);
        double x_k_km1=atan(frame_meas[i-1].K);

        double x_k_z=P_k_z*(x_k_k/P_k_k-x_k_km1/P_k_km1);
        smooth_scale[i].first=tan(x_k_z);
        smooth_scale[i].second=1/P_k_z;


    }
}

void pose_graph::buildAngScale(){

    smooth_scale_z.resize(frame_meas.size());

    smooth_scale_z[0].first=atan(frame_meas[0].K);
    smooth_scale_z[0].second=(frame_meas[0].WK);

    for (int i=1;i<frame_meas.size();i++){

        double P_k_k=1/frame_meas[i].WK;
        double Q_k=frame_meas[i].QK/(1+util::square(frame_meas[i-1].K));
        double P_k_km1=1/frame_meas[i-1].WK+Q_k;
        double P_k_z=1/(1/P_k_k-1/P_k_km1);


        double x_k_k=atan(frame_meas[i].K);
        double x_k_km1=atan(frame_meas[i-1].K);

        double x_k_z=P_k_z*(x_k_k/P_k_k-x_k_km1/P_k_km1);
        smooth_scale_z[i].first=x_k_z;
        smooth_scale_z[i].second=1/P_k_z;


    }
}


TooN::Matrix <6>  scaleCov(TooN::Matrix <6> cov,double K){
    cov.slice<3,0,3,3>()*=K;
    cov.slice<0,3,3,3>()*=K;
    cov.slice<0,0,3,3>()*=K*K;
    return cov;
}
TooN::Matrix <6>  scaleInf(TooN::Matrix <6> inf,double K,double W_K,TooN::Vector<6> relPos){
    /*inf.slice<3,0,3,3>()/=K;
    inf.slice<0,3,3,3>()/=K;
    inf.slice<0,0,3,3>()/=K*K;

    Matrix<3,3> wK=relPos.slice<0,3>().as_diagonal()*relPos.slice<0,3>().as_diagonal()*W_K;

    inf.slice<0,0,3,3>()+=*/

    TooN::Matrix <6>cov=scaleCov(TooN::Cholesky<6>(inf).get_inverse(),K);

    TooN::Matrix<3,3> RK=relPos.slice<0,3>().as_diagonal()*relPos.slice<0,3>().as_diagonal();
    RK/=W_K;

    cov.slice<0,0,3,3>()+=RK;

    return TooN::Cholesky<6>(cov).get_inverse();
}

void pose_graph::loadIsam(std::vector<keyframe> &kf_list){

    using namespace std;

    if(!isam_graph)
        isam_graph=new isam_data();


    Pose3d_Node* pose0 = new Pose3d_Node();
    isam_graph->slam.add_node(pose0);
    isam_graph->kf_nodes.push_back(pose0);


    Scale_Node *snode0=new Scale_Node();
    isam_graph->kf_scale_nodes.push_back(snode0);
    isam_graph->slam.add_node(snode0);

    int curr_kf=0;

    // create a prior on the camera position
    Pose3d origin;
    Noise noise6 = Information(100. * eye(6));
    Pose3d_Factor* prior = new Pose3d_Factor(pose0, origin, noise6);

    isam_graph->slam.add_factor(prior);

    //Create prior on first scale

    isam_graph->slam.add_factor(new Scale_Factor(snode0,Point1d(frame_meas[0].K),Information(frame_meas[0].WK*eye(1))));


    int poses_num=frame_meas.size();


    cout << "Adding " << poses_num<< " poses and factors:" << endl;

    Pose3d_Node* prior_pose=pose0;






    for(int pose_inx=0;pose_inx<poses_num;pose_inx++){

        //Pose Node
        Pose3d_Node* new_pose = new Pose3d_Node();
        isam_graph->slam.add_node(new_pose);

        //Add odometry meas
        Noise rel_noise = Information(matrix_toon2eigen(scaleInf(frame_meas[pose_inx].WrelPosPose,frame_meas[pose_inx].K,frame_meas[pose_inx].WK,frame_meas[pose_inx].relPosPose)));
        Pose3d delta=revboRP2isam(frame_meas[pose_inx].relPosPose,frame_meas[pose_inx].K);

      //  cout <<"Odometry Factor KF:"<<frame_meas[pose_inx].KF_ID<<" Delta: " <<delta  << endl;

        Pose3d_Pose3d_Factor* odo = new Pose3d_Pose3d_Factor(prior_pose, new_pose, delta, rel_noise);

        //Push factor
        isam_graph->slam.add_factor(odo);

        if(pose_inx>74){


            //Add G as a pose factor

            GMeas_Factor *g_prior=new GMeas_Factor(new_pose,Point3d(vector_toon2eigen(frame_meas[pose_inx].GMeas)),Covariance(matrix_toon2eigen(frame_meas[pose_inx].Rg)));
            isam_graph->slam.add_factor(g_prior);
        }


        //is new keyframe?

        if(frame_meas[pose_inx].KF_ID!=curr_kf){

            isam_graph->kf_nodes.push_back(new_pose);
            curr_kf=frame_meas[pose_inx].KF_ID;

            //if(curr_kf<kf_list.size()){
                //Scale Node
            Scale_Node *snode=new Scale_Node();
            isam_graph->kf_scale_nodes.push_back(snode);
            isam_graph->slam.add_node(snode);

            //Scale prior

            Scale_Factor *scl_prior=new Scale_Factor(snode,
                                                     Point1d(frame_meas[pose_inx].K),
                                                     Information(frame_meas[pose_inx].WK*eye(1)*0));

            isam_graph->slam.add_factor(scl_prior);
            std::cout <<snode->value()<<frame_meas[pose_inx].K<<"\n";

        }



       // cout <<"Pose Value:" << new_pose->value() << endl;
        prior_pose=new_pose;
    }



    //isam_graph->slam.batch_optimization();

    std::ofstream pose_log("pose_log0.txt");

    if(pose_log.is_open()){
        for(auto &node:isam_graph->slam.get_nodes()){

            Pose3d_Node* pose_node=dynamic_cast<Pose3d_Node*>(node);

            if(pose_node==NULL)
                continue;
            const Pose3d &pose=pose_node->value();

            pose_log <<pose.x()<<" "<<pose.y()<<" "<<pose.z()<< endl;
        }
    }
    pose_log.close();

    std::ofstream scale_log("scale_log0.txt");

    if(scale_log.is_open()){
        for(auto &node:isam_graph->slam.get_nodes()){

            Scale_Node* scale_node=dynamic_cast<Scale_Node*>(node);

            if(scale_node==NULL)
                continue;

            scale_log <<scale_node->value().x()<< std::endl;
        }
    }
    scale_log.close();

    std::cout <<"ISAM: Poses loaded\n";

}


void pose_graph::loadIsamWithScaleMeas(){

    using namespace std;

    buildAngScale();

    if(!isam_graph)
        isam_graph=new isam_data();

    Pose3d_Node* pose0 = new Pose3d_Node();
    isam_graph->slam.add_node(pose0);
    isam_graph->kf_nodes.push_back(pose0);


    Scale_Node *snode0=new Scale_Node();
    isam_graph->kf_scale_nodes.push_back(snode0);
    isam_graph->slam.add_node(snode0);

    int curr_kf=0;

    // create a prior on the camera position

    TooN::Vector <6>p0=TooN::Zeros;
    p0.slice<3,3>()=TooN::SO3<>(TooN::unit(-frame_meas[0].AcelMeas),TooN::makeVector(0,1,0)).ln();

    Pose3d origin=revboRP2isam(p0,1);
    Noise noise6 = Information(1.0* eye(6));
    Pose3d_Factor* prior = new Pose3d_Factor(pose0, origin, noise6);

    isam_graph->slam.add_factor(prior);

    // Create a prior on scale
    isam_graph->slam.add_factor(new Scale_Factor(snode0,Point1d(1),Information(1.0e-6*eye(1))));

    int poses_num=frame_meas.size();


    cout << "Adding " << poses_num<< " poses and factors:" << endl;

    Pose3d_Node* prior_pose=pose0;
    Scale_Node * prior_scale=snode0;

    for(int pose_inx=0;pose_inx<poses_num;pose_inx++){

        //Pose Node
        Pose3d_Node* new_pose = new Pose3d_Node();
        isam_graph->slam.add_node(new_pose);

        //Scale Node
        Scale_Node *new_scale=new Scale_Node();
        isam_graph->slam.add_node(new_scale);

        //Add scale - scale factor

        Scale_Scale_Angular_Factor *scl_odo=new Scale_Scale_Angular_Factor(prior_scale,new_scale,0,
                                                                       Covariance(frame_meas[pose_inx].QK*eye(1)/(1+util::square(frame_meas[pose_inx].K))));
        isam_graph->slam.add_factor(scl_odo);

       // Scale_Factor *scl_prior=new Scale_Factor(new_scale,Point1d(smooth_scale[pose_inx].first),Information(smooth_scale[pose_inx].second*eye(1)));

        if(pose_inx>74){
            AngularScale_Factor*scl_prior=new AngularScale_Factor(new_scale,vector_toon2eigen(frame_meas[pose_inx].VisualMeas),
                                                                  vector_toon2eigen(frame_meas[pose_inx].AcelMeas),
                                                                  vector_toon2eigen(frame_meas[pose_inx].GMeas),
                                                                  matrix_toon2eigen(frame_meas[pose_inx].Rv),
                                                                  matrix_toon2eigen(frame_meas[pose_inx].Rs+frame_meas[pose_inx].Rg),
                                                                  Point1d(),Information(eye(1)));
            // Scale_Factor *scl_prior=new Scale_Factor(new_scale,Point1d(frame_meas[pose_inx].K),Information(frame_meas[pose_inx].WK*eye(1)));

            isam_graph->slam.add_factor(scl_prior);

            //Add G as a pose factor

            GMeas_Factor *g_prior=new GMeas_Factor(new_pose,Point3d(vector_toon2eigen(frame_meas[pose_inx].GMeas)),Covariance(matrix_toon2eigen(frame_meas[pose_inx].Rg)));
            isam_graph->slam.add_factor(g_prior);
        }

        //Add odometry meas
        Noise rel_noise = Information(matrix_toon2eigen(frame_meas[pose_inx].WrelPosPose));
        Pose3d delta=revboRP2isam(frame_meas[pose_inx].relPosPose,1.0);

        //cout <<"Odometry Factor KF:"<<frame_meas[pose_inx].KF_ID<<" Delta: " <<delta<<" WK:"<<frame_meas[pose_inx].WK<<" Acel:"<<frame_meas[pose_inx].AcelMeas  << endl;

        AngularScaled_Pose3d_Pose3d_Factor* odo = new AngularScaled_Pose3d_Pose3d_Factor(prior_pose,new_scale, new_pose, delta, rel_noise);

        isam_graph->slam.add_factor(odo);


        //is new keyframe?

        if(frame_meas[pose_inx].KF_ID!=curr_kf){

            isam_graph->kf_nodes.push_back(new_pose);

            isam_graph->kf_scale_nodes.push_back(new_scale);

            curr_kf=frame_meas[pose_inx].KF_ID;




          //  Scale_Factor *scl_prior=new Scale_Factor(new_scale,
            //                                         Point1d(frame_meas[pose_inx].K),
              //                                       Information(frame_meas[pose_inx].WK*eye(1)));

            std::cout <<pose_inx<<"\n";

        }


        prior_pose=new_pose;
        prior_scale=new_scale;

    }




    //isam_graph->slam.batch_optimization();

    std::ofstream pose_log("pose_log0.txt");

    if(pose_log.is_open()){
        int pose_inx=0;
        for(auto &node:isam_graph->slam.get_nodes()){

            Pose3d_Node* pose_node=dynamic_cast<Pose3d_Node*>(node);

            if(pose_node==NULL)
                continue;
            const Pose3d &pose=pose_node->value();


            Eigen::Vector3d g_vect(0,9.8,0);
            Eigen::Vector3d g_rot=pose.rot().oRw()*g_vect;
            if(pose_inx<poses_num)
                pose_log <<pose.x()<<" "<<pose.y()<<" "<<pose.z()<<" "<<g_rot(0)<<" "<<g_rot(1)<<" "<<g_rot(2)
                        <<" "<<frame_meas[pose_inx].AcelMeas[0]<<" "<<frame_meas[pose_inx].AcelMeas[1]<<" "<<frame_meas[pose_inx].AcelMeas[2]<< endl;
            pose_inx++;
        }
    }
    pose_log.close();

    std::ofstream scale_log("scale_log0.txt");

    if(scale_log.is_open()){
        for(auto &node:isam_graph->slam.get_nodes()){

            Scale_Node* scale_node=dynamic_cast<Scale_Node*>(node);

            if(scale_node==NULL)
                continue;

            scale_log <<scale_node->value().x()<< std::endl;
        }
    }
    scale_log.close();

    std::cout <<"ISAM: Poses loaded\n";

}


void pose_graph::loadIsamWithScaleMonster(){

    using namespace std;

    buildAngScale();
    smoothScale();

    if(!isam_graph)
        isam_graph=new isam_data();

    Pose3d_Node* pose0 = new Pose3d_Node();
    isam_graph->slam.add_node(pose0);
    isam_graph->kf_nodes.push_back(pose0);


    Scale_Node *snode0=new Scale_Node();
    isam_graph->kf_scale_nodes.push_back(snode0);
    isam_graph->slam.add_node(snode0);
    isam_graph->kf_pose_inx.push_back(0);

    int curr_kf=0;

    // create a prior on the camera position

    TooN::Vector <6>p0=TooN::Zeros;
    p0.slice<3,3>()=TooN::SO3<>(TooN::unit(-frame_meas[0].AcelMeas),TooN::makeVector(0,1,0)).ln();

    Pose3d origin=revboRP2isam(p0,1);
    Noise noise6 = Information(1.0* eye(6));
    Pose3d_Factor* prior = new Pose3d_Factor(pose0, origin, noise6);

    isam_graph->slam.add_factor(prior);

    // Create a prior on scale
    isam_graph->slam.add_factor(new Scale_Factor(snode0,Point1d(1),Information(1.0*eye(1))));

    int poses_num=frame_meas.size();


    cout << "Adding " << poses_num<< " poses and factors:" << endl;

    Pose3d_Node* prior_pose=pose0;
    Scale_Node * prior_scale=snode0;

    for(int pose_inx=0;pose_inx<poses_num;pose_inx++){

        //Pose Node
        Pose3d_Node* new_pose = new Pose3d_Node();
        isam_graph->slam.add_node(new_pose);

        //Scale Node
        Scale_Node *new_scale=new Scale_Node();
        isam_graph->slam.add_node(new_scale);

        //Add scale - scale factor

        double delta_K=frame_meas[pose_inx].K/(pose_inx>0?frame_meas[pose_inx-1].K:1);
        //double delta_K=smooth_scale[pose_inx].first/(pose_inx>0?smooth_scale[pose_inx-1].first:1);
        Scale_Scale_Factor *scl_odo=new Scale_Scale_Factor(prior_scale,new_scale,delta_K,
                                                                       Covariance(frame_meas[pose_inx].QK*eye(1)));
        isam_graph->slam.add_factor(scl_odo);


        if(pose_inx>74){


            //Add G as a pose factor

            GMeas_Factor *g_prior=new GMeas_Factor(new_pose,Point3d(vector_toon2eigen(frame_meas[pose_inx].GMeas)),Covariance(matrix_toon2eigen(frame_meas[pose_inx].Rg)));
            isam_graph->slam.add_factor(g_prior);
        }else{
            //std::cout <<frame_meas[pose_inx].QK/(1+util::square(frame_meas[pose_inx].K))<<" "<<delta_ang<<"\n";
        }

        //Add odometry meas
        Noise rel_noise = Information(matrix_toon2eigen(frame_meas[pose_inx].WrelPosPose));
        Pose3d delta=revboRP2isam(frame_meas[pose_inx].relPosPose,1.0);

        Scaled_Pose3d_Pose3d_Factor* odo = new Scaled_Pose3d_Pose3d_Factor(prior_pose,new_scale, new_pose, delta, rel_noise);

        isam_graph->slam.add_factor(odo);


        //is new keyframe?

        if(frame_meas[pose_inx].KF_ID!=curr_kf){

            isam_graph->kf_nodes.push_back(new_pose);

            isam_graph->kf_scale_nodes.push_back(new_scale);

            curr_kf=frame_meas[pose_inx].KF_ID;



            isam_graph->kf_pose_inx.push_back(pose_inx);


        }



        prior_pose=new_pose;
        prior_scale=new_scale;

    }

    for(int kf_i=1;kf_i<isam_graph->kf_pose_inx.size();kf_i++){


        int inx0=(isam_graph->kf_pose_inx[kf_i-1]+isam_graph->kf_pose_inx[kf_i])/2;
        int inx1=isam_graph->kf_pose_inx[kf_i];
        int inx2=((kf_i<isam_graph->kf_pose_inx.size()-1?isam_graph->kf_pose_inx[kf_i+1]:poses_num)+isam_graph->kf_pose_inx[kf_i])/2;

        double xi=smooth_scale_z[inx0].first;
        double wi=1/smooth_scale_z[inx0].second;

        for(int i=inx0+1;i<=inx1;i++){

            double wip=1/(1/wi+frame_meas[i].QK);

            wi=(wip+smooth_scale_z[i].second);
            xi=(xi*wip+smooth_scale_z[i].first*smooth_scale_z[i].second)/wi;

        }

        double xs=smooth_scale_z[inx2-1].first;
        double ws=1/smooth_scale_z[inx2-1].second;

        for(int i=inx2-2;i>inx1;i--){

            double wsp=1/(1/ws+frame_meas[i].QK);

            ws=(wsp+smooth_scale_z[i].second);
            xs=(xs*wsp+smooth_scale_z[i].first*smooth_scale_z[i].second)/ws;

        }

        double w=wi+ws;
        double x=(xi*wi+xs*ws)/w;

        std::cout <<"KF: "<<kf_i<<" I0: "<<inx0<<" I2:"<<inx2<<" K-: "<<tan(xi)<<" K+: "<<tan(xs)<<" K: "<<tan(x)<<" Ko: "<<frame_meas[isam_graph->kf_pose_inx[kf_i]].K<<" p: "<<1/w<<" po: "<<1/frame_meas[isam_graph->kf_pose_inx[kf_i]].WK<< "\n";

       /* Scale_Factor *scl_prior=new Scale_Factor(isam_graph->kf_scale_nodes[kf_i],
                                                 Point1d(tan(x)),
                                                 Information(w*eye(1)/((1+util::square(tan(x))))));*/

        Scale_Factor *scl_prior=new Scale_Factor(isam_graph->kf_scale_nodes[kf_i],
                                                 frame_meas[isam_graph->kf_pose_inx[kf_i]].K,
                                                 Information(frame_meas[isam_graph->kf_pose_inx[kf_i]].WK*eye(1)/((1+util::square(tan(x))))));
        isam_graph->slam.add_factor(scl_prior);

    }




    //isam_graph->slam.batch_optimization();

    std::ofstream pose_log("pose_log0.txt");

    if(pose_log.is_open()){
        int pose_inx=0;
        for(auto &node:isam_graph->slam.get_nodes()){

            Pose3d_Node* pose_node=dynamic_cast<Pose3d_Node*>(node);

            if(pose_node==NULL)
                continue;
            const Pose3d &pose=pose_node->value();


            Eigen::Vector3d g_vect(0,9.8,0);
            Eigen::Vector3d g_rot=pose.rot().oRw()*g_vect;
            if(pose_inx<poses_num)
                pose_log <<pose.x()<<" "<<pose.y()<<" "<<pose.z()<<" "<<g_rot(0)<<" "<<g_rot(1)<<" "<<g_rot(2)
                        <<" "<<frame_meas[pose_inx].AcelMeas[0]<<" "<<frame_meas[pose_inx].AcelMeas[1]<<" "<<frame_meas[pose_inx].AcelMeas[2]<< endl;
            pose_inx++;
        }
    }
    pose_log.close();

    std::ofstream scale_log("scale_log0.txt");

    if(scale_log.is_open()){
        for(auto &node:isam_graph->slam.get_nodes()){

            Scale_Node* scale_node=dynamic_cast<Scale_Node*>(node);

            if(scale_node==NULL)
                continue;

            scale_log <<scale_node->value().x()<< std::endl;
        }
    }
    scale_log.close();

    std::ofstream kf_log("kf_inx.txt");

    if(kf_log.is_open()){
        for(int &inx:isam_graph->kf_pose_inx){
            kf_log<<inx<<"\n";
        }
    }
    kf_log.close();

    std::cout <<"ISAM: Poses loaded\n";

}



void pose_graph::loadIsamWithScaleMonocular(){

    using namespace std;

    if(!isam_graph)
        isam_graph=new isam_data();

    Pose3d_Node* pose0 = new Pose3d_Node();
    isam_graph->slam.add_node(pose0);
    isam_graph->kf_nodes.push_back(pose0);


    Scale_Node *snode0=new Scale_Node();
    isam_graph->kf_scale_nodes.push_back(snode0);
    isam_graph->slam.add_node(snode0);
    isam_graph->kf_pose_inx.push_back(0);

    int curr_kf=0;

    // create a prior on the camera position

    Pose3d origin;
    Noise noise6 = Information(1.0* eye(6));
    Pose3d_Factor* prior = new Pose3d_Factor(pose0, origin, noise6);

    isam_graph->slam.add_factor(prior);

    // Create a prior on scale
    isam_graph->slam.add_factor(new Scale_Factor(snode0,Point1d(1),Information(1.0*eye(1))));

    int poses_num=frame_meas.size();


    cout << "Adding " << poses_num<< " poses and factors:" << endl;

    Pose3d_Node* prior_pose=pose0;
    Scale_Node * prior_scale=snode0;

    for(int pose_inx=0;pose_inx<poses_num;pose_inx++){

        //Pose Node
        Pose3d_Node* new_pose = new Pose3d_Node();
        isam_graph->slam.add_node(new_pose);

        //Scale Node
        Scale_Node *new_scale=new Scale_Node();
        isam_graph->slam.add_node(new_scale);

        //Add scale - scale factor

        double delta_K=frame_meas[pose_inx].K/(pose_inx>0?frame_meas[pose_inx-1].K:1);
        Scale_Scale_Factor *scl_odo=new Scale_Scale_Factor(prior_scale,new_scale,delta_K,
                                                                       Covariance(frame_meas[pose_inx].QK*eye(1)));

        isam_graph->slam.add_factor(scl_odo);



        //Add odometry meas
        Noise rel_noise = Information(matrix_toon2eigen(frame_meas[pose_inx].WrelPosPose));
        Pose3d delta=revboRP2isam(frame_meas[pose_inx].relPosPose,1.0);

        Scaled_Pose3d_Pose3d_Factor* odo = new Scaled_Pose3d_Pose3d_Factor(prior_pose,new_scale, new_pose, delta, rel_noise);

        isam_graph->slam.add_factor(odo);


        //is new keyframe?

        if(frame_meas[pose_inx].KF_ID!=curr_kf){

            isam_graph->kf_nodes.push_back(new_pose);

            isam_graph->kf_scale_nodes.push_back(new_scale);

            curr_kf=frame_meas[pose_inx].KF_ID;

            isam_graph->kf_pose_inx.push_back(pose_inx);

/*
            Scale_Factor *scl_prior=new Scale_Factor(new_scale,
                                                     1,
                                                     Information(1e6*eye(1)));

            isam_graph->slam.add_factor(scl_prior);
*/
        }



        prior_pose=new_pose;
        prior_scale=new_scale;

    }



    //isam_graph->slam.batch_optimization();

    std::ofstream pose_log("pose_log0.txt");

    if(pose_log.is_open()){
        int pose_inx=0;
        for(auto &node:isam_graph->slam.get_nodes()){

            Pose3d_Node* pose_node=dynamic_cast<Pose3d_Node*>(node);

            if(pose_node==NULL)
                continue;
            const Pose3d &pose=pose_node->value();


            Eigen::Vector3d g_vect(0,9.8,0);
            Eigen::Vector3d g_rot=pose.rot().oRw()*g_vect;
            if(pose_inx<poses_num)
                pose_log <<pose.x()<<" "<<pose.y()<<" "<<pose.z()<<" "<<g_rot(0)<<" "<<g_rot(1)<<" "<<g_rot(2)
                        <<" "<<frame_meas[pose_inx].AcelMeas[0]<<" "<<frame_meas[pose_inx].AcelMeas[1]<<" "<<frame_meas[pose_inx].AcelMeas[2]<< endl;
            pose_inx++;
        }
    }
    pose_log.close();

    std::ofstream scale_log("scale_log0.txt");

    if(scale_log.is_open()){
        for(auto &node:isam_graph->slam.get_nodes()){

            Scale_Node* scale_node=dynamic_cast<Scale_Node*>(node);

            if(scale_node==NULL)
                continue;

            scale_log <<scale_node->value().x()<< std::endl;
        }
    }
    scale_log.close();

    std::ofstream kf_log("kf_inx.txt");

    if(kf_log.is_open()){
        for(int &inx:isam_graph->kf_pose_inx){
            kf_log<<inx<<"\n";
        }
    }
    kf_log.close();

    std::cout <<"ISAM: Poses loaded\n";

}


void pose_graph::loadIsamWithScale(){
    using namespace std;

    if(!isam_graph)
        isam_graph=new isam_data();

    Properties prop = isam_graph->slam.properties();

    prop.method = DOG_LEG;
    prop.max_iterations=50;
    prop.mod_solve=1;
    prop.mod_update=1;
    isam_graph->slam.set_properties(prop);
    //isam_graph->slam.set_cost_function(robust_cost_function);

    // optimize



    Pose3d_Node* pose0 = new Pose3d_Node();
    isam_graph->slam.add_node(pose0);
    isam_graph->kf_nodes.push_back(pose0);


    Scale_Node *snode0=new Scale_Node();
    isam_graph->kf_scale_nodes.push_back(snode0);
    isam_graph->slam.add_node(snode0);

    int curr_kf=0;

    // create a prior on the camera position

    TooN::Vector <6>p0=TooN::Zeros;
    p0.slice<3,3>()=TooN::SO3<>(TooN::unit(-frame_meas[0].AcelMeas),TooN::makeVector(0,1,0)).ln();

    Pose3d origin=revboRP2isam(p0,1);
    Noise noise6 = Information(1.0* eye(6));
    Pose3d_Factor* prior = new Pose3d_Factor(pose0, origin, noise6);

    isam_graph->slam.add_factor(prior);

    // Create a prior on scale
    isam_graph->slam.add_factor(new Scale_Factor(snode0,Point1d(1),Information(1.0e-6*eye(1))));

    // Create a bias node

    Point3d_Node *bias0=new Point3d_Node();
    isam_graph->slam.add_node(bias0);

    // Add a prior on bias
    Point3d_Factor *bias_prior_factor=new Point3d_Factor(bias0,Point3d(),Information(1.0e-6*eye(3)));
    isam_graph->slam.add_factor(bias_prior_factor);

    // Add prior on GMod

    Scale_Node * GMod=new Scale_Node();
    isam_graph->slam.add_node(GMod);
    isam_graph->slam.add_factor(new Scale_Factor(GMod,Point1d(9.8),Covariance(0.1*0.1*eye(1))));


    int poses_num=frame_meas.size();


    cout << "Adding " << poses_num<< " poses and factors:" << endl;

    Pose3d_Node* prior_pose=pose0;
    Scale_Node * prior_scale=snode0;

    Pose3d_Node * prior_prior_pose=NULL;

    Point3d_Node *prior_bias=bias0;

    //isam_graph->slam.batch_optimization();
    //isam_graph->slam.update();

    for(int pose_inx=0;pose_inx<poses_num;pose_inx++){

        //Pose Node
        Pose3d_Node* new_pose = new Pose3d_Node();
        isam_graph->slam.add_node(new_pose);

        //Scale Node
        Scale_Node *new_scale=new Scale_Node();
        isam_graph->slam.add_node(new_scale);

        //Add scale - scale factor

        Scale_Scale_Factor *scl_odo=new Scale_Scale_Factor(prior_scale,new_scale,
                                                           1,//(pose_inx>0?frame_meas[pose_inx-1].K:1)/frame_meas[pose_inx].K,
                                                                       Information(frame_meas[pose_inx].WK*eye(1)));
        isam_graph->slam.add_factor(scl_odo);

        //Add odometry meas
        Noise rel_noise = Information(matrix_toon2eigen(frame_meas[pose_inx].WrelPosPose));
        Pose3d delta=revboRP2isam(frame_meas[pose_inx].relPosPose,1.0);

        //cout <<"Odometry Factor KF:"<<frame_meas[pose_inx].KF_ID<<" Delta: " <<delta<<" WK:"<<frame_meas[pose_inx].WK<<" Acel:"<<frame_meas[pose_inx].AcelMeas  << endl;

        Scaled_Pose3d_Pose3d_Factor* odo = new Scaled_Pose3d_Pose3d_Factor(prior_pose,new_scale, new_pose, delta, rel_noise);

        isam_graph->slam.add_factor(odo);

        //Add bias node
        Point3d_Node *new_bias=new Point3d_Node();
        isam_graph->slam.add_node(new_bias);

        Point3d_Point3d_Factor *bias_bias_factor=new Point3d_Point3d_Factor(prior_bias,new_bias,Point3d(),Covariance(eye(3)*1e-14));
        isam_graph->slam.add_factor(bias_bias_factor);

        if(prior_prior_pose){

            //Add accelerometer

            Point3d acel_meas(frame_meas[pose_inx].AcelMeas[0],frame_meas[pose_inx].AcelMeas[1],frame_meas[pose_inx].AcelMeas[2]);
            Noise acel_noise=Covariance(eye(3)*2.0000e-3*2.0000e-3);
            double Ts=0.05;
            Acelerometer_Factor *acel_odo=new Acelerometer_Factor(prior_prior_pose,prior_pose,new_pose,new_bias,acel_meas,acel_noise,Ts,GMod);
            isam_graph->slam.add_factor(acel_odo);

        }


        //is new keyframe?

        if(frame_meas[pose_inx].KF_ID!=curr_kf){

            isam_graph->kf_nodes.push_back(new_pose);

            isam_graph->kf_scale_nodes.push_back(new_scale);

            curr_kf=frame_meas[pose_inx].KF_ID;

        }



        //cout <<"OdoErr:" << odo->basic_error()<<" Scale Err:"<< scl_odo->basic_error()<< endl;
        prior_prior_pose=prior_pose;
        prior_pose=new_pose;
        prior_scale=new_scale;
        prior_bias=new_bias;


        if(pose_inx==400){
            util::timer tproc;
            tproc.start();
            isam_graph->slam.update();
            isam_graph->slam.batch_optimization();
            std::cout << "batch time: "<<tproc.stop()<<"\n";
        }else if(pose_inx>400){
            util::timer tproc;
            tproc.start();
            isam_graph->slam.update();
            std::cout << "update time: "<<tproc.stop()<<"\n";
        }
    }




    //isam_graph->slam.batch_optimization();

    std::ofstream pose_log("pose_log0.txt");

    if(pose_log.is_open()){
        int pose_inx=0;
        for(auto &node:isam_graph->slam.get_nodes()){

            Pose3d_Node* pose_node=dynamic_cast<Pose3d_Node*>(node);

            if(pose_node==NULL)
                continue;
            const Pose3d &pose=pose_node->value();


            Eigen::Vector3d g_vect(0,9.8,0);
            Eigen::Vector3d g_rot=pose.rot().oRw()*g_vect;
            if(pose_inx<poses_num)
                pose_log <<pose.x()<<" "<<pose.y()<<" "<<pose.z()<<" "<<g_rot(0)<<" "<<g_rot(1)<<" "<<g_rot(2)
                        <<" "<<frame_meas[pose_inx].AcelMeas[0]<<" "<<frame_meas[pose_inx].AcelMeas[1]<<" "<<frame_meas[pose_inx].AcelMeas[2]<< endl;
            pose_inx++;
        }
    }
    pose_log.close();

    std::ofstream scale_log("scale_log0.txt");

    if(scale_log.is_open()){
        for(auto &node:isam_graph->slam.get_nodes()){

            Scale_Node* scale_node=dynamic_cast<Scale_Node*>(node);

            if(scale_node==NULL)
                continue;

            scale_log <<scale_node->value().x()<< std::endl;
        }
    }
    scale_log.close();

    std::cout <<"ISAM: Poses loaded\n";

}




void pose_graph::loadIsamWithScaleDecoupled(){
    using namespace std;

    if(!isam_graph)
        isam_graph=new isam_data();


    Pose3d_Node* pose0 = new Pose3d_Node();
    isam_graph->slam.add_node(pose0);
    isam_graph->kf_nodes.push_back(pose0);


    Scale_Node *snode0=new Scale_Node();
    isam_graph->kf_scale_nodes.push_back(snode0);
    isam_graph->slam.add_node(snode0);

    int curr_kf=0;

    // create a prior on the camera position

    TooN::Vector <6>p0=TooN::Zeros;
    p0.slice<3,3>()=TooN::SO3<>(TooN::unit(-frame_meas[0].AcelMeas),TooN::makeVector(0,1,0)).ln();

    Pose3d origin=revboRP2isam(p0,1);
    Noise noise6 = Information(1.0 * eye(6));
    Pose3d_Factor* prior = new Pose3d_Factor(pose0, origin, noise6);

    isam_graph->slam.add_factor(prior);

    // Create a prior on scale
    isam_graph->slam.add_factor(new Scale_Factor(snode0,Point1d(1),Information(1.0e0*eye(1))));


    // Create a bias node

    Point3d_Node *bias0=new Point3d_Node();
    isam_graph->slam.add_node(bias0);

    // Add a prior on bias
    Point3d_Factor *bias_prior_factor=new Point3d_Factor(bias0,Point3d(),Information(1.0e0*eye(3)));
    isam_graph->slam.add_factor(bias_prior_factor);



    int poses_num=frame_meas.size();


    cout << "Adding " << poses_num<< " poses and factors:" << endl;

    Pose3d_Node* prior_pose=pose0;
    Scale_Node * prior_scale=snode0;
    Point3d_Node *prior_bias=bias0;



    for(int pose_inx=0;pose_inx<poses_num;pose_inx++){

        //Pose Node
        Pose3d_Node* new_pose = new Pose3d_Node();
        isam_graph->slam.add_node(new_pose);

        //Scale Node
        Scale_Node *new_scale=new Scale_Node();
        isam_graph->slam.add_node(new_scale);

        new_scale->init(1.0*frame_meas[pose_inx].K);
        //Add scale - scale factor

        Scale_Scale_Factor *scl_odo=new Scale_Scale_Factor(prior_scale,new_scale,
                                                           1,//(pose_inx>0?frame_meas[pose_inx-1].K:1)-frame_meas[pose_inx].K,
                                                                       Information(frame_meas[pose_inx].WK*eye(1)));
        isam_graph->slam.add_factor(scl_odo);


        //Add odometry meas
        Noise rel_noise = Information(matrix_toon2eigen(frame_meas[pose_inx].WrelPosPose));
        Pose3d delta=revboRP2isam(frame_meas[pose_inx].relPosPose,1.0);

        //cout <<"Odometry Factor KF:"<<frame_meas[pose_inx].KF_ID<<" Delta: " <<delta<<" WK:"<<frame_meas[pose_inx].WK<<" Acel:"<<frame_meas[pose_inx].AcelMeas  << endl;

        Scaled_Pose3d_Pose3d_Factor* odo = new Scaled_Pose3d_Pose3d_Factor(prior_pose,new_scale, new_pose, delta, rel_noise);

        isam_graph->slam.add_factor(odo);

        //Add bias node
        Point3d_Node *new_bias=new Point3d_Node();
        isam_graph->slam.add_node(new_bias);

        Point3d_Point3d_Factor *bias_bias_factor=new Point3d_Point3d_Factor(prior_bias,new_bias,Point3d(),Covariance(eye(3)*1e-14));
        isam_graph->slam.add_factor(bias_bias_factor);



        if(pose_inx>0){

            //Add visual acel node
            Point3d_Node *new_visual_acel=new Point3d_Node();
            isam_graph->slam.add_node(new_visual_acel);

            double GMod=9.8;
            double Ts=0.05;

            TooN::SO3 <>back_rot(frame_meas[pose_inx].relPosPose.slice<3,3>());
            TooN::Vector <3> vis_acel=(frame_meas[pose_inx].relPosPose.slice<0,3>()-back_rot.get_matrix().T()*frame_meas[pose_inx-1].relPosPose.slice<0,3>())*(1/Ts/Ts);

            Point3d_Factor *visual_acel_factor=new Point3d_Factor(new_visual_acel,Point3d(vis_acel[0],vis_acel[1],vis_acel[2]),
                    Information(matrix_toon2eigen(frame_meas[pose_inx].WrelPosPose.slice<0,0,3,3>())*Ts*Ts*Ts*Ts));
            isam_graph->slam.add_factor(visual_acel_factor);




            //Add accelerometer


            Point3d acel_meas(frame_meas[pose_inx].AcelMeas[0],frame_meas[pose_inx].AcelMeas[1],frame_meas[pose_inx].AcelMeas[2]);
            Noise acel_noise=Covariance(eye(3)*2.0000e-2*2.0000e-2);

            Acelerometer_Factor_Dec *acel_odo=new Acelerometer_Factor_Dec(new_pose,new_visual_acel,new_scale,new_bias,acel_meas,acel_noise,GMod);
            isam_graph->slam.add_factor(acel_odo);

        }


        //is new keyframe?

        if(frame_meas[pose_inx].KF_ID!=curr_kf){

            isam_graph->kf_nodes.push_back(new_pose);

            isam_graph->kf_scale_nodes.push_back(new_scale);

            curr_kf=frame_meas[pose_inx].KF_ID;

        }


        prior_pose=new_pose;
        prior_scale=new_scale;
        prior_bias=new_bias;

       // isam_graph->slam.incremental_update();
    }




    //isam_graph->slam.batch_optimization();

    std::ofstream pose_log("pose_log0.txt");

    if(pose_log.is_open()){
        int pose_inx=0;
        for(auto &node:isam_graph->slam.get_nodes()){

            Pose3d_Node* pose_node=dynamic_cast<Pose3d_Node*>(node);

            if(pose_node==NULL)
                continue;
            const Pose3d &pose=pose_node->value();


            Eigen::Vector3d g_vect(0,9.8,0);
            Eigen::Vector3d g_rot=pose.rot().oRw()*g_vect;
            if(pose_inx<poses_num)
                pose_log <<pose.x()<<" "<<pose.y()<<" "<<pose.z()<<" "<<g_rot(0)<<" "<<g_rot(1)<<" "<<g_rot(2)
                        <<" "<<frame_meas[pose_inx].AcelMeas[0]<<" "<<frame_meas[pose_inx].AcelMeas[1]<<" "<<frame_meas[pose_inx].AcelMeas[2]<< endl;
            pose_inx++;
        }
    }
    pose_log.close();

    std::ofstream scale_log("scale_log0.txt");

    if(scale_log.is_open()){
        for(auto &node:isam_graph->slam.get_nodes()){

            Scale_Node* scale_node=dynamic_cast<Scale_Node*>(node);

            if(scale_node==NULL)
                continue;

            scale_log <<scale_node->value().x()<< std::endl;
        }
    }
    scale_log.close();

    std::cout <<"ISAM: Poses loaded\n";

}


void pose_graph::addOdoMeasWithScale(TooN::Vector <6> relPose, TooN::Matrix<6, 6> relPosCov,int kf_from,int kf_to)
{


    if(kf_from<isam_graph->kf_nodes.size() && kf_to<isam_graph->kf_nodes.size() ){
        Noise rel_noise = Covariance(matrix_toon2eigen(relPosCov));
        Pose3d delta=revboRP2isam(relPose,1);

        Scaled_Pose3d_Pose3d_Factor* odo = new Scaled_Pose3d_Pose3d_Factor(isam_graph->kf_nodes[kf_to], isam_graph->kf_scale_nodes[kf_from], isam_graph->kf_nodes[kf_from], delta, rel_noise);

        //Push factor
        isam_graph->slam.add_factor(odo);
    }else{
        std::cout<<"\n pose_graph::addOdoMeasWithScale(): kf out of size!";
    }
}


/**
 * @brief pose_graph::addScaleOdometry
 *
 * Adds a contraint in the form scale(kf_to)=scale(kf_from)*Kp
 *
 * @param Kr
 * @param WKr
 * @param kf_from
 * @param kf_to
 */
void pose_graph::addScaleOdometry(double Kr,double WKr,int kf_from,int kf_to)
{


    if(kf_from<isam_graph->kf_nodes.size() && kf_to<isam_graph->kf_nodes.size() ){

        Scale_Scale_Factor* odo = new Scale_Scale_Factor(isam_graph->kf_scale_nodes[kf_from], isam_graph->kf_scale_nodes[kf_to], Kr, Information(WKr*eye(1)));

        //Push factor
        isam_graph->slam.add_factor(odo);
    }else{
        std::cout<<"\n pose_graph::addScaleOdometry(): kf out of size!";
    }
}

void pose_graph::addOdoMeasWithAngScale(TooN::Vector <6> relPose, TooN::Matrix<6, 6> relPosCov,int kf_from,int kf_to)
{


    if(kf_from<isam_graph->kf_nodes.size() && kf_to<isam_graph->kf_nodes.size() ){
        Noise rel_noise = Covariance(matrix_toon2eigen(relPosCov));
        Pose3d delta=revboRP2isam(relPose,1);

        AngularScaled_Pose3d_Pose3d_Factor* odo = new AngularScaled_Pose3d_Pose3d_Factor(isam_graph->kf_nodes[kf_to], isam_graph->kf_scale_nodes[kf_from], isam_graph->kf_nodes[kf_from], delta, rel_noise);

        //Push factor
        isam_graph->slam.add_factor(odo);
    }else{
        std::cout<<"\n pose_graph::addOdoMeasWithScale(): kf out of size!";
    }
}


double robust_cost_function(double d) {
  return cost_pseudo_huber(d, .5);
}


void pose_graph::optimizeAndPrint(bool update)
{


    Properties prop = isam_graph->slam.properties();

    prop.method = DOG_LEG;
    prop.max_iterations=50;
    isam_graph->slam.set_properties(prop);

    if(update)
        isam_graph->slam.update();
    else
        isam_graph->slam.batch_optimization();

    std::ofstream pose_log("pose_log1.txt");

    if(pose_log.is_open()){
        for(auto &node:isam_graph->slam.get_nodes()){

            Pose3d_Node* pose_node=dynamic_cast<Pose3d_Node*>(node);

            if(pose_node==NULL)
                continue;
            const Pose3d &pose=pose_node->value();

            pose_log <<pose.x()<<" "<<pose.y()<<" "<<pose.z()<< std::endl;
        }
    }
    pose_log.close();

    std::ofstream scale_log("scale_log1.txt");

    if(scale_log.is_open()){
        for(auto &node:isam_graph->slam.get_nodes()){

            Scale_Node* scale_node=dynamic_cast<Scale_Node*>(node);

            if(scale_node==NULL)
                continue;

            scale_log <<scale_node->value().x()<< std::endl;
        }
    }
    scale_log.close();

}

void pose_graph::getKfPose(int kf_id, TooN::Matrix<3, 3> &Pose, TooN::Vector<3> &Pos,double &K)
{
    if(kf_id<isam_graph->kf_nodes.size() ){
        const Pose3d &pose=isam_graph->kf_nodes[kf_id]->value();
        TooN::Matrix<4,4> t44=matrix_eigen2toon(pose.wTo());

        Pose=t44.slice<0,0,3,3>();
        Pos=t44.T()[3].slice<0,3>();

        K=isam_graph->kf_scale_nodes[kf_id]->value().x();
    }else{
        std::cout<<"\n pose_graph::get_kf_pose(): kf out of size!";
    }

}


}
