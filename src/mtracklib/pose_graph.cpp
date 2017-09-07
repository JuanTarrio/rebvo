#include "isam/isam.h"
#include "mtracklib/pose_graph.h"
#include <TooN/so3.h>
#include <TooN/Cholesky.h>
namespace  rebvo {
using namespace isam;

class Point1d {
  friend std::ostream& operator<<(std::ostream& out, const Point1d& p) {
    p.write(out);
    return out;
  }

  double _x;
public:
  static const int dim = 1;
  static const char* name() {
    return "Point1d";
  }
  Point1d() : _x(0.){}
  Point1d(double x) : _x(x) {}
  Point1d(const Eigen::Vector1d& vec) : _x(vec(0)) {}

  double x() const {return _x;}

  void set_x(double x) {_x = x;}

  Point1d exmap(const Eigen::Vector1d& delta) const {
    Point1d res = *this;
    res._x += delta(0);
    return res;
  }

  Eigen::Vector1d vector() const {
    Eigen::Vector1d tmp;
    tmp(0)=_x;
    return tmp;
  }
  void set(double x) {
    _x = x;
  }
  void set(const Eigen::Vector1d& v) {
    _x = v(0);
  }

  Eigen::VectorXb is_angle() const {
    return Eigen::VectorXb::Zero(dim);
  }

  Point1d to_point1d() {
    return *this;
  }


  void write(std::ostream &out) const {
    out << "(" << _x  << ")";
  }
};



typedef NodeT<Point1d> Scale_Node;



class Scale_Factor : public FactorT<Point1d> {
  Scale_Node* _scale;

public:

  /**
   * Constructor.
   * @param pose The scale node the prior acts on.
   * @param prior The actual prior measurement.
   * @param noise The 1x1 square root information matrix (upper triangular).
   */
  Scale_Factor(Scale_Node* scale, const Point1d& prior, const Noise& noise)
    : FactorT<Point1d>("Scale_Factor", 1, noise, prior), _scale(scale) {
    _nodes.resize(1);
    _nodes[0] = scale;
  }

  void initialize() {
    if (!_scale->initialized()) {
      Point1d predict = _measure;
      _scale->init(predict);
    }
  }

  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {

    Eigen::Vector1d err = _nodes[0]->vector(s) - _measure.vector();
    return err;
  }
};


class Scaled_Pose3d_Pose3d_Factor : public FactorT<Pose3d> {
  Pose3d_Node* _pose1;
  Scale_Node* _scale;
  Pose3d_Node* _pose2;

public:

  /**
   * Constructor.
   * @param pose1 The pose from which the measurement starts.
   * @param pose2 The pose to which the measurement extends.
   * @param measure The relative measurement from pose1 to pose2 (pose2 in pose1's frame).
   * @param noise The 6x6 square root information matrix (upper triangular).
   * @param anchor1 Optional anchor node for trajectory to which pose1 belongs to.
   * @param anchor2 Optional anchor node for trajectory to which pose2 belongs to.
   */
  Scaled_Pose3d_Pose3d_Factor(Pose3d_Node* pose1,Scale_Node* scale, Pose3d_Node* pose2,
      const Pose3d& measure, const Noise& noise)
    : FactorT<Pose3d>("Scaled_Pose3d_Pose3d_Factor", 6, noise, measure), _pose1(pose1),_scale(scale), _pose2(pose2) {


    _nodes.resize(3);

    _nodes[0] = pose1;
    _nodes[1] = pose2;
    _nodes[2] = scale;
  }

  static Pose3d scalePose(const Pose3d &delta,double scale){


      return Pose3d(delta.x()*scale,delta.y()*scale,delta.z()*scale,delta.yaw(),delta.pitch(),delta.roll());

  }

  void initialize() {
    require((_pose1->initialized() || _pose2->initialized())&&_scale->initialized(),
        "slam3d: ScaledPose3d_Pose3d_Factor requires pose1 or pose2 and scale to be initialized ");

    if (!_pose1->initialized() && _pose2->initialized()) {
      // Reverse constraint
      Pose3d p2 = _pose2->value();
      Pose3d z;
      Pose3d predict = p2.oplus(z.ominus(scalePose(_measure,_scale->value().x())));
      _pose1->init(predict);
    } else if (_pose1->initialized() && !_pose2->initialized()) {
      Pose3d p1 = _pose1->value();
      Pose3d predict = p1.oplus(scalePose(_measure,_scale->value().x()));
      _pose2->init(predict);
    }

  }


  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
    const Pose3d& p1 = _pose1->value(s);
    const Pose3d& p2 = _pose2->value(s);
    Pose3d predicted;

    predicted = p2.ominus(p1);



    Eigen::VectorXd err = scalePose(predicted,1.0/_scale->value(s).x()).vector() - _measure.vector();


    err(3) = standardRad(err(3));
    err(4) = standardRad(err(4));
    err(5) = standardRad(err(5));
    return err;
  }

};


struct isam_data{
    Slam slam;


    std::vector <Pose3d_Node*> kf_nodes;
    std::vector <Scale_Node*> kf_scale_nodes;
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

Pose3d revboRP2isam(TooN::Vector <6> relPos,double K){

    TooN::Matrix <4,4> T=TooN::Zeros;

    TooN::Matrix<3,3> R=TooN::SO3<>(relPos.slice<3,3>()).get_matrix();

    T(3,3)=1;
    T.slice<0,0,3,3>()=R;

    T.T()[3].slice<0,3>()=relPos.slice<0,3>()*K;
    //std::cout << relPos<<"\n";

    Eigen::Matrix<double,4,4> Te;
    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++)
            Te(i,j)=T(i,j);
    //std::cout << Te<<"\n";
    return Pose3d(Te);

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


Eigen::MatrixXd matrix_toon2eigen(TooN::Matrix <TooN::Dynamic,TooN::Dynamic> m){

    Eigen::MatrixXd me(m.num_rows(),m.num_cols());
    for(int i=0;i<m.num_rows();i++)
        for(int j=0;j<m.num_cols();j++){
            me(i,j)=m(i,j);
        }

    return me;
}

TooN::Matrix <TooN::Dynamic,TooN::Dynamic> matrix_eigen2toon(Eigen::MatrixXd m){

    TooN::Matrix <TooN::Dynamic,TooN::Dynamic> me(m.rows(),m.cols());
    for(int i=0;i<m.rows();i++)
        for(int j=0;j<m.cols();j++){
            me(i,j)=m(i,j);
        }

    return me;
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
                                                     Information(frame_meas[pose_inx].WK*eye(1)));
            isam_graph->slam.add_factor(scl_prior);

        }



       // cout <<"Pose Value:" << new_pose->value() << endl;
        prior_pose=new_pose;
    }



    isam_graph->slam.batch_optimization();

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

void pose_graph::addOdoMeasScaled(TooN::Vector <6> relPose, TooN::Matrix<6, 6> relPosCov, double K,int kf_from,int kf_to)
{



    if(kf_from<isam_graph->kf_nodes.size() && kf_to<isam_graph->kf_nodes.size() ){

        Noise rel_noise = Covariance(matrix_toon2eigen(scaleCov(relPosCov,K)));
        Pose3d delta=revboRP2isam(relPose,K);

        Pose3d_Pose3d_Factor* odo = new Pose3d_Pose3d_Factor(isam_graph->kf_nodes[kf_to], isam_graph->kf_nodes[kf_from], delta, rel_noise);

        //Push factor
        isam_graph->slam.add_factor(odo);
    }else{
        std::cout<<"\n pose_graph::addOdoMeasScaled(): kf out of size!";
    }
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

void pose_graph::optimizeAndPrint()
{


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
