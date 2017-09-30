#ifndef ISAM_DEFS_H
#define ISAM_DEFS_H
#include "isam/isam.h"
#include <TooN/TooN.h>
#include <TooN/so3.h>
namespace  isam {

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
      Pose3d predict = p2.oplus(z.ominus(scalePose(_measure,1.0*_scale->value().x())));
      _pose1->init(predict);
    } else if (_pose1->initialized() && !_pose2->initialized()) {
      Pose3d p1 = _pose1->value();
      Pose3d predict = p1.oplus(scalePose(_measure,1.0*_scale->value().x()));
      _pose2->init(predict);
    }

  }


  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
    const Pose3d& p1 = _pose1->value(s);
    const Pose3d& p2 = _pose2->value(s);
    Pose3d predicted;

    predicted = p2.ominus(p1);



    Eigen::VectorXd err = scalePose(predicted,1/_scale->value(s).x()).vector() - _measure.vector();
    //Eigen::VectorXd err = predicted.vector() - _measure.vector();


    err(3) = standardRad(err(3));
    err(4) = standardRad(err(4));
    err(5) = standardRad(err(5));
    return err;
  }

};



class AngularScale_Factor : public FactorT<Point1d> {
  Scale_Node* _scale;

  Eigen::Vector3d _Av;
  Eigen::Vector3d _As;
  Eigen::Vector3d _G;
  Eigen::Matrix3d _Rv;
  Eigen::Matrix3d _Rs;

public:

  /**
   * Constructor.
   * @param pose The scale node the prior acts on.
   * @param prior The actual prior measurement.
   * @param noise The 1x1 square root information matrix (upper triangular).
   */
  AngularScale_Factor(Scale_Node* scale, const Eigen::Vector3d& Av, const Eigen::Vector3d& As, const Eigen::Vector3d& G,Eigen::Matrix3d Rv,Eigen::Matrix3d Rs, const Point1d& prior, const Noise& noise)
    : FactorT<Point1d>("Scale_Factor", 1, noise, prior), _scale(scale),_Av(Av),_As(As),_G(G),_Rv(Rv),_Rs(Rs) {
    _nodes.resize(1);
    _nodes[0] = scale;
  }

  void initialize() {
    /*if (!_scale->initialized()) {
      Point1d predict;
      predict.set_x(atan(As+G));
      _scale->init(predict);
    }*/
  }

  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {

    double ang=_scale->value(s).x();
    Eigen::Vector3d err3= (_As+_G)*cos(ang)-_Av*sin(ang);

    Eigen::Matrix3d R=sin(ang)*sin(ang)*_Rv+cos(ang)*cos(ang)*_Rs;

    Eigen::Vector1d err;
    err[0]=sqrt(err3.transpose()*(R.inverse())*err3);
    return err;
  }
};

class AngularScaled_Pose3d_Pose3d_Factor : public FactorT<Pose3d> {
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
   */
  AngularScaled_Pose3d_Pose3d_Factor(Pose3d_Node* pose1,Scale_Node* scale, Pose3d_Node* pose2,
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
      Pose3d predict = p2.oplus(z.ominus(scalePose(_measure,1.0*tan(_scale->value().x()))));
      _pose1->init(predict);
    } else if (_pose1->initialized() && !_pose2->initialized()) {
      Pose3d p1 = _pose1->value();
      Pose3d predict = p1.oplus(scalePose(_measure,1.0*tan(_scale->value().x())));
      _pose2->init(predict);
    }

  }


  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
    const Pose3d& p1 = _pose1->value(s);
    const Pose3d& p2 = _pose2->value(s);
    Pose3d predicted;

    predicted = p2.ominus(p1);



    Eigen::VectorXd err = scalePose(predicted,1/tan(_scale->value(s).x())).vector() - _measure.vector();


    err(3) = standardRad(err(3));
    err(4) = standardRad(err(4));
    err(5) = standardRad(err(5));
    return err;
  }

};



class Point3d_Factor : public FactorT<Point3d> {
  Point3d_Node* _point;

public:

  /**
   * Constructor.
   * @param point The point node the prior acts on.
   * @param prior The actual prior measurement.
   * @param noise The 2x2 square root information matrix (upper triangular).
   */
  Point3d_Factor(Point3d_Node* point, const Point3d& prior, const Noise& noise)
    : FactorT<Point3d>("Point3d_Factor", 3, noise, prior), _point(point) {
    _nodes.resize(1);
    _nodes[0] = point;
  }

  void initialize() {
    if (!_point->initialized()) {
      Point3d predict = _measure;
      _point->init(predict);
    }
  }

  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
    return (_point->vector(s) - _measure.vector());
  }

};


class Point3d_Point3d_Factor : public FactorT<Point3d> {
  Point3d_Node* _point1;
  Point3d_Node* _point2;

public:

  /**
   * Constructor.
   * @param scale1 The first scale.
   * @param scale2 The second scale.
   * @param measure The relative measurement between scales.
   * @param noise The 1x1 square root information matrix (upper triangular).
   */
  Point3d_Point3d_Factor(Point3d_Node* point1,Point3d_Node* point2,
      const Point3d& measure, const Noise& noise)
    : FactorT<Point3d>("Point3d_Point3d_Factor",3, noise, measure), _point1(point1),_point2(point2) {


    _nodes.resize(2);

    _nodes[0] = _point1;
    _nodes[1] = _point2;
  }


  void initialize() {
    require((_point1->initialized() || _point2->initialized()),
        "slam3d: Point3d_Point3d_Factor requires point1 or point2  to be initialized ");

    if (!_point1->initialized()) {
      // Reverse constraint

      _point1->init(Point3d(_point2->vector()-_measure.vector()));
    } else if ( !_point2->initialized()) {
      _point2->init(Point3d(_point1->vector()+_measure.vector()));
    }

  }


  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {


    Eigen::VectorXd err = (_point2->vector(s)-(_point1->vector(s))) - _measure.vector();

    //Eigen::VectorXd err = _scale2->vector(s) - _measure.vector()*_scale1->vector(s);

    return err;
  }

};


class Scale_Scale_Factor : public FactorT<Point1d> {
  Scale_Node* _scale1;
  Scale_Node* _scale2;

public:

  /**
   * Constructor.
   * @param scale1 The first scale.
   * @param scale2 The second scale.
   * @param measure The relative measurement between scales.
   * @param noise The 1x1 square root information matrix (upper triangular).
   */
  Scale_Scale_Factor(Scale_Node* scale1,Scale_Node* scale2,
      const Point1d& measure, const Noise& noise)
    : FactorT<Point1d>("Scale_Scale_Factor", 1, noise, measure), _scale1(scale1),_scale2(scale2) {


    _nodes.resize(2);

    _nodes[0] = scale1;
    _nodes[1] = scale2;
  }


  void initialize() {
    require((_scale1->initialized() || _scale2->initialized()),
        "slam3d: Scale_Scale_Factor requires scale1 or scale2  to be initialized ");

    if (!_scale1->initialized()) {
      // Reverse constraint

      _scale1->init(Point1d(_scale2->value().x()/_measure.x()));
    } else if ( !_scale2->initialized()) {
      _scale2->init(Point1d(_scale1->value().x()*_measure.x()));
    }

  }


  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {

    Point1d predic(_scale2->value(s).x()/(_scale1->value(s).x()));

    Eigen::VectorXd err = predic.vector() - _measure.vector();

    //Eigen::VectorXd err = _scale2->vector(s) - _measure.vector()*_scale1->vector(s);

    return err;
  }

};

class Scale_Scale_Angular_Factor : public FactorT<Point1d> {
  Scale_Node* _scale1;
  Scale_Node* _scale2;

public:

  /**
   * Constructor.
   * @param scale1 The first scale.
   * @param scale2 The second scale.
   * @param measure The relative measurement between scales.
   * @param noise The 1x1 square root information matrix (upper triangular).
   */
  Scale_Scale_Angular_Factor(Scale_Node* scale1,Scale_Node* scale2,
      const Point1d& measure, const Noise& noise)
    : FactorT<Point1d>("Scale_Scale_Factor", 1, noise, measure), _scale1(scale1),_scale2(scale2) {


    _nodes.resize(2);

    _nodes[0] = scale1;
    _nodes[1] = scale2;
  }


  void initialize() {
    require((_scale1->initialized() || _scale2->initialized()),
        "slam3d: Scale_Scale_Factor requires scale1 or scale2  to be initialized ");

    if (!_scale1->initialized()) {
      // Reverse constraint

      _scale1->init(Point1d(_scale2->value().x()-_measure.x()));
    } else if ( !_scale2->initialized()) {
      _scale2->init(Point1d(_scale1->value().x()+_measure.x()));
    }

  }


  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {

    double ang_err=_scale2->value(s).x()-(_scale1->value(s).x());

    if(ang_err>M_PI)                               //Error correction on the angle (cyclicity)
        ang_err-=2*M_PI;
    else if(ang_err<-M_PI)
        ang_err+=2*M_PI;

    Eigen::VectorXd err =  - _measure.vector();
    err[0]+=ang_err;

    //Eigen::VectorXd err = _scale2->vector(s) - _measure.vector()*_scale1->vector(s);

    return err;
  }

};

class GMeas_Factor : public FactorT<Point3d> {
  Pose3d_Node* _pose;

public:

  /**
   * Constructor.
   * @param pose1 The pose from which the measurement starts.
   * @param pose2 The pose to which the measurement extends.
   * @param measure The relative measurement from pose1 to pose2 (pose2 in pose1's frame).
   * @param noise The 6x6 square root information matrix (upper triangular).
   */
  GMeas_Factor(Pose3d_Node* pose,const Point3d& measure, const Noise& noise)
    : FactorT<Point3d>("GMeas_Factor", 3, noise, measure), _pose(pose)
  {


    _nodes.resize(1);

    _nodes[0] = pose;
  }

  void initialize() {

  }


  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {

    const Pose3d& p2 = _pose->value(s);

    double GMod=sqrt(_measure.vector().dot(_measure.vector()));

    Eigen::Vector3d g_vect(0,GMod,0);


    Eigen::VectorXd err = (p2.rot().oRw()*g_vect) - _measure.vector();
    return err;
  }

};

class Acelerometer_Factor : public FactorT<Point3d> {
  Pose3d_Node* _pose1;
  Pose3d_Node* _pose2;
  Pose3d_Node* _pose3;
  Point3d_Node* _bias;
  Scale_Node* _GMod;
  double _Ts;

public:

  /**
   * Constructor.
   * @param pose1 The pose from which the measurement starts.
   * @param pose2 The pose to which the measurement extends.
   * @param measure The relative measurement from pose1 to pose2 (pose2 in pose1's frame).
   * @param noise The 6x6 square root information matrix (upper triangular).
   */
  Acelerometer_Factor(Pose3d_Node* pose1, Pose3d_Node* pose2, Pose3d_Node* pose3,Point3d_Node*bias,
      const Point3d& measure, const Noise& noise,const double &Ts,Scale_Node*GMod)
    : FactorT<Point3d>("Acelerometer_Factor", 3, noise, measure), _pose1(pose1), _pose2(pose2), _pose3(pose3),_bias(bias),_GMod(GMod),_Ts(Ts)
  {


    _nodes.resize(5);

    _nodes[0] = pose1;
    _nodes[1] = pose2;
    _nodes[2] = pose3;
    _nodes[3] = bias;
    _nodes[4] = GMod;
  }

  void initialize() {
   /* require((_pose1->initialized() || _pose2->initialized())&&_scale->initialized(),
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
    }*/

  }


  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
    const Pose3d& p1 = _pose1->value(s);
    const Pose3d& p2 = _pose2->value(s);
    const Pose3d& p3 = _pose3->value(s);

    Point3d t_32=(p3.ominus(p2)).trans();
    Point3d t_12=(p1.ominus(p2)).trans();

    Eigen::Vector3d g_vect(0,_GMod->value(s).x(),0);

    Eigen::Vector3d acel=(t_32.vector()+t_12.vector())*(1.0/_Ts)*(1.0/_Ts);

    Eigen::VectorXd err = (acel-p2.rot().oRw()*g_vect)+_bias->vector(s) - _measure.vector();
    return err;
  }

};



class Acelerometer_Factor_Dec : public FactorT<Point3d> {
  Pose3d_Node* _pose;
  Point3d_Node* _visual_acel;
  Scale_Node* _scale;
  Point3d_Node* _bias;
  double _GMod;

public:

  /**
   * Constructor.
   * @param pose1 The pose from which the measurement starts.
   * @param pose2 The pose to which the measurement extends.
   * @param measure The relative measurement from pose1 to pose2 (pose2 in pose1's frame).
   * @param noise The 6x6 square root information matrix (upper triangular).
   */
  Acelerometer_Factor_Dec(Pose3d_Node* pose, Point3d_Node* visual_acel, Scale_Node* scale,Point3d_Node*bias,
      const Point3d& measure, const Noise& noise,double GMod)
    : FactorT<Point3d>("Acelerometer_Factor_Dec", 3, noise, measure), _pose(pose), _visual_acel(visual_acel), _scale(scale),_bias(bias),_GMod(GMod)
  {


    _nodes.resize(4);

    _nodes[0] = pose;
    _nodes[1] = visual_acel;
    _nodes[2] = scale;
    _nodes[3] = bias;
  }

  void initialize() {
   /* require((_pose1->initialized() || _pose2->initialized())&&_scale->initialized(),
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
    }*/

  }


  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
    const Pose3d& p2 = _pose->value(s);

    Eigen::Vector3d acel=_visual_acel->vector(s)*_scale->value(s).x();

    Eigen::Vector3d g_vect(0,_GMod,0);
    Eigen::VectorXd err = (acel-p2.rot().oRw()*g_vect)+_bias->vector(s) - _measure.vector();
    return err;
  }

};


inline Eigen::MatrixXd matrix_toon2eigen(TooN::Matrix <TooN::Dynamic,TooN::Dynamic> m){

    Eigen::MatrixXd me(m.num_rows(),m.num_cols());
    for(int i=0;i<m.num_rows();i++)
        for(int j=0;j<m.num_cols();j++){
            me(i,j)=m(i,j);
        }

    return me;
}

inline Eigen::VectorXd vector_toon2eigen(TooN::Vector <TooN::Dynamic> m){

    Eigen::VectorXd me(m.size());
    for(int i=0;i<m.size();i++)
            me[i]=m[i];

    return me;
}

inline TooN::Matrix <TooN::Dynamic,TooN::Dynamic> matrix_eigen2toon(Eigen::MatrixXd m){

    TooN::Matrix <TooN::Dynamic,TooN::Dynamic> me(m.rows(),m.cols());
    for(int i=0;i<m.rows();i++)
        for(int j=0;j<m.cols();j++){
            me(i,j)=m(i,j);
        }

    return me;
}

inline Pose3d revboRP2isam(TooN::Vector <6> relPos,double K=1){

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



}


#endif // ISAM_DEFS_H
