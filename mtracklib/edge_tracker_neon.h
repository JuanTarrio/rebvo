 #ifndef EDGE_TRACKER_H
#define EDGE_TRACKER_H

#include "edge_finder.h"
#include "NormalDistribution.h"
#include "cam_model.h"

#define RHO_MAX     20
#define RHO_MIN     1e-3
#define RhoInit     1
#define VRhoInit    1e6
#define SRhoInit    1e3

#define DepthR0     5

#define FeatKMin    5

#define RefKNum    200

#define EKF_DQ     1.0
#define EKF_SGIRO  0.01
#define ERR_DQ     2.0


class edge_tracker : public edge_finder
{
public:

    cam_model cam_mod;
private:


    util::FixedCircList <TooN::Vector<3>,EP_LIST_SIZE> vlist;

    unsigned long int tag_inx;

    util::NormalDistribution <double,65536>norm_dist;


public:



    int nmatch;

    edge_tracker(float max_img_value, Point2DF pp, Point2DF zf, double Kc[2], Size2D sz);
    void rotate_keylines(TooN::Matrix <3,3> RotF,double zf);

    void BackRotate_keylines(TooN::Matrix <3,3> RotF);
    static void rotate_klpos(KLPos &kl, TooN::Matrix <3,3> RotF, double zf);
    int directed_matching(TooN::Vector <3> Vel,TooN::Matrix <3,3> RVel,TooN::Matrix <3,3> BackRot,edge_tracker *et0, double min_score,double max_radius,bool clear=false);


    template <bool rotate,bool test_rho>
    int search_match(KeyLine &k,TooN::Vector <3> Vel,TooN::Matrix <3,3> RVel,TooN::Matrix <3,3> BackRot,double min_score,double max_radius);

    int FordwardMatch(edge_tracker *et, bool clear=false);

    void UpdateInverseDepthKalman(TooN::Vector <3> vel, TooN::Matrix <3,3> RVel, TooN::Matrix <3,3> RW0, double ReshapeFactor, double RelError);
    double UpdateInverseDepthKalman(TooN::Vector <3> vel, TooN::Matrix <3,3> RVel, double RelError, double ReshapeFactor, int kl_inx);
    double UpdateInverseDepthKalmanU(TooN::Vector <3> vel, TooN::Matrix <3,3> RVel, int kl_inx);
    double UpdateInverseDepthKalmanU_TNorm(TooN::Vector <3> vel, TooN::Matrix <3,3> RVel, TooN::Matrix <3,3> RW0, double ReshapeFactor, int kl_inx);
    double UpdateInverseDepthIterativeKalman(TooN::Vector <3> vel, TooN::Matrix <3,3> RVel, double RelError, double ReshapeFactor, int kl_inx);
    double UpdateInverseDepthIterativeKalman2(TooN::Vector <3> vel, TooN::Matrix <3,3> RVel, double RelError, double ReshapeFactor, int kl_inx);
    double UpdateInverseDepthKalmanUniformMix(TooN::Vector <3> vel,TooN::Matrix <3,3> RVel,TooN::Matrix <3,3> RW0,double ReshapeFactor,int kl_inx);


    double Stabilize(TooN::Vector <3> vel,TooN::Matrix <3,3> RVel,double zf);
    void ReScale(double Kp, double RKp);


    void ExtVel(TooN::Vector <3> &vel,TooN::Matrix <3,3> &RVel);

    double EstimateQuantile(double s_rho_min, double s_rho_max, double percentile, int n);

    void Interp();
    void CalcReScaling(double & Kp, double &RKp, int &n);

    double EstimateReScaling(double &RKp);

    double EstimateScaling();



    void AddMeas(TooN::Vector <3> vel,TooN::Matrix <3,3> RVel,TooN::Matrix <3,3> RotF,edge_tracker *et0,double zf);
    double CalcDephs(double zf, double &RKp,TooN::Matrix <3,3> RVel);

    void saveTrays(const char *fname);

    double HistoMedian(int bins, double bounds, double &tmass);


    cam_model & CamMod(){return cam_mod;}

    void PrintTrails(RGB24Pixel *data, char r, char g, char b);

    static void LimitRho(double &rho,double &s_rho);

    static void LimitZ(double &rho,double &s_rho);

    void build_kl_mask();


    int forward_directed_matching(TooN::Vector <3> Vel,TooN::Matrix <3,3> RVel,edge_tracker *et,double min_score,double max_radius,bool clear=false);

    int Regularize_1_iter(double thresh);
};




#endif // EDGE_TRACKER_H
