 #ifndef EDGE_TRACKER_H
#define EDGE_TRACKER_H

#include "edge_finder.h"
#include "NormalDistribution.h"


class edge_tracker : public edge_finder
{

    int nmatch;     //Number of matches

public:



    using edge_finder::edge_finder;

    void rotate_keylines(TooN::Matrix <3,3> RotF);


    int directed_matching(TooN::Vector <3> Vel, TooN::Matrix <3,3> RVel, TooN::Matrix <3,3> BackRot, edge_tracker *et0, double min_thr_mod, double min_thr_ang, double max_radius, double loc_uncertainty, bool clear=false);

    int search_match(KeyLine &k,TooN::Vector <3> Vel,TooN::Matrix <3,3> RVel,TooN::Matrix <3,3> BackRot,double min_thr_mod,double min_thr_ang,double max_radius, double loc_uncertainty);

    int FordwardMatch(edge_tracker *et, bool clear=false);


    void UpdateInverseDepthKalman(TooN::Vector <3> vel, TooN::Matrix <3,3> RVel, TooN::Matrix <3,3> RW0, double ReshapeQAbsolute, double ReshapeQRelative, double LocationUncertainty);
    double UpdateInverseDepthKalman(KeyLine &kli, TooN::Vector <3> vel, TooN::Matrix <3,3> RVel, TooN::Matrix<3, 3> RW0, double ReshapeQAbsolute, double ReshapeQRelative, double LocationUncertainty);
    double UpdateInverseDepthKalmanSimple(KeyLine &kli, TooN::Vector <3> vel, TooN::Matrix <3,3> RVel, TooN::Matrix<3, 3> RW0, double ReshapeQAbsolute, double ReshapeQRelative, double LocationUncertainty);

    double EstimateQuantile(double s_rho_min, double s_rho_max, double percentile, int n);

    double EstimateReScaling(double &RKp, const double &s_rho_min, const uint &MatchNumMin, bool re_escale);


    bool ExtRotVel(TooN::Vector <3> &rot,TooN::Matrix <3,3> &RRot,TooN::Vector <3> &vel,TooN::Matrix <3,3> &RVel);

    int Regularize_1_iter(double thresh);

    int NumMatches(){return nmatch;}

    void DebugMatchHisto(uint m_num_max, uint bin_num, uint *histo);

    bool ExtRotVel(const TooN::Vector<3> &vel, TooN::Matrix<6, 6> &Wx, TooN::Matrix<6, 6> &Rx, TooN::Vector <6> &X, const double &LocUncert, double HubReweigth);
    static void BiasCorrect(TooN::Vector <6> &X,TooN::Matrix<6,6> &Wx,TooN::Vector <3> &Gb,TooN::Matrix<3,3> &Wb,const TooN::Matrix<3,3> &Rg,const TooN::Matrix<3,3> &Rb);

};




#endif // EDGE_TRACKER_H
