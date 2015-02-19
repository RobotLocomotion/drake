#ifndef __IKOPTIONS_H__
#define __IKOPTIONS_H__
//#include "RigidBodyManipulator.h"
#include <Eigen/Dense>

#undef drakeIKoptions_DLLEXPORT
#if defined(WIN32) || defined(WIN64)
  #if defined(drakeIKoptions_EXPORTS)
    #define drakeIKoptions_DLLEXPORT __declspec( dllexport )
  #elif defined(drakeIK_EXPORTS) // because this gets built in both the drakeIKoptions target and the drakeIK target...
    #define drakeIKoptions_DLLEXPORT __declspec( dllexport )
  #else
    #define drakeIKoptions_DLLEXPORT __declspec( dllimport )
  #endif  
#else
  #define drakeIKoptions_DLLEXPORT
#endif

class RigidBodyManipulator;

class drakeIKoptions_DLLEXPORT IKoptions
{
  private:
    RigidBodyManipulator* robot;
    int nq;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd Qa;
    Eigen::MatrixXd Qv;
    bool debug_mode;
    bool sequentialSeedFlag;
    double SNOPT_MajorFeasibilityTolerance;
    int SNOPT_MajorIterationsLimit;
    int SNOPT_IterationsLimit;
    int SNOPT_SuperbasicsLimit;
    double SNOPT_MajorOptimalityTolerance;
    Eigen::RowVectorXd additional_tSamples;
    bool fixInitialState;
    Eigen::VectorXd q0_lb;
    Eigen::VectorXd q0_ub;
    Eigen::VectorXd qd0_lb;
    Eigen::VectorXd qd0_ub;
    Eigen::VectorXd qdf_lb;
    Eigen::VectorXd qdf_ub;
  protected:
    void setDefaultParams(RigidBodyManipulator* robot);
  public:
    IKoptions(RigidBodyManipulator* robot);
    IKoptions(const IKoptions &rhs);
    ~IKoptions(void);
    RigidBodyManipulator* getRobotPtr() const;
    void setQ(const Eigen::MatrixXd &Q);
    void setQa(const Eigen::MatrixXd &Qa);
    void setQv(const Eigen::MatrixXd &Qv);
    void setDebug(bool flag);
    void setSequentialSeedFlag(bool flag);
    void setMajorOptimalityTolerance(double tol);
    void setMajorFeasibilityTolerance(double tol);
    void setSuperbasicsLimit(int limit);
    void setMajorIterationsLimit(int limit);
    void setIterationsLimit(int limit);
    void setFixInitialState(bool flag);
    void setq0(const Eigen::VectorXd &lb, const Eigen::VectorXd &ub);
    void setqd0(const Eigen::VectorXd &lb, const Eigen::VectorXd &ub);
    void setqdf(const Eigen::VectorXd &lb, const Eigen::VectorXd &ub);
    void setAdditionaltSamples(const Eigen::RowVectorXd &t_samples);
    void updateRobot(const RigidBodyManipulator* robot);
    void getQ(Eigen::MatrixXd &Q) const;
    void getQa(Eigen::MatrixXd &Qa) const;
    void getQv(Eigen::MatrixXd &Qv) const;
    bool getDebug() const;
    bool getSequentialSeedFlag() const;
    double getMajorOptimalityTolerance() const;
    double getMajorFeasibilityTolerance() const;
    int getSuperbasicsLimit() const;
    int getMajorIterationsLimit() const;
    int getIterationsLimit() const;
    void getAdditionaltSamples(Eigen::RowVectorXd &additional_tSamples) const;
    bool getFixInitialState() const;
    void getq0(Eigen::VectorXd &lb, Eigen::VectorXd &ub) const;
    void getqd0(Eigen::VectorXd &lb, Eigen::VectorXd &ub) const;
    void getqdf(Eigen::VectorXd &lb, Eigen::VectorXd &ub) const;
    void updateRobot(RigidBodyManipulator* new_robot);
};
#endif

