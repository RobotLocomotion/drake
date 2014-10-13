#ifndef __IKOPTIONS_H__
#define __IKOPTIONS_H__
//#include "RigidBodyManipulator.h"
#include <Eigen/Dense>
using namespace Eigen;

#if defined(WIN32) || defined(WIN64)
  #if defined(drakeIKoptions_EXPORTS)
    #define DLLEXPORT __declspec( dllexport )
  #else
    #define DLLEXPORT __declspec( dllimport )
  #endif
#else
  #define DLLEXPORT
#endif

class RigidBodyManipulator;

class DLLEXPORT IKoptions
{
  private:
    RigidBodyManipulator* robot;
    int nq;
    MatrixXd Q;
    MatrixXd Qa;
    MatrixXd Qv;
    bool debug_mode;
    bool sequentialSeedFlag;
    double SNOPT_MajorFeasibilityTolerance;
    int SNOPT_MajorIterationsLimit;
    int SNOPT_IterationsLimit;
    int SNOPT_SuperbasicsLimit;
    double SNOPT_MajorOptimalityTolerance;
    RowVectorXd additional_tSamples;
    bool fixInitialState;
    VectorXd q0_lb;
    VectorXd q0_ub;
    VectorXd qd0_lb;
    VectorXd qd0_ub;
    VectorXd qdf_lb;
    VectorXd qdf_ub;
  protected:
    void setDefaultParams(RigidBodyManipulator* robot);
  public:
    IKoptions(RigidBodyManipulator* robot);
    IKoptions(const IKoptions &rhs);
    ~IKoptions(void);
    RigidBodyManipulator* getRobotPtr() const;
    void setQ(const MatrixXd &Q);
    void setQa(const MatrixXd &Qa);
    void setQv(const MatrixXd &Qv);
    void setDebug(bool flag);
    void setSequentialSeedFlag(bool flag);
    void setMajorOptimalityTolerance(double tol);
    void setMajorFeasibilityTolerance(double tol);
    void setSuperbasicsLimit(int limit);
    void setMajorIterationsLimit(int limit);
    void setIterationsLimit(int limit);
    void setFixInitialState(bool flag);
    void setq0(const VectorXd &lb, const VectorXd &ub);
    void setqd0(const VectorXd &lb, const VectorXd &ub);
    void setqdf(const VectorXd &lb, const VectorXd &ub);
    void setAdditionaltSamples(const RowVectorXd &t_samples);
    void updateRobot(const RigidBodyManipulator* robot);
    void getQ(MatrixXd &Q) const;
    void getQa(MatrixXd &Qa) const;
    void getQv(MatrixXd &Qv) const;
    bool getDebug() const;
    bool getSequentialSeedFlag() const;
    double getMajorOptimalityTolerance() const;
    double getMajorFeasibilityTolerance() const;
    int getSuperbasicsLimit() const;
    int getMajorIterationsLimit() const;
    int getIterationsLimit() const;
    void getAdditionaltSamples(RowVectorXd &additional_tSamples) const;
    bool getFixInitialState() const;
    void getq0(VectorXd &lb, VectorXd &ub) const;
    void getqd0(VectorXd &lb, VectorXd &ub) const;
    void getqdf(VectorXd &lb, VectorXd &ub) const;
    void updateRobot(RigidBodyManipulator* new_robot);
};
#endif

