#ifndef __IKOPTIONS_H__
#define __IKOPTIONS_H__
//#include "RigidBodyManipulator.h"
#include <Eigen/Dense>
using namespace Eigen;

class RigidBodyManipulator;
class IKoptions
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
    RigidBodyManipulator* getRobotPtr();
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
    void getQ(MatrixXd &Q);
    void getQa(MatrixXd &Qa);
    void getQv(MatrixXd &Qv);
    bool getDebug();
    bool getSequentialSeedFlag();
    double getMajorOptimalityTolerance();
    double getMajorFeasibilityTolerance();
    int getSuperbasicsLimit();
    int getMajorIterationsLimit();
    int getIterationsLimit();
    void getAdditionaltSamples(RowVectorXd &additional_tSamples);
    bool getFixInitialState();
    void getq0(VectorXd &lb, VectorXd &ub);
    void getqd0(VectorXd &lb, VectorXd &ub);
    void getqdf(VectorXd &lb, VectorXd &ub);
    void updateRobot(RigidBodyManipulator* new_robot);
};
#endif

