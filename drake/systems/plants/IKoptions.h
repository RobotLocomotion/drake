#ifndef __IKOPTIONS_H__
#define __IKOPTIONS_H__
#include <Eigen/Dense>
#include "drake/drakeIKoptions_export.h"


class RigidBodyTree;

class DRAKEIKOPTIONS_EXPORT IKoptions
{
  private:
    RigidBodyTree * robot;
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
    void setDefaultParams(RigidBodyTree * robot);
  public:
    IKoptions(RigidBodyTree * robot);
    IKoptions(const IKoptions &rhs);
    ~IKoptions(void);
    RigidBodyTree * getRobotPtr() const;
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
    void updateRobot(RigidBodyTree * new_robot);
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
};
#endif

