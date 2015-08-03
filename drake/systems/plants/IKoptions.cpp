#include "RigidBodyManipulator.h"
#include "IKoptions.h"
using namespace std;
using namespace Eigen;

IKoptions::IKoptions(RigidBodyManipulator* robot)
{
  // It is important to make sure these default values are consistent with the MATLAB IKoptions
  this->setDefaultParams(robot);
}

IKoptions::IKoptions(const IKoptions &rhs)
{
  this->robot = rhs.robot;
  this->nq = rhs.nq;
  this->Q = rhs.Q;
  this->Qa = rhs.Qa;
  this->Qv = rhs.Qv;
  this->debug_mode = rhs.debug_mode;
  this->sequentialSeedFlag = rhs.sequentialSeedFlag;
  this->SNOPT_MajorFeasibilityTolerance = rhs.SNOPT_MajorFeasibilityTolerance;
  this->SNOPT_MajorIterationsLimit = rhs.SNOPT_MajorIterationsLimit;
  this->SNOPT_IterationsLimit = rhs.SNOPT_IterationsLimit;
  this->SNOPT_SuperbasicsLimit = rhs.SNOPT_SuperbasicsLimit;
  this->SNOPT_MajorOptimalityTolerance = rhs.SNOPT_MajorOptimalityTolerance;
  this->additional_tSamples = rhs.additional_tSamples;
  this->fixInitialState = rhs.fixInitialState;
  this->q0_lb = rhs.q0_lb;
  this->q0_ub = rhs.q0_ub;
  this->qd0_lb = rhs.qd0_lb;
  this->qd0_ub = rhs.qd0_ub;
  this->qdf_lb = rhs.qdf_lb;
  this->qdf_ub = rhs.qdf_ub;
}
IKoptions::~IKoptions()
{
}

void IKoptions::setDefaultParams(RigidBodyManipulator* robot)
{
  this->robot = robot;
  this->nq = this->robot->num_positions;
  this->Q = MatrixXd::Identity(this->nq,this->nq);
  this->Qa = 0.1*MatrixXd::Identity(this->nq,this->nq);
  this->Qv = MatrixXd::Zero(this->nq,this->nq);
  this->debug_mode = true;
  this->sequentialSeedFlag = false;
  this->SNOPT_MajorFeasibilityTolerance = 1E-6;
  this->SNOPT_MajorIterationsLimit = 200;
  this->SNOPT_IterationsLimit = 10000;
  this->SNOPT_SuperbasicsLimit = 2000;
  this->SNOPT_MajorOptimalityTolerance = 1E-4;
  this->additional_tSamples.resize(0);
  this->fixInitialState = true;
  this->q0_lb = this->robot->joint_limit_min;
  this->q0_ub = this->robot->joint_limit_max;
  this->qd0_ub = VectorXd::Zero(this->nq);
  this->qd0_lb = VectorXd::Zero(this->nq);
  this->qdf_ub = VectorXd::Zero(this->nq);
  this->qdf_lb = VectorXd::Zero(this->nq);
}

RigidBodyManipulator* IKoptions::getRobotPtr() const
{
  return this->robot;
}

void IKoptions::setQ(const MatrixXd &Q)
{
  if(Q.rows() != this->nq || Q.cols() != this->nq)
  {
    cerr<<"Q should be nq x nq matrix"<<endl;
  }
  this->Q = (Q+Q.transpose())/2;
  SelfAdjointEigenSolver<MatrixXd> eigensolver(this->Q);
  VectorXd ev= eigensolver.eigenvalues();
  for(int i = 0;i<this->nq;i++)
  {
    if(ev(i)<0)
    {
      cerr<<"Q is not positive semi-definite"<<endl;
    }
  }
}

void IKoptions::setQa(const MatrixXd &Qa)
{
  if(Qa.rows() != this->nq || Qa.cols() != this->nq)
  {
    cerr<<"Qa should be nq x nq matrix"<<endl;
  }
  this->Qa = (Qa+Qa.transpose())/2;
  SelfAdjointEigenSolver<MatrixXd> eigensolver(this->Qa);
  VectorXd ev= eigensolver.eigenvalues();
  for(int i = 0;i<this->nq;i++)
  {
    if(ev(i)<0)
    {
      cerr<<"Qa is not positive semi-definite"<<endl;
    }
  }
}

void IKoptions::setQv(const MatrixXd &Qv)
{
  if(Qv.rows() != this->nq || Qv.cols() != this->nq)
  {
    cerr<<"Qv should be nq x nq matrix"<<endl;
  }
  this->Qv = (Qv+Qv.transpose())/2;
  SelfAdjointEigenSolver<MatrixXd> eigensolver(this->Qv);
  VectorXd ev= eigensolver.eigenvalues();
  for(int i = 0;i<this->nq;i++)
  {
    if(ev(i)<0)
    {
      cerr<<"Qv is not positive semi-definite"<<endl;
    }
  }
}
void IKoptions::getQ(MatrixXd &Q) const
{
  Q = this->Q;
}

void IKoptions::getQa(MatrixXd &Qa) const
{
  Qa = this->Qa;
}

void IKoptions::getQv(MatrixXd &Qv) const
{
  Qv = this->Qv;
}

void IKoptions::setDebug(bool flag)
{
  this->debug_mode = flag;
}

bool IKoptions::getDebug() const
{
  return this->debug_mode;
}

void IKoptions::setSequentialSeedFlag(bool flag)
{
  this->sequentialSeedFlag = flag;
}

bool IKoptions::getSequentialSeedFlag() const
{
  return this->sequentialSeedFlag;
}

void IKoptions::setMajorOptimalityTolerance(double tol)
{
  if(tol<=0)
  {
    cerr<<"Major Optimality Tolerance must be positive"<<endl;
  }
  this->SNOPT_MajorOptimalityTolerance = tol;
}

double IKoptions::getMajorOptimalityTolerance() const
{
  return this->SNOPT_MajorOptimalityTolerance;
}

void IKoptions::setMajorFeasibilityTolerance(double tol)
{
  if(tol<=0)
  {
    cerr<<"Major Feasibility Tolerance must be positive"<<endl;
  }
  this->SNOPT_MajorFeasibilityTolerance = tol;
}

double IKoptions::getMajorFeasibilityTolerance() const
{
  return this->SNOPT_MajorFeasibilityTolerance;
}

void IKoptions::setSuperbasicsLimit(int limit)
{
  if(limit<=0)
  {
    cerr<<"Superbasics limit must be positive"<<endl;
  }
  this->SNOPT_SuperbasicsLimit = limit;
}

int IKoptions::getSuperbasicsLimit() const
{
  return this->SNOPT_SuperbasicsLimit;
}

void IKoptions::setMajorIterationsLimit(int limit)
{
  if(limit<=0)
  {
    cerr<<"Major iterations limit must be positive"<<endl;
  }
  this->SNOPT_MajorIterationsLimit = limit;
}

int IKoptions::getMajorIterationsLimit() const
{
  return this->SNOPT_MajorIterationsLimit;
}

void IKoptions::setIterationsLimit(int limit)
{
  if(limit<=0)
  {
    cerr<<"Iterations limit must be positive"<<endl;
  }
  this->SNOPT_IterationsLimit = limit;
}

int IKoptions::getIterationsLimit() const
{
  return this->SNOPT_IterationsLimit;
}

void IKoptions::setFixInitialState(bool flag)
{
  this->fixInitialState = flag;
}

bool IKoptions::getFixInitialState() const
{
  return this->fixInitialState;
}

void IKoptions::setq0(const VectorXd &lb, const VectorXd& ub)
{
  if(lb.rows() != this->nq || ub.rows() != this->nq)
  {
    cerr<<"q0_lb and q0_ub must be nq x 1 column vector"<<endl;
  }
  this->q0_lb = lb;
  this->q0_ub = ub;
  for(int i = 0;i<this->nq;i++)
  {
    if(this->q0_lb(i)>this->q0_ub(i))
    {
      cerr<<"q0_lb must be no larger than q0_ub"<<endl;
    }
    this->q0_lb(i) = this->q0_lb(i)>this->robot->joint_limit_min[i]? this->q0_lb(i): this->robot->joint_limit_min[i];
    this->q0_ub(i) = this->q0_ub(i)<this->robot->joint_limit_max[i]? this->q0_ub(i): this->robot->joint_limit_max[i];
  }
}

void IKoptions::getq0(VectorXd &lb, VectorXd &ub) const
{
  lb = this->q0_lb;
  ub = this->q0_ub;
}

void IKoptions::setqd0(const VectorXd &lb, const VectorXd& ub)
{
  if(lb.rows() != this->nq || ub.rows() != this->nq)
  {
    cerr<<"qd0_lb and qd0_ub must be nq x 1 column vector"<<endl;
  }
  for(int i = 0;i<this->nq;i++)
  {
    if(lb(i)>ub(i))
    {
      cerr<<"qd0_lb must be no larger than qd0_ub"<<endl;
    }
  }
  this->qd0_lb = lb;
  this->qd0_ub = ub;
}

void IKoptions::getqd0(VectorXd &lb, VectorXd &ub) const
{
  lb = this->qd0_lb;
  ub = this->qd0_ub;
}

void IKoptions::setqdf(const VectorXd &lb, const VectorXd& ub)
{
  if(lb.rows() != this->nq || ub.rows() != this->nq)
  {
    cerr<<"qdf_lb and qdf_ub must be nq x 1 column vector"<<endl;
  }
  for(int i = 0;i<this->nq;i++)
  {
    if(lb(i)>ub(i))
    {
      cerr<<"qdf_lb must be no larger than qdf_ub"<<endl;
    }
  }
  this->qdf_lb = lb;
  this->qdf_ub = ub;
}

void IKoptions::getqdf(VectorXd &lb, VectorXd &ub) const
{
  lb = this->qdf_lb;
  ub = this->qdf_ub;
}

void IKoptions::setAdditionaltSamples(const RowVectorXd &t_samples)
{
  if(t_samples.size()>0)
  {
    set<double> unique_sort_t(t_samples.data(),t_samples.data()+t_samples.size());
    this->additional_tSamples.resize(unique_sort_t.size());
    int t_idx = 0;
    for(auto it = unique_sort_t.begin();it != unique_sort_t.end();it++)
    {
      this->additional_tSamples(t_idx) = *it;
      t_idx++;
    }
  }
  else
  {
    this->additional_tSamples.resize(0);
  }
}

void IKoptions::getAdditionaltSamples(RowVectorXd &t_samples) const
{
  t_samples = this->additional_tSamples;
}

void IKoptions::updateRobot(RigidBodyManipulator* new_robot)
{
  this->robot = new_robot;
  int nq_cache = this->nq;
  this->nq = this->robot->num_positions;
  if(nq_cache != nq)
  {
    this->setDefaultParams(new_robot);
  }
}

