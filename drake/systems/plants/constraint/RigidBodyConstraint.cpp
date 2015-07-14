#include "RigidBodyConstraint.h"
#include "../RigidBodyManipulator.h"

#include <map>
#include "../../../util/drakeGeometryUtil.h"

#if defined(WIN32) || defined(WIN64)
  #include <math.h>
  #define isnan(x) _isnan(x)
  #define isinf(x) (!_finite(x))
#else
  #define isnan(x) std::isnan(x)
  #define isinf(x) std::isinf(x)
#endif
using namespace Eigen;


void drakePrintMatrix(const MatrixXd &mat)
{
  for(int i = 0;i<mat.rows();i++)
  {
    for(int j = 0;j<mat.cols();j++)
    {
      printf("%7.3f ",mat(i,j));
    }
    printf("\n");
  }
};

namespace DrakeRigidBodyConstraint{
  Vector3d com_pts = Vector3d::Zero();
  const int WorldCoMDefaultRobotNum[1] = {0};
  Vector2d default_tspan(-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
}
RigidBodyConstraint::RigidBodyConstraint(int category, RigidBodyManipulator* robot, const Vector2d &tspan)
{
  if(category>=0 || category <=-7)
  {
    std::cerr<<"Drake:RigidBodyConstraint:Unsupported constraint category"<<std::endl;
  }
  this->category = category;
  this->type = 0;
  this->robot = robot;
  if(tspan(0)>tspan(1))
  {
    std::cerr<<"Drake:RigidBodyConstraint:tspan(0) should be no larger than tspan(1)"<<std::endl;
  }
  this->tspan[0] = tspan(0);
  this->tspan[1] = tspan(1);
}

RigidBodyConstraint::RigidBodyConstraint(const RigidBodyConstraint &rhs):category(rhs.category),type(rhs.type),robot(rhs.robot)
{
  this->tspan[0] = rhs.tspan[0];
  this->tspan[1] = rhs.tspan[1];
}

RigidBodyConstraint::~RigidBodyConstraint(void){};


const int QuasiStaticDefaultRobotNum[1] = {0};
const std::set<int> QuasiStaticConstraint::defaultRobotNumSet(QuasiStaticDefaultRobotNum,QuasiStaticDefaultRobotNum+1);
QuasiStaticConstraint::QuasiStaticConstraint(RigidBodyManipulator* robot, const Vector2d &tspan,const std::set<int> &robotnumset):RigidBodyConstraint(RigidBodyConstraint::QuasiStaticConstraintCategory,robot,tspan)
{
  this->m_robotnumset = robotnumset;
  this->shrinkFactor = 0.9;
  this->active = false;
  this->num_bodies = 0;
  this->num_pts = 0;
  this->type = RigidBodyConstraint::QuasiStaticConstraintType;
}

QuasiStaticConstraint::QuasiStaticConstraint(const QuasiStaticConstraint &rhs):RigidBodyConstraint(rhs),m_robotnumset(rhs.m_robotnumset),shrinkFactor(rhs.shrinkFactor),active(rhs.active),num_bodies(rhs.num_bodies),num_pts(rhs.num_pts),bodies(rhs.bodies),num_body_pts(rhs.num_body_pts),body_pts(rhs.body_pts)
{
}

QuasiStaticConstraint::~QuasiStaticConstraint()
{
}

bool QuasiStaticConstraint::isTimeValid(const double* t) const
{
  if(t == nullptr) return true;
  return (*t)>=this->tspan[0]&&(*t)<=this->tspan[1];
}

int QuasiStaticConstraint::getNumConstraint(const double* t) const
{
  if(this->isTimeValid(t))
  {
    return 3;
  }
  else
  {
    return 0;
  }
}

void QuasiStaticConstraint::updateRobot(RigidBodyManipulator* robot)
{
  this->robot = robot;
}
void QuasiStaticConstraint::eval(const double* t, const double* weights, VectorXd &c, MatrixXd &dc) const
{
  if(this->isTimeValid(t))
  {
    int nq = this->robot->num_positions;
    dc.resize(2,nq+this->num_pts);
    auto com_gradientvar = robot->centerOfMass<double>(1, m_robotnumset);
    const auto& com = com_gradientvar.value();
    const auto& dcom = com_gradientvar.gradient().value();
    MatrixXd contact_pos(3,this->num_pts);
    MatrixXd dcontact_pos(3*this->num_pts,nq);
    int num_accum_pts = 0;
    Vector3d center_pos= Vector3d::Zero();
    MatrixXd dcenter_pos = MatrixXd::Zero(3,nq);
    for(int i = 0;i<this->num_bodies;i++)
    {
      auto body_contact_pos_gradientvar = robot->forwardKinNew(body_pts[i], bodies[i], 0, 0, 1);
      const auto& body_contact_pos = body_contact_pos_gradientvar.value();
      const auto& dbody_contact_pos = body_contact_pos_gradientvar.gradient().value();

      contact_pos.block(0,num_accum_pts,3,this->num_body_pts[i]) = body_contact_pos;
      dcontact_pos.block(3*num_accum_pts,0,3*this->num_body_pts[i],nq) = dbody_contact_pos;
      for(int j = 0;j<this->num_body_pts[i];j++)
      {
        center_pos = center_pos+body_contact_pos.col(j);
        dcenter_pos = dcenter_pos+dbody_contact_pos.block(3*j,0,3,nq);
      }
      num_accum_pts += this->num_body_pts[i];
    }
    center_pos = center_pos/this->num_pts;
    dcenter_pos = dcenter_pos/this->num_pts;
    MatrixXd support_pos(2,this->num_pts);
    MatrixXd dsupport_pos(2*this->num_pts,nq);
    c = com.head(2);
    dc.block(0,0,2,nq) = dcom.block(0,0,2,nq);
    for(int i = 0;i<this->num_pts;i++)
    {
      support_pos.col(i) = center_pos.head(2)*(1.0-this->shrinkFactor)+contact_pos.block(0,i,2,1)*this->shrinkFactor;
      dsupport_pos.block(2*i,0,2,nq) = dcenter_pos.block(0,0,2,nq)*(1.0-this->shrinkFactor)+dcontact_pos.block(3*i,0,2,nq)*this->shrinkFactor;
      c = c-weights[i]*support_pos.col(i);
      dc.block(0,0,2,nq) = dc.block(0,0,2,nq)-weights[i]*dsupport_pos.block(2*i,0,2,nq);
    }
    dc.block(0,nq,2,this->num_pts) = -support_pos;
  }
  else
  {
    c.resize(0);
    dc.resize(0,0);
  }
}

void QuasiStaticConstraint::bounds(const double* t,VectorXd &lb, VectorXd &ub) const
{
  if(this->isTimeValid(t))
  {
    lb.resize(2);
    ub.resize(2);
    lb<<0.0,0.0;
    ub<<0.0,0.0;
  }
  else
  {
    lb.resize(0);
    ub.resize(0);
  }
}

void QuasiStaticConstraint::name(const double* t,std::vector<std::string> &name_str) const
{
  if(t == nullptr)
  {
    char cnst_name_str_buffer[500];
    sprintf(cnst_name_str_buffer,"QuasiStaticConstraint x");
    std::string cnst_name_str1(cnst_name_str_buffer);
    name_str.push_back(cnst_name_str1);
    sprintf(cnst_name_str_buffer,"QuasiStaticConstraint y");
    std::string cnst_name_str2(cnst_name_str_buffer);
    name_str.push_back(cnst_name_str2);
  }
  else
  {
    char cnst_name_str_buffer[500];
    sprintf(cnst_name_str_buffer,"QuasiStaticConstraint x at time %10.4f",*t);
    std::string cnst_name_str1(cnst_name_str_buffer);
    name_str.push_back(cnst_name_str1);
    sprintf(cnst_name_str_buffer,"QuasiStaticConstraint y at time %10.4f",*t);
    std::string cnst_name_str2(cnst_name_str_buffer);
    name_str.push_back(cnst_name_str2);
  }
}

static bool compare3Dvector(const Vector3d& a, const Vector3d& b)
{
  if(a(0)<b(0)) return true;
  if(a(0)>b(0)) return false;
  if(a(1)<b(1)) return true;
  if(a(1)>b(1)) return false;
  if(a(2)<b(2)) return true;
  return false;
}
void QuasiStaticConstraint::addContact(int num_new_bodies,const int* new_bodies, const Matrix3Xd* new_body_pts)
{
  for(int i = 0;i<num_new_bodies;i++)
  {
    bool findDuplicateBody = false;
    if(new_body_pts[i].rows() != 3)
    {
      std::cerr<<"new_body_pts must all have 3 rows"<<std::endl;
    }
    for(int j = 0;j<this->num_bodies;j++)
    {
      if(this->bodies[j] == new_bodies[i])
      {
        findDuplicateBody = true;
        bool (*compare3Dvector_ptr)(const Vector3d&,const Vector3d&) = compare3Dvector;
        std::set<Vector3d,bool(*)(const Vector3d&, const Vector3d&)> unique_body_pts (compare3Dvector_ptr);
        for(int k = 0;k<this->body_pts[j].cols();k++)
        {
          unique_body_pts.insert(body_pts[j].block(0,k,3,1));
        }
        for(int k = 0;k<new_body_pts[i].cols();k++)
        {
          unique_body_pts.insert(new_body_pts[i].block(0,k,3,1));
        }
        this->num_pts -= this->num_body_pts[j];
        this->num_body_pts[j] = static_cast<int>(unique_body_pts.size());
        this->num_pts += this->num_body_pts[j];
        this->body_pts[j].resize(3,this->num_body_pts[j]);
        int col_idx = 0;
        for(auto it = unique_body_pts.begin();it!=unique_body_pts.end();it++)
        {
          this->body_pts[j].block(0, col_idx,3,1) = *it;
          col_idx++;
        }
      }
    }
    if(!findDuplicateBody)
    {
      bodies.push_back(new_bodies[i]);
      num_body_pts.push_back(static_cast<int>(new_body_pts[i].cols()));
      body_pts.push_back(new_body_pts[i]);
      num_bodies++;
      num_pts += static_cast<int>(new_body_pts[i].cols());
    }
  }
}

void QuasiStaticConstraint::setShrinkFactor(double factor)
{
  if(factor<=0.0)
  {
    std::cerr<<"factor should be positive"<<std::endl;
  }
  this->shrinkFactor = factor;
}

void QuasiStaticConstraint::updateRobotnum(std::set<int> &robotnumset)
{
  this->m_robotnumset = robotnumset;
}

PostureConstraint::PostureConstraint(RigidBodyManipulator* robot, const Eigen::Vector2d &tspan):RigidBodyConstraint(RigidBodyConstraint::PostureConstraintCategory,robot,tspan)
{
	this->joint_limit_min0 = this->robot->joint_limit_min;
	this->joint_limit_max0 = this->robot->joint_limit_max;
  this->lb = this->joint_limit_min0;
  this->ub = this->joint_limit_max0;
  this->type = RigidBodyConstraint::PostureConstraintType;
}

PostureConstraint::PostureConstraint(const PostureConstraint& rhs):RigidBodyConstraint(rhs)
{
  int nq = this->robot->num_positions;
  this->lb.resize(nq);
  this->ub.resize(nq);
  this->joint_limit_min0 = rhs.joint_limit_min0;
  this->joint_limit_max0 = rhs.joint_limit_max0;
  for(int i = 0;i<nq;i++)
  {
    this->lb[i] = rhs.lb[i];
    this->ub[i] = rhs.ub[i];
  }
}

bool PostureConstraint::isTimeValid(const double* t) const
{
  if(t == nullptr) return true;
  return (*t)>=this->tspan[0]&&(*t)<=this->tspan[1];
}

void PostureConstraint::setJointLimits(int num_idx,const int* joint_idx, const VectorXd& lb, const VectorXd& ub)
{
  for(int i = 0;i<num_idx;i++)
  {
    if(joint_idx[i]>=this->robot->num_positions || joint_idx[i]<0)
    {
      std::cerr<<"joint_idx["<<i<<"] is should be within [0 nq-1]"<<std::endl;
    }
		if(lb[i]>this->robot->joint_limit_max[joint_idx[i]])
		{
			std::cerr<<"joint lb is greater than the robot default joint maximum"<<std::endl;
		}
		if(ub[i]<this->robot->joint_limit_min[joint_idx[i]])
    {
			std::cerr<<"joint ub is smaller than the robot default joint minimum"<<std::endl;
    }
		this->lb[joint_idx[i]] = (this->robot->joint_limit_min[joint_idx[i]]<lb[i]?lb[i]:this->robot->joint_limit_min[joint_idx[i]]);
		this->ub[joint_idx[i]] = (this->robot->joint_limit_max[joint_idx[i]]>ub[i]?ub[i]:this->robot->joint_limit_max[joint_idx[i]]);
  }
}

void PostureConstraint::bounds(const double* t, VectorXd& joint_min, VectorXd& joint_max) const
{
  if(this->isTimeValid(t))
  {
    joint_min = this->lb;
    joint_max = this->ub;
  }
  else
  {
    joint_min = this->robot->joint_limit_min;
    joint_max = this->robot->joint_limit_max;
  }
}


MultipleTimeLinearPostureConstraint::MultipleTimeLinearPostureConstraint(RigidBodyManipulator *robot, const Eigen::Vector2d &tspan):RigidBodyConstraint(RigidBodyConstraint::MultipleTimeLinearPostureConstraintCategory,robot,tspan)
{
}

MultipleTimeLinearPostureConstraint::MultipleTimeLinearPostureConstraint(const MultipleTimeLinearPostureConstraint &rhs):RigidBodyConstraint(rhs)
{
}

std::vector<bool> MultipleTimeLinearPostureConstraint::isTimeValid(const double* t, int n_breaks) const
{
  std::vector<bool> flag;
  for(int i = 0;i<n_breaks;i++)
  {
    if(i<n_breaks-1)
    {
      if(t[i+1]<t[i])
      {
        std::cerr<<"Drake:Constraint:BadInputs: t must be in ascending order"<<std::endl;
      }
    }
    if((t[i]>this->tspan[1]||t[i]<this->tspan[0]))
    {
      flag.push_back(false);
    }
    else
    {
      flag.push_back(true);
    }
  }
  return flag;
}

int MultipleTimeLinearPostureConstraint::numValidTime(const std::vector<bool> &valid_flag) const
{
  int num_valid_t = 0;
  for(auto it=valid_flag.begin();it != valid_flag.end(); it++)
  {
    if(*it)
    {
      num_valid_t++;
    }
  }
  return num_valid_t;
}

void MultipleTimeLinearPostureConstraint::validTimeInd(const std::vector<bool> &valid_flag, VectorXi &valid_t_ind) const
{
  valid_t_ind.resize(0);
  for(int i = 0;i<valid_flag.size();i++)
  {
    if(valid_flag.at(i))
    {
      valid_t_ind.conservativeResize(valid_t_ind.size()+1);
      valid_t_ind(valid_t_ind.size()-1) = i;
    }
  }
}

void MultipleTimeLinearPostureConstraint::eval(const double* t, int n_breaks, const MatrixXd &q, VectorXd &c, SparseMatrix<double> &dc) const
{
  this->feval(t,n_breaks,q,c);
  VectorXi iAfun;
  VectorXi jAvar;
  VectorXd A;
  this->geval(t,n_breaks,iAfun,jAvar,A);
  int num_cnst = this->getNumConstraint(t,n_breaks);
  dc.resize(num_cnst,static_cast<int>(q.size()));
  dc.reserve(static_cast<int>(A.size()));
  for(int i = 0;i<iAfun.size();i++)
  {
    dc.insert(iAfun(i),jAvar(i)) = A(i);
  }
}

SingleTimeLinearPostureConstraint::SingleTimeLinearPostureConstraint(RigidBodyManipulator* robot, const VectorXi &iAfun, const VectorXi &jAvar, const VectorXd &A, const VectorXd &lb, const VectorXd &ub, const Vector2d &tspan):RigidBodyConstraint(RigidBodyConstraint::SingleTimeLinearPostureConstraintCategory,robot,tspan)
{
  this->lb = lb;
  this->ub = ub;
  this->num_constraint = static_cast<int>(this->lb.size());
  for(int i = 0;i<this->num_constraint;i++)
  {
    if(lb(i)>ub(i))
      std::cerr<<"Drake:RigidBodyConstraint:SingleTimeLinearPostureConstraint:lb must be no larger than ub"<<std::endl;
  }
  int lenA = static_cast<int>(iAfun.size());
  if(jAvar.size() != lenA || A.size() != lenA)
  {
    std::cerr<<"Drake:RigidBodyConstraint:SingleTimeLinearPostureConstraint:iAfun,jAvar and A should be of the same size"<<std::endl;
  }
  std::set<std::pair<int,int>> mat_ind_set;
  for(int i = 0;i<lenA;i++)
  {
    std::pair<int,int> ind_pair(iAfun(i),jAvar(i));
    mat_ind_set.insert(ind_pair);
  }
  if(mat_ind_set.size() != lenA)
  {
    std::cerr<<"Drake:RigidBodyConstraint:SingleTimeLinearPostureConstraint:The pair (iAfun(i),jAvar(i)) is not unique"<<std::endl;
  }
  if(iAfun.maxCoeff() != this->num_constraint-1)
  {
    std::cerr<<"Drake:RigidBodyConstraint:SingleTimeLinearPostureConstraint:max(iAfun) should be lb.size()-1"<<std::endl;
  }
  if(iAfun.minCoeff() != 0)
  {
    std::cerr<<"Drake:RigidBodyConstraint:SingleTimeLinearPostureConstraint:min(iAfun) should be 0"<<std::endl;
  }
  if(jAvar.minCoeff()<0 || jAvar.maxCoeff()>this->robot->num_positions-1)
  {
    std::cerr<<"Drake:RigidBodyConstraint:SingleTimeLinearPostureConstraint: jAvar should be within[0 robot.nq-1]"<<std::endl;
  }
  this->iAfun = iAfun;
  this->jAvar = jAvar;
  this->A = A;
  this->A_mat.resize(this->num_constraint,this->robot->num_positions);
  this->A_mat.reserve(lenA);
  for(int i = 0;i<lenA;i++)
  {
    A_mat.insert(iAfun(i),jAvar(i)) = A(i);
  }
  this->type = RigidBodyConstraint::SingleTimeLinearPostureConstraintType;
}

SingleTimeLinearPostureConstraint::SingleTimeLinearPostureConstraint(const SingleTimeLinearPostureConstraint &rhs):RigidBodyConstraint(rhs),iAfun(rhs.iAfun),jAvar(rhs.jAvar),A(rhs.A),lb(rhs.lb),ub(rhs.ub),num_constraint(rhs.num_constraint),A_mat(rhs.A_mat)
{
}

bool SingleTimeLinearPostureConstraint::isTimeValid(const double* t) const
{
  if(t == nullptr)
  {
    return true;
  }
  if(*t>=this->tspan[0] && *t<= this->tspan[1])
  {
    return true;
  }
  return false;
}

int SingleTimeLinearPostureConstraint::getNumConstraint(const double* t) const
{
  if(this->isTimeValid(t))
  {
    return this->num_constraint;
  }
  else
  {
    return 0;
  }
}


void SingleTimeLinearPostureConstraint::bounds(const double* t, VectorXd &lb, VectorXd &ub) const
{
  if(this->isTimeValid(t))
  {
    lb = this->lb;
    ub = this->ub;
  }
  else
  {
    lb.resize(0);
    ub.resize(0);
  }
}

void SingleTimeLinearPostureConstraint::feval(const double* t, const VectorXd &q, VectorXd &c) const
{
  if(this->isTimeValid(t))
  {
    c = this->A_mat*q;
  }
  else
  {
    c.resize(0);
  }
}

void SingleTimeLinearPostureConstraint::geval(const double* t, VectorXi &iAfun, VectorXi &jAvar, VectorXd &A) const
{
  if(this->isTimeValid(t))
  {
    iAfun = this->iAfun;
    jAvar = this->jAvar;
    A = this->A;
  }
  else
  {
    iAfun.resize(0);
    jAvar.resize(0);
    A.resize(0);
  }
}

void SingleTimeLinearPostureConstraint::eval(const double* t, const VectorXd &q, VectorXd &c, SparseMatrix<double> &dc) const
{
  if(this->isTimeValid(t))
  {
    c = this->A_mat*q;
    dc = this->A_mat;
  }
  else
  {
    c.resize(0);
    dc.resize(0,0);
  }
}

void SingleTimeLinearPostureConstraint::name(const double* t, std::vector<std::string> &name_str) const
{
  if(this->isTimeValid(t))
  {
    char cnst_name_buffer[100];
    for(int i = 0;i<this->num_constraint;i++)
    {
      if(t == nullptr)
      {
        sprintf(cnst_name_buffer,"SingleTimeLinearPostureConstraint row %d\n",i);
      }
      else
      {
        sprintf(cnst_name_buffer,"SingleTimeLinearPostureConstraint row %d at time %6.3f\n",i,*t);
      }
      std::string cnst_name_str(cnst_name_buffer);
      name_str.push_back(cnst_name_str);
    }
  }
}


SingleTimeKinematicConstraint::SingleTimeKinematicConstraint(RigidBodyManipulator *robot, const Vector2d &tspan): RigidBodyConstraint(RigidBodyConstraint::SingleTimeKinematicConstraintCategory,robot,tspan)
{
  this->num_constraint = 0;
}

SingleTimeKinematicConstraint::SingleTimeKinematicConstraint(const SingleTimeKinematicConstraint &rhs):RigidBodyConstraint(rhs)
{
  this->num_constraint = rhs.num_constraint;
}
bool SingleTimeKinematicConstraint::isTimeValid(const double* t) const
{
   if(t == nullptr) return true;
   return (*t)>=this->tspan[0]&&(*t)<=this->tspan[1];
}

int SingleTimeKinematicConstraint::getNumConstraint(const double* t) const
{
  if(isTimeValid(t))
  {
    return this->num_constraint;
  }
  return 0;
}

void SingleTimeKinematicConstraint::updateRobot(RigidBodyManipulator* robot)
{
  this->robot = robot;
}

MultipleTimeKinematicConstraint::MultipleTimeKinematicConstraint(RigidBodyManipulator *robot, const Vector2d &tspan): RigidBodyConstraint(RigidBodyConstraint::MultipleTimeKinematicConstraintCategory,robot,tspan)
{
}

MultipleTimeKinematicConstraint::MultipleTimeKinematicConstraint(const MultipleTimeKinematicConstraint &rhs):RigidBodyConstraint(rhs)
{
}

void MultipleTimeKinematicConstraint::eval(const double* t, int n_breaks, const MatrixXd &q, VectorXd &c, MatrixXd &dc) const
{
  int num_valid_t = this->numValidTime(t,n_breaks);
  if(num_valid_t>=2)
  {
    std::vector<bool> valid_time_flag = this->isTimeValid(t,n_breaks);
    int nq = this->robot->num_positions;
    double* valid_t = new double[num_valid_t];
    MatrixXd valid_q(nq,num_valid_t);
    int valid_idx = 0;
    int* valid2tMap = new int[num_valid_t];
    for(int i = 0;i<n_breaks;i++)
    {
      if(valid_time_flag[i])
      {
        valid_t[valid_idx] = t[i];
        valid_q.col(valid_idx) = q.col(i);
        valid2tMap[valid_idx] = i;
        valid_idx++;
      }
    }
    MatrixXd dc_valid;
    this->eval_valid(valid_t,num_valid_t,valid_q,c,dc_valid);
    int nc = this->getNumConstraint(t,n_breaks);
    dc = MatrixXd::Zero(nc,nq*n_breaks);
    for(int i = 0;i<num_valid_t;i++)
    {
      dc.block(0,valid2tMap[i]*nq,nc,nq) = dc_valid.block(0,i*nq,nc,nq);
    }
    delete[] valid_t;
    delete[] valid2tMap;
  }
  else
  {
    c.resize(0);
    dc.resize(0,0);
  }
}

std::vector<bool> MultipleTimeKinematicConstraint::isTimeValid(const double* t,int n_breaks) const
{
  std::vector<bool> flag;
  for(int i = 0;i<n_breaks;i++)
  {
    if((t[i]>this->tspan[1]||t[i]<this->tspan[0]))
    {
      flag.push_back(false);
    }
    else
    {
      flag.push_back(true);
    }
  }
  return flag;
}

int MultipleTimeKinematicConstraint::numValidTime(const double* t, int n_breaks) const
{
  std::vector<bool> valid_flag = this->isTimeValid(t,n_breaks);
  int num_valid_t = 0;
  for(auto it=valid_flag.begin();it != valid_flag.end(); it++)
  {
    if(*it)
    {
      num_valid_t++;
    }
  }
  return num_valid_t;
}

void MultipleTimeKinematicConstraint::updateRobot(RigidBodyManipulator* robot)
{
  this->robot = robot;
}

PositionConstraint::PositionConstraint(RigidBodyManipulator *robot, const Matrix3Xd &pts, MatrixXd lb, MatrixXd ub, const Vector2d &tspan):SingleTimeKinematicConstraint(robot,tspan)
{
  this->n_pts = static_cast<int>(pts.cols());
  if(pts.rows() != 3)
  {
    std::cerr<<"pts must have 3 rows"<<std::endl;
  }

  this->pts = pts;
  if(lb.rows() != 3 || lb.cols() != n_pts || ub.rows() != 3 || ub.cols() != n_pts)
  {
    std::cerr<<"lb and ub must have 3 rows, the same number of columns as pts"<<std::endl;
  }

  this->null_constraint_rows.resize(3*n_pts);
  this->num_constraint = 0;
  for(int j = 0;j<n_pts;j++)
  {
    for(int i = 0;i<3;i++)
    {
      int idx = j*3+i;
      if(isnan(lb(i,j)))
      {
        lb(i,j) = -std::numeric_limits<double>::infinity();
      }
      if(isnan(ub(i,j)))
      {
        ub(i,j) = std::numeric_limits<double>::infinity();
      }
      if(ub(i,j)<lb(i,j))
      {
        std::cerr<<"Drake:PositionConstraint:BadInputs: lb must be no larger than ub"<<std::endl;
      }
      if(isinf(lb(i,j))&&isinf(ub(i,j)))
      {
        this->null_constraint_rows[idx] = true;
      }
      else
      {
        this->null_constraint_rows[idx] = false;
        this->num_constraint++;
      }
    }
  }
  this->lb.resize(this->num_constraint);
  this->ub.resize(this->num_constraint);
  int valid_row_idx = 0;
  int valid_col_idx = 0;
  int bnd_idx = 0;
  while(bnd_idx<this->num_constraint)
  {
    int idx = 3*valid_col_idx+valid_row_idx;
    if(!this->null_constraint_rows[idx])
    {
      this->lb[bnd_idx] = lb(valid_row_idx,valid_col_idx);
      this->ub[bnd_idx] = ub(valid_row_idx,valid_col_idx);
      bnd_idx++;
    }
    valid_row_idx++;
    if(valid_row_idx == 3)
    {
      valid_row_idx = 0;
      valid_col_idx++;
    }
  }
}

PositionConstraint::PositionConstraint(const PositionConstraint &rhs):SingleTimeKinematicConstraint(rhs)
{
  this->n_pts = rhs.n_pts;
  this->pts = rhs.pts;
  this->lb = rhs.lb;
  this->ub = rhs.ub;
  this->null_constraint_rows = rhs.null_constraint_rows;
}

void PositionConstraint::eval(const double* t, VectorXd &c, MatrixXd &dc) const
{
  if(this->isTimeValid(t))
  {
    Matrix3Xd pos(3,this->n_pts);
    MatrixXd J(3*this->n_pts,this->robot->num_positions);
    this->evalPositions(pos,J);
    c.resize(this->getNumConstraint(t),1);
    dc.resize(this->getNumConstraint(t),this->robot->num_positions);
    int valid_row_idx = 0;
    int i = 0;
    while(i<this->getNumConstraint(t))
    {
      if(!this->null_constraint_rows[valid_row_idx])
      {
        c(i) = pos(valid_row_idx);
        dc.row(i) = J.row(valid_row_idx);
        i++;
        valid_row_idx++;
      }
      else
      {
        valid_row_idx++;
      }
    }
  }
  else
  {
    c.resize(0);
    dc.resize(0,0);
  }
}

void PositionConstraint::bounds(const double* t,VectorXd &lb, VectorXd &ub) const
{
  if(this->isTimeValid(t))
  {
    lb = this->lb;
    ub = this->ub;
  }
}

void PositionConstraint::name(const double* t, std::vector<std::string> &name_str) const
{
  if(this->isTimeValid(t))
  {
    std::vector<std::string> cnst_names;
    this->evalNames(t,cnst_names);
    for(int i = 0;i<3*this->n_pts;i++)
    {
      if(!this->null_constraint_rows[i])
      {
        name_str.push_back(cnst_names.at(i));
      }
    }
  }
}

WorldPositionConstraint::WorldPositionConstraint(RigidBodyManipulator *robot, int body, const Matrix3Xd &pts, MatrixXd lb, MatrixXd ub, const Vector2d &tspan):PositionConstraint(robot,pts,lb,ub,tspan)
{
  this->body = body;
  this->body_name = robot->getBodyOrFrameName(body);
  this->type = RigidBodyConstraint::WorldPositionConstraintType;
}

void WorldPositionConstraint::evalPositions(Matrix3Xd &pos, MatrixXd &J) const
{
  auto pos_gradientvar = robot->forwardKinNew(pts, body, 0, 0, 1);
  pos = pos_gradientvar.value();
  J = pos_gradientvar.gradient().value();
}

void WorldPositionConstraint::evalNames(const double* t, std::vector<std::string>& cnst_names) const
{
  std::string time_str;
  if(t!=nullptr)
  {
    char time_str_buffer[20];
    sprintf(time_str_buffer,"at time %5.2f",*t);
    time_str+=std::string(time_str_buffer);
  }
  char cnst_name_str_buffer[500];
  for(int i = 0;i<this->n_pts;i++)
  {
    sprintf(cnst_name_str_buffer,"%s pts(:,%d) x %s",this->body_name.c_str(),i+1,time_str.c_str());
    cnst_names.push_back(std::string(cnst_name_str_buffer));
    sprintf(cnst_name_str_buffer,"%s pts(:,%d) y %s",this->body_name.c_str(),i+1,time_str.c_str());
    cnst_names.push_back(std::string(cnst_name_str_buffer));
    sprintf(cnst_name_str_buffer,"%s pts(:,%d) z %s",this->body_name.c_str(),i+1,time_str.c_str());
    cnst_names.push_back(std::string(cnst_name_str_buffer));
  }
}


WorldPositionConstraint::~WorldPositionConstraint()
{
}



const std::set<int> WorldCoMConstraint::defaultRobotNumSet(DrakeRigidBodyConstraint::WorldCoMDefaultRobotNum,DrakeRigidBodyConstraint::WorldCoMDefaultRobotNum+1);

WorldCoMConstraint::WorldCoMConstraint(RigidBodyManipulator *robot, Vector3d lb, Vector3d ub, const Vector2d &tspan, const std::set<int> &robotnum):PositionConstraint(robot,DrakeRigidBodyConstraint::com_pts,lb,ub,tspan)
{
  this->m_robotnum = robotnum;
  this->body = -1;
  this->body_name = "CoM";
  this->type = RigidBodyConstraint::WorldCoMConstraintType;
}

void WorldCoMConstraint::evalPositions(Matrix3Xd &pos, MatrixXd &J) const
{
  auto com_gradientvar = robot->centerOfMass<double>(1, m_robotnum);
  pos = com_gradientvar.value();
  J = com_gradientvar.gradient().value();
}


void WorldCoMConstraint::evalNames(const double* t, std::vector<std::string> &cnst_names) const
{
  std::string time_str;
  if(t!=nullptr)
  {
    char time_str_buffer[20];
    sprintf(time_str_buffer,"at time %5.2f",*t);
    time_str+=std::string(time_str_buffer);
  }
  char cnst_name_str_buffer[500];
  sprintf(cnst_name_str_buffer,"CoM x %s",time_str.c_str());
  cnst_names.push_back(std::string(cnst_name_str_buffer));
  sprintf(cnst_name_str_buffer,"CoM y %s",time_str.c_str());
  cnst_names.push_back(std::string(cnst_name_str_buffer));
  sprintf(cnst_name_str_buffer,"CoM z %s",time_str.c_str());
  cnst_names.push_back(std::string(cnst_name_str_buffer));
}


void WorldCoMConstraint::updateRobotnum(const std::set<int> &robotnum)
{
  this->m_robotnum = robotnum;
}

WorldCoMConstraint::~WorldCoMConstraint()
{
}

RelativePositionConstraint::RelativePositionConstraint(RigidBodyManipulator* robot, const Matrix3Xd &pts, const MatrixXd &lb, const MatrixXd &ub, int bodyA_idx, int bodyB_idx, const Matrix<double,7,1> &bTbp, const Vector2d &tspan):PositionConstraint(robot,pts,lb,ub,tspan)
{
  this->bTbp = bTbp;
  this->bodyA_idx = bodyA_idx;
  this->bodyB_idx = bodyB_idx;
  this->bodyA_name = this->robot->bodies[this->bodyA_idx]->linkname;
  this->bodyB_name = this->robot->bodies[this->bodyB_idx]->linkname;
  Vector4d bpTb_quat = quatConjugate(bTbp.block(3,0,4,1));
  Vector3d bpTb_trans = quatRotateVec(bpTb_quat,-bTbp.block(0,0,3,1));
  this->bpTb << bpTb_trans, bpTb_quat;
  this->type = RigidBodyConstraint::RelativePositionConstraintType;
}

void RelativePositionConstraint::evalPositions(Matrix3Xd &pos, MatrixXd &J) const
{
  int nq = this->robot->num_positions;

  auto bodyA_pos_gradientvar = robot->forwardKinNew(pts, bodyA_idx, 0, 0, 1);
  const auto& bodyA_pos = bodyA_pos_gradientvar.value();
  const auto& JA = bodyA_pos_gradientvar.gradient().value();

  auto wTb_gradientvar = robot->forwardKinNew(Vector3d::Zero().eval(), bodyB_idx, bodyB_idx, 2, 1);
  const auto& wTb = wTb_gradientvar.value();
  const auto& dwTb = wTb_gradientvar.gradient().value();

  Vector4d bTw_quat = quatConjugate(wTb.block(3,0,4,1));
  Matrix4d dbTw_quat = dquatConjugate();
  MatrixXd dbTw_quatdq = dbTw_quat*dwTb.block(3,0,4,nq);
  Vector3d bTw_trans = quatRotateVec(bTw_quat,-wTb.block(0,0,3,1));
  Matrix<double,3,7> dbTw_trans = dquatRotateVec(bTw_quat,-wTb.block(0,0,3,1));

  MatrixXd dbTw_transdq(3,nq);
  dbTw_transdq = dbTw_trans.block(0,0,3,4)*dbTw_quatdq-dbTw_trans.block(0,4,3,3)*dwTb.block(0,0,3,nq);

  Vector3d bpTw_trans1 = quatRotateVec(this->bpTb.block(3,0,4,1),bTw_trans);
  Matrix<double,3,7> dbpTw_trans1 = dquatRotateVec(this->bpTb.block(3,0,4,1),bTw_trans);

  MatrixXd dbpTw_trans1dq = dbpTw_trans1.block(0,4,3,3)*dbTw_transdq;
  Vector3d bpTw_trans = bpTw_trans1+this->bpTb.block(0,0,3,1);
  MatrixXd dbpTw_transdq = dbpTw_trans1dq;
  Vector4d bpTw_quat = quatProduct(this->bpTb.block(3,0,4,1),bTw_quat);
  Matrix<double,4,8> dbpTw_quat = dquatProduct(this->bpTb.block(3,0,4,1),bTw_quat);
  MatrixXd dbpTw_quatdq = dbpTw_quat.block(0,4,4,4)*dbTw_quatdq;

  pos.resize(3,this->n_pts);
  J.resize(3*this->n_pts,nq);
  for(int i = 0;i<this->n_pts;i++)
  {
    Vector3d bp_bodyA_pos1 = quatRotateVec(bpTw_quat,bodyA_pos.col(i));
    Matrix<double,3,7> dbp_bodyA_pos1 = dquatRotateVec(bpTw_quat,bodyA_pos.col(i));

    MatrixXd dbp_bodyA_pos1dq = dbp_bodyA_pos1.block(0,0,3,4)*dbpTw_quatdq+dbp_bodyA_pos1.block(0,4,3,3)*JA.block(3*i,0,3,nq);
    pos.col(i) = bp_bodyA_pos1+bpTw_trans;
    J.block(3*i,0,3,nq) = dbp_bodyA_pos1dq+dbpTw_transdq;
  }
}

void RelativePositionConstraint::evalNames(const double* t, std::vector<std::string> &cnst_names) const
{
  std::string time_str;
  if(t!=nullptr)
  {
    char time_str_buffer[20];
    sprintf(time_str_buffer,"at time %5.2f",*t);
    time_str+=std::string(time_str_buffer);
  }
  char cnst_name_str_buffer[500];
  for(int i = 0;i<this->n_pts;i++)
  {
    sprintf(cnst_name_str_buffer,"%s pts(:,%d) in %s x %s",this->bodyA_name.c_str(),i+1,this->bodyB_name.c_str(),time_str.c_str());
    cnst_names.push_back(std::string(cnst_name_str_buffer));
    sprintf(cnst_name_str_buffer,"%s pts(:,%d) in %s y %s",this->bodyA_name.c_str(),i+1,this->bodyB_name.c_str(),time_str.c_str());
    cnst_names.push_back(std::string(cnst_name_str_buffer));
    sprintf(cnst_name_str_buffer,"%s pts(:,%d) in %s z %s",this->bodyA_name.c_str(),i+1,this->bodyB_name.c_str(),time_str.c_str());
    cnst_names.push_back(std::string(cnst_name_str_buffer));
  }
}

RelativePositionConstraint::~RelativePositionConstraint()
{
}

QuatConstraint::QuatConstraint(RigidBodyManipulator *robot, double tol, const Vector2d &tspan):SingleTimeKinematicConstraint(robot,tspan)
{
  if(tol<0.0 || tol>M_PI)
  {
    std::cerr<<"tol must be within [0 PI]"<<std::endl;
  }
  this->tol = tol;
  this->num_constraint = 1;
}

void QuatConstraint::eval(const double* t, VectorXd &c, MatrixXd &dc) const
{
  int num_constraint = this->getNumConstraint(t);
  c.resize(num_constraint);
  dc.resize(num_constraint,this->robot->num_positions);
  if(this->isTimeValid(t))
  {
    double prod;
    MatrixXd dprod(1,this->robot->num_positions);
    this->evalOrientationProduct(prod,dprod);
    c(0) = 2.0*prod*prod-1.0;
    dc = 4.0*prod*dprod;
  }
  else
  {
    c.resize(0);
    dc.resize(0,0);
  }
}

void QuatConstraint::bounds(const double* t, VectorXd &lb, VectorXd &ub) const
{
  lb.resize(this->getNumConstraint(t));
  ub.resize(this->getNumConstraint(t));
  if(this->isTimeValid(t))
  {
    lb[0] = cos(this->tol);
    ub[0] = 1.0;
  }
}

QuatConstraint::~QuatConstraint(void)
{
}

WorldQuatConstraint::WorldQuatConstraint(RigidBodyManipulator *robot, int body, const Vector4d &quat_des, double tol, const Vector2d &tspan):QuatConstraint(robot,tol,tspan)
{
  this->body = body;
  this->body_name = robot->getBodyOrFrameName(body);
  if(quat_des.norm()<=0)
  {
    std::cerr<<"quat_des must be non-zero"<<std::endl;
  }
  this->quat_des = quat_des/quat_des.norm();
  this->type = RigidBodyConstraint::WorldQuatConstraintType;
}

void WorldQuatConstraint::evalOrientationProduct(double &prod, MatrixXd &dprod) const
{
  Vector3d pts = Vector3d::Zero();
  Vector3d pts;
  auto x_gradientvar = robot->forwardKinNew(pts, body, 0, 2, 1);
  const auto& x = x_gradientvar.value();
  const auto& J = x_gradientvar.gradient().value();

  Vector4d quat = x.tail(4);
  prod = (quat.transpose()*this->quat_des);
  dprod = this->quat_des.transpose()*J.block(3,0,4,this->robot->num_positions);
}


void WorldQuatConstraint::name(const double* t, std::vector<std::string> &name_str) const
{
  if(this->isTimeValid(t))
  {
    char cnst_name_str_buffer[500];
    std::string time_str;
    char time_str_buffer[30];
    if(t != nullptr)
    {
      sprintf(time_str_buffer,"at time %5.2f",*t);
      time_str+= std::string(time_str_buffer);
    }
    sprintf(cnst_name_str_buffer,"%s quaternion constraint %s",this->body_name.c_str(),time_str.c_str());
    std::string cnst_name_str(cnst_name_str_buffer);
    for(int i = 0;i<this->num_constraint;i++)
    {
      name_str.push_back(cnst_name_str);
    }
  }
}


WorldQuatConstraint::~WorldQuatConstraint()
{
}

RelativeQuatConstraint::RelativeQuatConstraint(RigidBodyManipulator* robot, int bodyA_idx, int bodyB_idx, const Vector4d &quat_des, double tol, const Vector2d &tspan):QuatConstraint(robot,tol,tspan)
{
  this->bodyA_idx = bodyA_idx;
  this->bodyB_idx = bodyB_idx;
  this->bodyA_name = this->robot->bodies[bodyA_idx]->linkname;
  this->bodyB_name = this->robot->bodies[bodyB_idx]->linkname;
  double quat_norm = quat_des.norm();
  this->quat_des = quat_des/quat_norm;
  this->type = RigidBodyConstraint::RelativeQuatConstraintType;
}

void RelativeQuatConstraint::evalOrientationProduct(double &prod, MatrixXd &dprod) const
{
  int nq = this->robot->num_positions;
  Vector3d origin_pt = Vector3d::Zero();
  auto pos_a_gradientvar = robot->forwardKinNew(origin_pt, bodyA_idx, 0, 2, 1);
  const auto& pos_a = pos_a_gradientvar.value();
  const auto& J_a = pos_a_gradientvar.gradient().value();
  auto pos_b_gradientvar = robot->forwardKinNew(origin_pt, bodyB_idx, 0, 2, 1);
  const auto& pos_b = pos_b_gradientvar.value();
  const auto& J_b = pos_a_gradientvar.gradient().value();

  Vector4d quat_a2w = pos_a.block(3,0,4,1);
  MatrixXd dquat_a2w = J_a.block(3,0,4,nq);
  Vector4d quat_b2w = pos_b.block(3,0,4,1);
  MatrixXd dquat_b2w = J_b.block(3,0,4,nq);
  Vector4d quat_w2b = quatConjugate(quat_b2w);
  Matrix<double,4,4> dquat_w2b = dquatConjugate();
  MatrixXd dquat_w2bdq = dquat_w2b*dquat_b2w;

  Vector4d quat_a2b = quatProduct(quat_w2b,quat_a2w);
  Matrix<double,4,8> dquat_a2b = dquatProduct(quat_w2b,quat_a2w);

  MatrixXd dquat_a2bdq = dquat_a2b.block(0,0,4,4)*dquat_w2bdq+dquat_a2b.block(0,4,4,4)*dquat_a2w;

  prod = quat_a2b.dot(this->quat_des);
  dprod = this->quat_des.transpose()*dquat_a2bdq;
}

void RelativeQuatConstraint::name(const double* t, std::vector<std::string> &name_str) const
{
  if(this->isTimeValid(t))
  {
    char cnst_name_str_buffer[500];
    std::string time_str;
    char time_str_buffer[30];
    if(t != nullptr)
    {
      sprintf(time_str_buffer,"at time %5.2f",*t);
      time_str+= std::string(time_str_buffer);
    }
    sprintf(cnst_name_str_buffer,"%s relative to %s quaternion constraint %s",this->bodyA_name.c_str(),this->bodyB_name.c_str(),time_str.c_str());
    std::string cnst_name_str(cnst_name_str_buffer);
    for(int i = 0;i<this->num_constraint;i++)
    {
      name_str.push_back(cnst_name_str);
    }
  }

}

RelativeQuatConstraint::~RelativeQuatConstraint()
{
}

EulerConstraint::EulerConstraint(RigidBodyManipulator *robot, const Vector3d &lb, const Vector3d &ub, const Vector2d &tspan):SingleTimeKinematicConstraint(robot,tspan)
{
  this->num_constraint = 0;
  Vector3d my_lb = lb, my_ub = ub;
  for(int i = 0;i<3;i++)
  {
    if(isnan(my_lb(i)))
    {
      my_lb(i) = -std::numeric_limits<double>::infinity();
    }
    if(isnan(my_ub(i)))
    {
      my_ub(i) = std::numeric_limits<double>::infinity();
    }
    if(my_ub(i)<my_lb(i))
    {
      std::cerr<<"Drake:EulerConstraint:BadInputs:lb must be no larger than ub"<<std::endl;;
    }
    if(isinf(my_lb(i))&&isinf(my_ub(i)))
    {
      null_constraint_rows[i] = true;
    }
    else
    {
      null_constraint_rows[i] = false;
      this->num_constraint++;
    }
  }
  this->lb.resize(this->num_constraint);
  this->ub.resize(this->num_constraint);
  int valid_row_idx = 0;
  int bnd_idx = 0;
  while(bnd_idx<this->num_constraint)
  {
    if(!this->null_constraint_rows[valid_row_idx])
    {
      this->lb[bnd_idx] = my_lb(valid_row_idx);
      this->ub[bnd_idx] = my_ub(valid_row_idx);
      bnd_idx++;
      valid_row_idx++;
    }
    else
    {
      valid_row_idx++;
    }
  }
  this->avg_rpy.resize(this->num_constraint);
  for(int i = 0;i<this->num_constraint;i++)
  {
    this->avg_rpy[i] = (this->lb[i]+this->ub[i])/2.0;
  }
}

EulerConstraint::EulerConstraint(const EulerConstraint &rhs):SingleTimeKinematicConstraint(rhs)
{
  this->null_constraint_rows[0] = rhs.null_constraint_rows[0];
  this->null_constraint_rows[1] = rhs.null_constraint_rows[1];
  this->null_constraint_rows[2] = rhs.null_constraint_rows[2];
  this->lb = rhs.lb;
  this->ub = rhs.ub;
  this->avg_rpy = rhs.avg_rpy;
}

void EulerConstraint::eval(const double* t, VectorXd &c, MatrixXd &dc) const
{
  int n_constraint = this->getNumConstraint(t);
  if(this->isTimeValid(t))
  {
    Vector3d rpy;
    MatrixXd drpy(3,this->robot->num_positions);
    this->evalrpy(rpy,drpy);
    c.resize(n_constraint);
    dc.resize(n_constraint,this->robot->num_positions);
    int valid_row_idx = 0;
    int i = 0;
    while(i<n_constraint)
    {
      if(!this->null_constraint_rows[valid_row_idx])
      {
        c(i) = rpy(valid_row_idx);
        c(i) = angleDiff(this->avg_rpy[i],c(i))+this->avg_rpy[i];
        dc.row(i) = drpy.row(valid_row_idx);
        valid_row_idx++;
        i++;
      }
      else
      {
        valid_row_idx++;
      }
    }
  }
  else
  {
    c.resize(0);
    dc.resize(0,0);
  }
}

void EulerConstraint::bounds(const double* t, VectorXd &lb, VectorXd &ub) const
{
  if(this->isTimeValid(t))
  {
    lb = this->lb;
    ub = this->ub;
  }
}

WorldEulerConstraint::WorldEulerConstraint(RigidBodyManipulator *robot, int body, const Vector3d &lb, const Vector3d &ub, const Vector2d &tspan): EulerConstraint(robot,lb,ub,tspan)
{
  this->body = body;
  this->body_name = robot->getBodyOrFrameName(body);
  this->type = RigidBodyConstraint::WorldEulerConstraintType;
}

void WorldEulerConstraint::evalrpy(Vector3d &rpy,MatrixXd &J) const
{
  auto x_gradientvar = robot->forwardKinNew(Vector3d::Zero().eval(), body, 0, 1, 1);
  const auto& x = x_gradientvar.value();
  const auto& dx = x_gradientvar.gradient().value();

  rpy = x.tail(3);
  J = dx.block(3,0,3,this->robot->num_positions);
}

void WorldEulerConstraint::name(const double* t, std::vector<std::string> &name_str) const
{
  if(this->isTimeValid(t))
  {
    char cnst_name_str_buffer[500];
    int constraint_idx = 0;
    if(!this->null_constraint_rows[0])
    {
      if(t == nullptr)
      {
        sprintf(cnst_name_str_buffer,"%s roll",this->body_name.c_str());
      }
      else
      {
        sprintf(cnst_name_str_buffer,"%s roll at time %10.4f",this->body_name.c_str(),*t);
      }
      std::string cnst_name_str(cnst_name_str_buffer);
      name_str.push_back(cnst_name_str);
      constraint_idx++;
    }
    if(!this->null_constraint_rows[1])
    {
      if(t == nullptr)
      {
        sprintf(cnst_name_str_buffer,"%s pitch",this->body_name.c_str());
      }
      else
      {
        sprintf(cnst_name_str_buffer,"%s pitch at time %10.4f",this->body_name.c_str(),*t);
      }
      std::string cnst_name_str(cnst_name_str_buffer);
      name_str.push_back(cnst_name_str);
      constraint_idx++;
    }
    if(!this->null_constraint_rows[2])
    {
      if(t == nullptr)
      {
        sprintf(cnst_name_str_buffer,"%s yaw",this->body_name.c_str());
      }
      else
      {
        sprintf(cnst_name_str_buffer,"%s yaw at time %10.4f",this->body_name.c_str(),*t);
      }
      std::string cnst_name_str(cnst_name_str_buffer);
      name_str.push_back(cnst_name_str);
      constraint_idx++;
    }
  }
}


WorldEulerConstraint::~WorldEulerConstraint()
{
}

GazeConstraint::GazeConstraint(RigidBodyManipulator* robot, const Vector3d &axis, double conethreshold, const Vector2d &tspan):SingleTimeKinematicConstraint(robot,tspan)
{
  double len_axis = axis.norm();
  if(len_axis<=0)
  {
    std::cerr<<"axis must be non-zero"<<std::endl;
  }
  this->axis = axis/len_axis;
  if(conethreshold < 0.0 || conethreshold >M_PI+1E-10)
  {
    std::cerr<<"conethreshold should be within [0 PI]"<<std::endl;
  }
  this->conethreshold = conethreshold;
}

GazeOrientConstraint::GazeOrientConstraint(RigidBodyManipulator* robot, const Vector3d &axis, const Vector4d &quat_des, double conethreshold, double threshold, const Vector2d &tspan): GazeConstraint(robot,axis,conethreshold,tspan)
{
  double len_quat_des = quat_des.norm();
  if(len_quat_des<=0)
  {
    std::cerr<<"quat_des must be non-zero"<<std::endl;
  }
  this->quat_des = quat_des/len_quat_des;
  if(threshold < 0.0 || threshold >M_PI+1E-10)
  {
    std::cerr<<"threshold should be within [0 PI]"<<std::endl;
  }
  this->threshold = threshold;
  this->num_constraint = 2;
}

void GazeOrientConstraint::eval(const double* t, VectorXd &c, MatrixXd &dc) const
{
  int num_constraint = this->getNumConstraint(t);
  c.resize(num_constraint);
  dc.resize(num_constraint,this->robot->num_positions);
  if(this->isTimeValid(t))
  {
    Vector4d quat;
    int nq = this->robot->num_positions;
    MatrixXd dquat(4,nq);
    this->evalOrientation(quat,dquat);
    double axis_err = quatDiffAxisInvar(quat,this->quat_des,this->axis);
    Matrix<double,1,11> daxis_err = dquatDiffAxisInvar(quat,this->quat_des,this->axis);

    MatrixXd daxis_err_dq(1,nq);
    daxis_err_dq = daxis_err.block(0,0,1,4)*dquat;
    Vector4d q_diff = quatDiff(quat,this->quat_des);
    Matrix<double,4,8> dq_diff = dquatDiff(quat,this->quat_des);

    MatrixXd dq_diff_dq(4,nq);
    dq_diff_dq = dq_diff.block(0,0,4,4)*dquat;
    c << axis_err, q_diff(0);
    dc.row(0) = daxis_err_dq;
    dc.row(1) = dq_diff_dq.row(0);
  }
  else
  {
    c.resize(0);
    dc.resize(0,0);
  }
}

void GazeOrientConstraint::bounds(const double* t, VectorXd &lb, VectorXd &ub) const
{
  lb.resize(this->getNumConstraint(t));
  ub.resize(this->getNumConstraint(t));
  if(this->isTimeValid(t))
  {
    lb << cos(this->conethreshold)-1.0,cos(this->threshold/2.0);
    ub << 0, std::numeric_limits<double>::infinity();
  }
}

WorldGazeOrientConstraint::WorldGazeOrientConstraint(RigidBodyManipulator* robot, int body, const Vector3d &axis, const Vector4d &quat_des,double conethreshold, double threshold, const Vector2d &tspan): GazeOrientConstraint(robot,axis,quat_des,conethreshold,threshold,tspan)
{
  this->body = body;
  this->body_name = robot->getBodyOrFrameName(body);
  this->type = RigidBodyConstraint::WorldGazeOrientConstraintType;
}

void WorldGazeOrientConstraint::evalOrientation(Vector4d &quat, MatrixXd &dquat_dq) const
{
  auto x_gradientvar = robot->forwardKinNew(Vector3d::Zero().eval(), body, 0, 2, 1);
  const auto& x = x_gradientvar.value();
  const auto& J = x_gradientvar.gradient().value();
  quat = x.tail(4);
  dquat_dq = J.block(3,0,4,this->robot->num_positions);
}


void WorldGazeOrientConstraint::name(const double* t, std::vector<std::string> &name_str) const
{
  if(this->isTimeValid(t))
  {
    char cnst_name_str_buffer[500];
    std::string cnst_name_str;
    if(t == nullptr)
    {
      sprintf(cnst_name_str_buffer,"%s conic gaze orientation constraint",this->body_name.c_str());
      cnst_name_str = std::string(cnst_name_str_buffer);
      name_str.push_back(cnst_name_str);
      sprintf(cnst_name_str_buffer,"%s revolute gaze orientation constraint",this->body_name.c_str());
      cnst_name_str = std::string(cnst_name_str_buffer);
      name_str.push_back(cnst_name_str);
    }
    else
    {
      sprintf(cnst_name_str_buffer,"%s conic gaze orientation constraint at time %10.4f",this->body_name.c_str(),*t);
      cnst_name_str = std::string(cnst_name_str_buffer);
      name_str.push_back(cnst_name_str);
      sprintf(cnst_name_str_buffer,"%s revolute gaze orientation constraint at time %10.4f",this->body_name.c_str(),*t);
      cnst_name_str = std::string(cnst_name_str_buffer);
      name_str.push_back(cnst_name_str);
    }
  }
}


GazeDirConstraint::GazeDirConstraint(RigidBodyManipulator* robot, const Vector3d &axis, const Vector3d &dir, double conethreshold, const Vector2d &tspan):GazeConstraint(robot,axis,conethreshold,tspan)
{
  double len_dir = dir.norm();
  if(len_dir<= 0)
  {
    std::cerr<<"dir should be non-zero"<<std::endl;
  }
  this->dir = dir/len_dir;
  this->num_constraint = 1;
}


void GazeDirConstraint::bounds(const double* t, VectorXd &lb, VectorXd &ub) const
{
  int num_constraint = this->getNumConstraint(t);
  lb.resize(num_constraint);
  ub.resize(num_constraint);
  if(this->isTimeValid(t))
  {
    lb[0] = cos(this->conethreshold)-1.0;
    ub[0] = 0.0;
  }
}

WorldGazeDirConstraint::WorldGazeDirConstraint(RigidBodyManipulator *robot, int body, const Vector3d &axis, const Vector3d &dir, double conethreshold, const Vector2d &tspan): GazeDirConstraint(robot,axis,dir,conethreshold,tspan)
{
  this->body = body;
  this->body_name = robot->getBodyOrFrameName(body);
  this->type = RigidBodyConstraint::WorldGazeDirConstraintType;
}


void WorldGazeDirConstraint::eval(const double* t, VectorXd &c, MatrixXd &dc) const
{
  if(this->isTimeValid(t))
  {
    Matrix3Xd body_axis_ends(3, 2);
    body_axis_ends.col(0).setZero();
    body_axis_ends.col(1) = this->axis;
    int nq = this->robot->num_positions;
    auto axis_pos_gradientvar = robot->forwardKinNew(body_axis_ends, body, 0, 0, 1);
    const auto &axis_pos = axis_pos_gradientvar.value();
    const auto &daxis_pos = axis_pos_gradientvar.gradient().value();
    Vector3d axis_world = axis_pos.col(1) - axis_pos.col(0);
    MatrixXd daxis_world = daxis_pos.block(3, 0, 3, nq) - daxis_pos.block(0, 0, 3, nq);
    c.resize(1);
    c(0) = axis_world.dot(this->dir) - 1.0;
    dc = this->dir.transpose()*daxis_world;
  }
  else
  {
    c.resize(0);
    dc.resize(0,0);
  }
}

void WorldGazeDirConstraint::name(const double* t, std::vector<std::string> &name_str) const
{
  if(this->isTimeValid(t))
  {
    char cnst_name_str_buffer[500];
    if(t == nullptr)
    {
      sprintf(cnst_name_str_buffer,"%s conic gaze direction constraint",this->body_name.c_str());
    }
    else
    {
      sprintf(cnst_name_str_buffer,"%s conic gaze direction constraint at time %10.4f",this->body_name.c_str(),*t);
    }
    std::string cnst_name_str(cnst_name_str_buffer);
    name_str.push_back(cnst_name_str);
  }
}

GazeTargetConstraint::GazeTargetConstraint(RigidBodyManipulator* robot, const Vector3d &axis, const Vector3d &target, const Vector3d &gaze_origin, double conethreshold, const Vector2d &tspan):GazeConstraint(robot,axis,conethreshold,tspan)
{
  this->target = target;
  this->gaze_origin = gaze_origin;
  this->num_constraint = 1;
}

void GazeTargetConstraint::bounds(const double* t, VectorXd &lb, VectorXd &ub) const
{
  int num_constraint = this->getNumConstraint(t);
  lb.resize(num_constraint);
  ub.resize(num_constraint);
  if(this->isTimeValid(t))
  {
    lb[0] = cos(this->conethreshold)-1.0;
    ub[0] = 0.0;
  }
}

WorldGazeTargetConstraint::WorldGazeTargetConstraint(RigidBodyManipulator* robot, int body, const Vector3d &axis, const Vector3d &target, const Vector3d &gaze_origin, double conethreshold, const Vector2d &tspan): GazeTargetConstraint(robot,axis,target,gaze_origin,conethreshold,tspan)
{
  this->body = body;
  this->body_name = robot->getBodyOrFrameName(body);
  this->type = RigidBodyConstraint::WorldGazeTargetConstraintType;
}

void WorldGazeTargetConstraint::eval(const double* t,VectorXd &c, MatrixXd &dc) const
{
  int num_constraint = this->getNumConstraint(t);
  int nq = this->robot->num_positions;
  c.resize(num_constraint);
  dc.resize(num_constraint,nq);
  if(this->isTimeValid(t))
  {
    Matrix3Xd body_axis_ends(3, 2);
    body_axis_ends.col(0) = this->gaze_origin;
    body_axis_ends.col(1) = this->gaze_origin + this->axis;
    int nq = this->robot->num_positions;
    auto axis_ends_gradientvar = robot->forwardKinNew(body_axis_ends, body, 0, 0, 1);
    const auto &axis_ends = axis_ends_gradientvar.value();
    const auto &daxis_ends = axis_ends_gradientvar.gradient().value();

    Vector3d world_axis = axis_ends.col(1) - axis_ends.col(0);
    MatrixXd dworld_axis = daxis_ends.block(3, 0, 3, nq) - daxis_ends.block(0, 0, 3, nq);
    Vector3d dir = this->target - axis_ends.col(0);
    MatrixXd ddir = -daxis_ends.block(0, 0, 3, nq);
    double dir_norm = dir.norm();
    Vector3d dir_normalized = dir / dir_norm;
    MatrixXd ddir_normalized = (MatrixXd::Identity(3, 3) * dir_norm * dir_norm - dir * dir.transpose()) / (std::pow(dir_norm, 3)) * ddir;
    c(0) = dir_normalized.dot(world_axis) - 1.0;
    dc = dir_normalized.transpose()*dworld_axis+world_axis.transpose()*ddir_normalized;
  }
  else
  {
    c.resize(0);
    dc.resize(0,0);
  }
}

void WorldGazeTargetConstraint::name(const double* t, std::vector<std::string> &name_str) const
{
  if(this->isTimeValid(t))
  {
    char cnst_name_str_buffer[500];
    if(t == nullptr)
    {
      sprintf(cnst_name_str_buffer,"%s conic gaze target constraint",this->body_name.c_str());
    }
    else
    {
      sprintf(cnst_name_str_buffer,"%s conic gaze target constraint at time %10.4f",this->body_name.c_str(),*t);
    }
    std::string cnst_name_str(cnst_name_str_buffer);
    name_str.push_back(cnst_name_str);
  }
}


RelativeGazeTargetConstraint::RelativeGazeTargetConstraint(RigidBodyManipulator *robot, int bodyA_idx, int bodyB_idx, const Eigen::Vector3d &axis, const Vector3d &target, const Vector3d &gaze_origin, double conethreshold, const Eigen::Vector2d &tspan):GazeTargetConstraint(robot,axis,target,gaze_origin,conethreshold,tspan)
{
  this->bodyA_idx = bodyA_idx;
  this->bodyB_idx = bodyB_idx;
  this->bodyA_name = this->robot->bodies[this->bodyA_idx]->linkname;
  this->bodyB_name = this->robot->bodies[this->bodyB_idx]->linkname;
  this->type = RigidBodyConstraint::RelativeGazeTargetConstraintType;
}

void RelativeGazeTargetConstraint::eval(const double* t, VectorXd &c, MatrixXd &dc) const
{
  if(this->isTimeValid(t))
  {
    int nq = this->robot->num_positions;
    auto target_pos_gradientvar = robot->forwardKinNew(target, bodyB_idx, 0, 0, 1);
    const auto& target_pos = target_pos_gradientvar.value();
    const auto& dtarget_pos = target_pos_gradientvar.gradient().value();

    auto origin_pos_gradientvar = robot->forwardKinNew(gaze_origin, bodyA_idx, 0, 0, 1);
    const auto& origin_pos = origin_pos_gradientvar.value();
    const auto& dorigin_pos = origin_pos_gradientvar.gradient().value();

    auto axis_pos_gradientvar = robot->forwardKinNew(axis, bodyA_idx, 0, 0, 1);
    const auto& axis_pos = axis_pos_gradientvar.value();
    const auto& daxis_pos = axis_pos_gradientvar.gradient().value();

    Vector3d axis_origin = Vector3d::Zero();
    auto axis_origin_pos_gradientvar = robot->forwardKinNew(axis_origin, bodyA_idx, 0, 0, 1);
    const auto& axis_origin_pos = axis_origin_pos_gradientvar.value();
    const auto& daxis_origin_pos = axis_origin_pos_gradientvar.gradient().value();

    axis_pos -= axis_origin_pos;
    daxis_pos -= daxis_origin_pos;

    Vector3d origin_to_target = target_pos - origin_pos;
    MatrixXd dorigin_to_target = dtarget_pos - dorigin_pos;
    double origin_to_target_norm = origin_to_target.norm();
    c.resize(1);
    c(0) = origin_to_target.dot(axis_pos) / origin_to_target_norm - 1.0;
    dc.resize(1, nq);
    MatrixXd dcdorigin_to_target = (axis_pos.transpose() * origin_to_target_norm - axis_pos.dot(origin_to_target) / origin_to_target_norm * origin_to_target.transpose()) / (origin_to_target_norm * origin_to_target_norm);
    MatrixXd dcdaxis_pos = origin_to_target.transpose() / origin_to_target_norm;
    dc = dcdorigin_to_target * dorigin_to_target+dcdaxis_pos*daxis_pos;
  }
  else
  {
    c.resize(0);
    dc.resize(0,0);
  }
}

void RelativeGazeTargetConstraint::name(const double* t, std::vector<std::string> &name_str) const
{
  if(this->isTimeValid(t))
  {
    char cnst_name_str_buffer[1000];
    if(t == nullptr)
    {
      sprintf(cnst_name_str_buffer,"%s relative to %s conic gaze constraint",this->bodyA_name.c_str(),this->bodyB_name.c_str());
    }
    else
    {
      sprintf(cnst_name_str_buffer,"%s relative to %s conic gaze constraint at time %10.4f",this->bodyA_name.c_str(),this->bodyB_name.c_str(),*t);
    }
    std::string cnst_name_str(cnst_name_str_buffer);
    name_str.push_back(cnst_name_str);
  }
}


RelativeGazeDirConstraint::
RelativeGazeDirConstraint(RigidBodyManipulator *robot, int bodyA_idx,
    int bodyB_idx, const Vector3d &axis, const Vector3d &dir,
    double conethreshold, const Eigen::Vector2d &tspan)
: GazeDirConstraint(robot,axis,dir,conethreshold,tspan),
  bodyA_idx(bodyA_idx), bodyB_idx(bodyB_idx)
{
  this->bodyA_name = this->robot->bodies[this->bodyA_idx]->linkname;
  this->bodyB_name = this->robot->bodies[this->bodyB_idx]->linkname;
  this->type = RigidBodyConstraint::RelativeGazeDirConstraintType;
}

void RelativeGazeDirConstraint::eval(const double* t, VectorXd &c, MatrixXd &dc) const
{
  if(this->isTimeValid(t))
  {
    Matrix3Xd body_axis_ends(3,2);
    body_axis_ends.block(0,0,3,1) = MatrixXd::Zero(3,1);
    body_axis_ends.block(0,1,3,1) = this->axis;
    Matrix3Xd body_dir_ends(3,2);
    body_dir_ends.block(0,0,3,1) = MatrixXd::Zero(3,1);
    body_dir_ends.block(0,1,3,1) = this->dir;
    int nq = this->robot->num_positions;

    auto axis_pos_gradientvar = robot->forwardKinNew(body_axis_ends, bodyA_idx, 0, 0, 1);
    const auto& axis_pos = axis_pos_gradientvar.value();
    const auto& daxis_pos = axis_pos_gradientvar.gradient().value();
    auto dir_pos_gradientvar = robot->forwardKinNew(body_dir_ends, bodyB_idx, 0, 0, 1);
    const auto& dir_pos = dir_pos_gradientvar.value();
    const auto& ddir_pos = dir_pos_gradientvar.gradient().value();

    Vector3d axis_world = axis_pos.col(1)-axis_pos.col(0);
    MatrixXd daxis_world = daxis_pos.block(3,0,3,nq)-daxis_pos.block(0,0,3,nq);
    Vector3d dir_world = dir_pos.col(1)-dir_pos.col(0);
    MatrixXd ddir_world = ddir_pos.block(3,0,3,nq)-ddir_pos.block(0,0,3,nq);
    c.resize(1);
    c(0) = axis_world.dot(dir_world)-1.0;
    dc = dir_world.transpose()*daxis_world + axis_world.transpose()*ddir_world;
  }
  else
  {
    c.resize(0);
    dc.resize(0,0);
  }
}

void RelativeGazeDirConstraint::name(const double* t, std::vector<std::string> &name_str) const
{
  if(this->isTimeValid(t))
  {
    char cnst_name_str_buffer[1000];
    if(t == nullptr)
    {
      sprintf(cnst_name_str_buffer,"%s relative to %s conic gaze constraint",this->bodyA_name.c_str(),this->bodyB_name.c_str());
    }
    else
    {
      sprintf(cnst_name_str_buffer,"%s relative to %s conic gaze constraint at time %10.4f",this->bodyA_name.c_str(),this->bodyB_name.c_str(),*t);
    }
    std::string cnst_name_str(cnst_name_str_buffer);
    name_str.push_back(cnst_name_str);
  }
}



Point2PointDistanceConstraint::Point2PointDistanceConstraint(RigidBodyManipulator *robot, int bodyA, int bodyB, const Matrix3Xd &ptA, const Matrix3Xd &ptB, const VectorXd &dist_lb, const VectorXd &dist_ub, const Vector2d &tspan): SingleTimeKinematicConstraint(robot,tspan)
{
  this->bodyA = bodyA;
  this->bodyB = bodyB;
  this->ptA = ptA;
  this->ptB = ptB;
  this->num_constraint = static_cast<int>(ptA.cols());
  this->dist_lb = dist_lb;
  this->dist_ub = dist_ub;
  this->type = RigidBodyConstraint::Point2PointDistanceConstraintType;
}

void Point2PointDistanceConstraint::eval(const double* t, VectorXd &c, MatrixXd &dc) const
{
  if(this->isTimeValid(t))
  {
    int num_cnst = this->getNumConstraint(t);
    MatrixXd posA(3,this->ptA.cols());
    MatrixXd dposA(3*this->ptA.cols(),this->robot->num_positions);
    if(this->bodyA != 0)
    {
      auto posA_gradientvar = robot->forwardKinNew(ptA, bodyA, 0, 0, 1);
      posA = posA_gradientvar.value();
      dposA = posA_gradientvar.gradient().value();
    }
    else
    {
      posA = this->ptA.block(0,0,3,this->ptA.cols());
      dposA = MatrixXd::Zero(3*this->ptA.cols(),this->robot->num_positions);
    }
    MatrixXd posB(3,this->ptB.cols());
    MatrixXd dposB(3*this->ptB.cols(),this->robot->num_positions);
    if(this->bodyB != 0)
    {
      auto posB_gradientvar = robot->forwardKinNew(ptA, bodyB, 0, 0, 1);
      posB = posB_gradientvar.value();
      dposB = posB_gradientvar.gradient().value();
    }
    else
    {
      posB = this->ptB.block(0,0,3,this->ptB.cols());
      dposB = MatrixXd::Zero(3*this->ptB.cols(),this->robot->num_positions);
    }
    MatrixXd d = posA-posB;
    MatrixXd dd = dposA-dposB;
    MatrixXd tmp1 = d.cwiseProduct(d);
    MatrixXd tmp2 = tmp1.colwise().sum();
    c.resize(num_cnst,1);
    c = tmp2.transpose();
    dc.resize(num_cnst,this->robot->num_positions);
    for(int i = 0;i<num_cnst;i++)
    {
      dc.row(i) = 2*d.col(i).transpose()*dd.block(3*i,0,3,this->robot->num_positions);
    }
  }
  else
  {
    c.resize(0);
    dc.resize(0,0);
  }
}

void Point2PointDistanceConstraint::name(const double* t, std::vector<std::string> &name_str) const
{
  if(this->isTimeValid(t))
  {
    int num_cnst = this->getNumConstraint(t);
    for(int i = 0;i<num_cnst;i++)
    {
      char cnst_name_buffer[1000];
      std::string bodyA_name;
      if(this->bodyA != 0)
      {
        bodyA_name = this->robot->bodies[bodyA]->linkname;
      }
      else
      {
        bodyA_name = "World";
      }
      std::string bodyB_name;
      if(this->bodyB != 0)
      {
        bodyB_name = this->robot->bodies[bodyB]->linkname;
      }
      else
      {
        bodyB_name = "World";
      }
      sprintf(cnst_name_buffer,"Distance from %s pt %d to %s pt %d",bodyA_name.c_str(),i,bodyB_name.c_str(),i);
      std::string cnst_name_str(cnst_name_buffer);
      name_str.push_back(cnst_name_str);
    }
  }
}

void Point2PointDistanceConstraint::bounds(const double* t, VectorXd &lb, VectorXd &ub) const
{
  if(this->isTimeValid(t))
  {
    lb = this->dist_lb.cwiseProduct(this->dist_lb);
    ub = this->dist_ub.cwiseProduct(this->dist_ub);
  }
  else
  {
    lb.resize(0);
    ub.resize(0);
  }
}



Point2LineSegDistConstraint::Point2LineSegDistConstraint(RigidBodyManipulator* robot, int pt_body, const Vector3d &pt, int line_body, const Matrix<double,3,2> &line_ends, double dist_lb, double dist_ub, const Vector2d &tspan):SingleTimeKinematicConstraint(robot,tspan)
{
  this->pt_body = pt_body;
  this->pt = pt;
  this->line_body = line_body;
  this->line_ends = line_ends;
  this->dist_lb = dist_lb;
  this->dist_ub = dist_ub;
  this->num_constraint = 2;
  this->type = RigidBodyConstraint::Point2LineSegDistConstraintType;
}

void Point2LineSegDistConstraint::eval(const double* t, VectorXd &c, MatrixXd &dc) const
{
  if(this->isTimeValid(t))
  {
    int nq = this->robot->num_positions;

    auto pt_pos_gradientvar = robot->forwardKinNew(pt, pt_body, 0, 0, 1);
    const auto& pt_pos = pt_pos_gradientvar.value();
    const auto& J_pt = pt_pos_gradientvar.gradient().value();

    auto line_pos_gradientvar = robot->forwardKinNew(line_ends, line_body, 0, 0, 1);
    const auto& line_pos = line_pos_gradientvar.value();
    const auto& J_line = line_pos_gradientvar.gradient().value();

    Vector3d x0 = pt_pos;
    Vector3d x1 = line_pos.col(0);
    Vector3d x2 = line_pos.col(1);
    Vector3d x21 = x2-x1;
    MatrixXd J21 = J_line.block(3,0,3,nq)-J_line.block(0,0,3,nq);
    Vector3d x10 = x1-x0;
    MatrixXd J10 = J_line.block(0,0,3,nq)-J_pt;
    double x21_square = x21.squaredNorm();
    double t = -x10.dot(x21)/x21_square;
    MatrixXd dtdx10 = -x21.transpose()/x21_square;
    MatrixXd dtdx21 = -(x10.transpose()*x21_square-x10.dot(x21)*2.0*x21.transpose())/(x21_square*x21_square);
    MatrixXd dtdq = dtdx10*J10+dtdx21*J21;
    Vector3d h = x0-(x1+x21*t);
    MatrixXd dhdq = J_pt-(J_line.block(0,0,3,nq)+J21*t+x21*dtdq);
    double d = h.squaredNorm();
    MatrixXd dddq = 2*h.transpose()*dhdq;
    c.resize(2);
    dc.resize(2,nq);
    c<<d,t;
    dc.block(0,0,1,nq) = dddq;
    dc.block(1,0,1,nq) = dtdq;
  }
  else
  {
    c.resize(0);
    dc.resize(0,0);
  }
}

void Point2LineSegDistConstraint::bounds(const double* t, VectorXd &lb, VectorXd &ub) const
{
  if(this->isTimeValid(t))
  {
    lb.resize(2);
    ub.resize(2);
    lb<<this->dist_lb*this->dist_lb,0.0;
    ub<<this->dist_ub*this->dist_ub,1.0;
  }
  else
  {
    lb.resize(0);
    ub.resize(0);
  }
}

void Point2LineSegDistConstraint::name(const double* t, std::vector<std::string> &name_str) const
{
  if(this->isTimeValid(t))
  {
    char cnst_name_buffer1[2000];
    sprintf(cnst_name_buffer1,"Distance from %s pt to a line on %s",this->robot->bodies[this->pt_body]->linkname.c_str(),this->robot->bodies[this->line_body]->linkname.c_str());
    std::string cnst_name_str1(cnst_name_buffer1);
    name_str.push_back(cnst_name_str1);
    char cnst_name_buffer2[100];
    sprintf(cnst_name_buffer2,"Fraction of point projection onto line segment");
    std::string cnst_name_str2(cnst_name_buffer2);
    name_str.push_back(cnst_name_str2);
  }
}


WorldFixedPositionConstraint::WorldFixedPositionConstraint(RigidBodyManipulator* robot, int body, const Matrix3Xd &pts, const Vector2d &tspan):MultipleTimeKinematicConstraint(robot,tspan)
{
  this->body = body;
  if(pts.rows() != 3)
  {
    std::cerr<<"pts must have 3 rows"<<std::endl;
  }
  this->pts = pts;
  this->body_name = robot->getBodyOrFrameName(body);
  this->type = RigidBodyConstraint::WorldFixedPositionConstraintType;
}

int WorldFixedPositionConstraint::getNumConstraint(const double* t, int n_breaks) const
{
  int num_valid_t = this->numValidTime(t,n_breaks);
  if(num_valid_t>=2)
  {
    return static_cast<int>(this->pts.cols());
  }
  else
  {
    return 0;
  }
}

void WorldFixedPositionConstraint::eval_valid(const double* valid_t, int num_valid_t, const MatrixXd &valid_q, VectorXd &c, MatrixXd &dc_valid) const
{
  int n_pts = static_cast<int>(this->pts.cols());
  int nq = this->robot->num_positions;
  MatrixXd *pos = new MatrixXd[num_valid_t];
  MatrixXd *dpos = new MatrixXd[num_valid_t];
  for(int i = 0;i<num_valid_t;i++)
  {
    Map<VectorXd> qvec((double*) valid_q.data()+i*nq, nq);
    VectorXd v = VectorXd::Zero(0);
    robot->doKinematicsNew(qvec, v);
    pos[i].resize(3,n_pts);
    auto pos_gradientvar = robot->forwardKinNew(pts, body, 0, 0, 1);
    pos[i] = pos_gradientvar.value();
    dpos[i] = pos_gradientvar.gradient().value();
  }
  int* next_idx = new int[num_valid_t];
  int* prev_idx = new int[num_valid_t];
  for(int i = 0;i<num_valid_t;i++)
  {
    next_idx[i] = (i+1)%num_valid_t;
    prev_idx[i] = (i+num_valid_t-1)%num_valid_t;
  }
  c = VectorXd::Zero(n_pts);
  dc_valid = MatrixXd::Zero(n_pts,nq*num_valid_t);
  for(int i = 0;i<num_valid_t;i++)
  {
    MatrixXd tmp1(3,n_pts);
    tmp1 = pos[i]-pos[next_idx[i]];
    MatrixXd tmp2 = tmp1.cwiseProduct(tmp1);
    VectorXd tmp3 = tmp2.colwise().sum();
    c += tmp3.transpose();
    for(int j = 0;j<n_pts;j++)
    {
      Vector3d tmp_vec = 4*pos[i].col(j)-2*pos[next_idx[i]].col(j)-2*pos[prev_idx[i]].col(j);
      dc_valid.block(j,i*nq,1,nq) = tmp_vec.transpose()*dpos[i].block(j*3,0,3,nq);
    }
  }
  delete[] pos;
  delete[] dpos;
  delete[] next_idx;
  delete[] prev_idx;
}

void WorldFixedPositionConstraint::bounds(const double* t, int n_breaks, VectorXd &lb, VectorXd &ub) const
{
  int num_valid_t = this->numValidTime(t,n_breaks);
  if(num_valid_t>=2)
  {
    int n_pts = static_cast<int>(this->pts.cols());
    lb.resize(n_pts,1);
    ub.resize(n_pts,1);
    lb = VectorXd::Zero(n_pts);
    ub = VectorXd::Zero(n_pts);
  }
  else
  {
    lb.resize(0);
    ub.resize(0);
  }
}

void WorldFixedPositionConstraint::name(const double* t, int n_breaks,std::vector<std::string> &name_str) const
{
  int num_valid_t = this->numValidTime(t,n_breaks);
  if(num_valid_t>=2)
  {
    int n_pts = static_cast<int>(this->pts.cols());
    for(int i = 0;i<n_pts;i++)
    {
      char cnst_name_buffer[500];
      sprintf(cnst_name_buffer,"World fixed position constraint for %s %ds point",this->body_name.c_str(),i);
      std::string cnst_name_str(cnst_name_buffer);
      name_str.push_back(cnst_name_str);
    }
  }
}


WorldFixedOrientConstraint::WorldFixedOrientConstraint(RigidBodyManipulator* robot, int body, const Vector2d &tspan): MultipleTimeKinematicConstraint(robot,tspan)
{
  this->body = body;
  this->body_name = robot->getBodyOrFrameName(body);
  this->type = RigidBodyConstraint::WorldFixedOrientConstraintType;
}

int WorldFixedOrientConstraint::getNumConstraint(const double* t, int n_breaks) const
{
  int num_valid_t = this->numValidTime(t,n_breaks);
  if(num_valid_t>=2)
    return 1;
  else
    return 0;
}

void WorldFixedOrientConstraint::eval_valid(const double* valid_t, int num_valid_t, const MatrixXd &valid_q, VectorXd &c, MatrixXd &dc_valid) const
{
  int nq = this->robot->num_positions;
  Vector4d* quat = new Vector4d[num_valid_t];
  MatrixXd* dquat = new MatrixXd[num_valid_t];
  for(int i = 0;i<num_valid_t;i++)
  {
    Map<VectorXd> qvec((double*) valid_q.data()+i*nq, nq);
    VectorXd v = VectorXd::Zero(0);
    robot->doKinematicsNew(qvec, v);
    auto tmp_pos_gradientvar = robot->forwardKinNew(Vector3d::Zero().eval(), body, 0, 2, 1);
    const auto& tmp_pos = tmp_pos_gradientvar.value();
    const auto& dtmp_pos = tmp_pos_gradientvar.gradient().value();
    quat[i] = tmp_pos.tail(4);
    dquat[i].resize(4,nq);
    dquat[i] = dtmp_pos.block(3,0,4,nq);
  }
  int* next_idx = new int[num_valid_t];
  int* prev_idx = new int[num_valid_t];
  for(int i = 0;i<num_valid_t;i++)
  {
    next_idx[i] = (i+1)%num_valid_t;
    prev_idx[i] = (i+num_valid_t-1)%num_valid_t;
  }
  c(0) = 0.0;
  dc_valid = MatrixXd::Zero(1,nq*num_valid_t);
  for(int i = 0;i<num_valid_t;i++)
  {

    double tmp1 = quat[i].transpose()*quat[next_idx[i]];
    double tmp2 = quat[i].transpose()*quat[prev_idx[i]];
    c(0) += pow(tmp1,2);
    dc_valid.block(0,nq*i,1,nq) = (2*tmp1*quat[next_idx[i]].transpose() + 2*tmp2*quat[prev_idx[i]].transpose())*dquat[i];
  }
  delete[] quat;
  delete[] dquat;
  delete[] next_idx;
  delete[] prev_idx;
}

void WorldFixedOrientConstraint::bounds(const double* t,int n_breaks, VectorXd &lb, VectorXd &ub) const
{
  int num_valid_t = this->numValidTime(t,n_breaks);
  if(num_valid_t>=2)
  {
    lb.resize(1);
    ub.resize(1);
    lb(0) = (double) num_valid_t;
    ub(0) = (double) num_valid_t;
  }
  else
  {
    lb.resize(0);
    ub.resize(0);
  }
}

void WorldFixedOrientConstraint::name(const double* t, int n_breaks, std::vector<std::string> &name_str) const
{
  int num_valid_t = this->numValidTime(t,n_breaks);
  if(num_valid_t>=2)
  {
    char cnst_name_buffer[500];
    sprintf(cnst_name_buffer,"World fixed orientation constraint for %s",this->body_name.c_str());
    std::string cnst_name_str(cnst_name_buffer);
    name_str.push_back(cnst_name_str);
  }
}


WorldFixedBodyPoseConstraint::WorldFixedBodyPoseConstraint(RigidBodyManipulator *robot, int body, const Vector2d &tspan): MultipleTimeKinematicConstraint(robot,tspan)
{
  this->body = body;
  this->body_name = robot->getBodyOrFrameName(body);
  this->type = RigidBodyConstraint::WorldFixedBodyPoseConstraintType;
}

int WorldFixedBodyPoseConstraint::getNumConstraint(const double* t, int n_breaks) const
{
  int num_valid_t = this->numValidTime(t,n_breaks);
  if(num_valid_t>=2)
  {
    return 2;
  }
  else
  {
    return 0;
  }
}

void WorldFixedBodyPoseConstraint::eval_valid(const double* valid_t, int num_valid_t,const MatrixXd &valid_q, VectorXd &c, MatrixXd &dc_valid) const
{
  int nq = this->robot->num_positions;
  Vector3d *pos = new Vector3d[num_valid_t];
  Vector4d *quat = new Vector4d[num_valid_t];
  MatrixXd *dpos = new MatrixXd[num_valid_t];
  MatrixXd *dquat = new MatrixXd[num_valid_t];
  for(int i = 0;i<num_valid_t;i++)
  {
    Map<VectorXd> qvec((double *) valid_q.data() + i * nq, nq);
    VectorXd v = VectorXd::Zero(0);
    robot->doKinematicsNew(qvec, v);
    auto pos_tmp_gradientvar = robot->forwardKinNew(Vector3d::Zero().eval(), body, 0, 2, 1);
    const auto& pos_tmp = pos_tmp_gradientvar.value();
    const auto& J_tmp = pos_tmp_gradientvar.gradient().value();

    pos[i] = pos_tmp.head(3);
    quat[i] = pos_tmp.tail(4);
    dpos[i].resize(3,nq);
    dpos[i] = J_tmp.block(0,0,3,nq);
    dquat[i].resize(4,nq);
    dquat[i] = J_tmp.block(3,0,4,nq);
  }
  int* next_idx = new int[num_valid_t];
  int* prev_idx = new int[num_valid_t];
  for(int i = 0;i<num_valid_t;i++)
  {
    next_idx[i] = (i+1)%num_valid_t;
    prev_idx[i] = (i+num_valid_t-1)%num_valid_t;
  }
  c = Vector2d::Zero();
  dc_valid = MatrixXd::Zero(2,nq*num_valid_t);
  for(int i = 0;i<num_valid_t;i++)
  {
    Vector3d tmp1;
    tmp1 = pos[i]-pos[next_idx[i]];
    c(0) += tmp1.transpose()*tmp1;
    Vector3d tmp_vec = 4*pos[i]-2*pos[next_idx[i]]-2*pos[prev_idx[i]];
    dc_valid.block(0,i*nq,1,nq) = tmp_vec.transpose()*dpos[i];
    double tmp2 = quat[i].transpose()*quat[next_idx[i]];
    double tmp3 = quat[i].transpose()*quat[prev_idx[i]];
    c(1) += pow(tmp2,2);
    dc_valid.block(1,nq*i,1,nq) = (2*tmp2*quat[next_idx[i]].transpose() + 2*tmp3*quat[prev_idx[i]].transpose())*dquat[i];
  }
  delete[] pos;
  delete[] dpos;
  delete[] quat;
  delete[] dquat;
  delete[] next_idx;
  delete[] prev_idx;
}

void WorldFixedBodyPoseConstraint::bounds(const double* t, int n_breaks, VectorXd &lb, VectorXd &ub) const
{
  int num_valid_t = this->numValidTime(t,n_breaks);
  if(num_valid_t>=2)
  {
    lb.resize(2);
    ub.resize(2);
    lb<<0.0,(double) num_valid_t;
    ub<<0.0,(double) num_valid_t;
  }
  else
  {
    lb.resize(0);
    ub.resize(0);
  }
}

void WorldFixedBodyPoseConstraint::name(const double* t, int n_breaks, std::vector<std::string> &name_str) const
{
  int num_valid_t = this->numValidTime(t,n_breaks);
  if(num_valid_t>=2)
  {
    char cnst_name_buffer1[500];
    sprintf(cnst_name_buffer1,"World fixed body pose constraint for %s position",this->body_name.c_str());
    std::string cnst_name_str1(cnst_name_buffer1);
    name_str.push_back(cnst_name_str1);
    char cnst_name_buffer2[500];
    sprintf(cnst_name_buffer2,"World fixed body pose constraint for %s orientation",this->body_name.c_str());
    std::string cnst_name_str2(cnst_name_buffer2);
    name_str.push_back(cnst_name_str2);
  }
}


AllBodiesClosestDistanceConstraint::AllBodiesClosestDistanceConstraint(
    RigidBodyManipulator* robot, double lb, double ub,
    const std::vector<int> &active_bodies_idx,
    const std::set<std::string> &active_group_names, const Vector2d &tspan)
  : SingleTimeKinematicConstraint(robot, tspan), lb(lb), ub(ub),
    active_bodies_idx(active_bodies_idx), active_group_names(active_group_names)
{
  VectorXd c;
  MatrixXd dc;
  double t = 0;
  eval(&t,c,dc);
  //DEBUG
  //std::cout << "ABCDC::ABCDC: c.size() = " << c.size() << std::endl;
  //END_DEBUG
  num_constraint = static_cast<int>(c.size());
  this->type = RigidBodyConstraint::AllBodiesClosestDistanceConstraintType;
};

//AllBodiesClosestDistanceConstraint::AllBodiesClosestDistanceConstraint(const AllBodiesClosestDistanceConstraint &rhs)
  //: SingleTimeKinematicConstraint(rhs)
//{
  //DEBUG
  //std::cout << "ABCDC::ABCDC: Copy constructor" << std::endl;
  //END_DEBUG
  //double t = 0;
  //VectorXd c;
  //MatrixXd dc;
  //eval(&t,c,dc);
  //num_constraint = c.size();
//}

void AllBodiesClosestDistanceConstraint::updateRobot(RigidBodyManipulator* robot)
{
  this->robot = robot;
  double t = 0;
  VectorXd c;
  MatrixXd dc;
  this->eval(&t,c,dc);
  this->num_constraint = static_cast<int>(c.size());
}

void
AllBodiesClosestDistanceConstraint::eval(const double* t, VectorXd& c, MatrixXd& dc) const
{
  if(this->isTimeValid(t)) {
    Matrix3Xd xA, xB, normal;
    std::vector<int> idxA;
    std::vector<int> idxB;

    if (active_bodies_idx.size() > 0) {
      if (active_group_names.size() > 0) {
        robot->collisionDetect(c,normal,xA,xB,idxA,idxB,active_bodies_idx,active_group_names);
      } else {
        robot->collisionDetect(c,normal,xA,xB,idxA,idxB,active_bodies_idx);
      }
    } else {
      if (active_group_names.size() > 0) {
        robot->collisionDetect(c,normal,xA,xB,idxA,idxB,active_group_names);
      } else {
        robot->collisionDetect(c,normal,xA,xB,idxA,idxB);
      }
    }
    int num_pts = static_cast<int>(xA.cols());
    dc = MatrixXd::Zero(num_pts,robot->num_positions);
    MatrixXd JA = MatrixXd::Zero(3,robot->num_positions);
    MatrixXd JB = MatrixXd::Zero(3,robot->num_positions);
    for (int i = 0; i < num_pts; ++i) {
      JA = robot->forwardJacV(xA.col(i), idxA.at(i), 0, 0, true, 0).value();
      JB = robot->forwardJacV(xB.col(i), idxB.at(i), 0, 0, true, 0).value();
      dc.row(i) = normal.col(i).transpose() * (JA - JB);
    }
  } else {
    c.resize(0);
    dc.resize(0,0);
  }
};

void AllBodiesClosestDistanceConstraint::bounds(const double* t, VectorXd& lb, VectorXd& ub) const
{
  lb.resize(num_constraint);
  ub.resize(num_constraint);
  if(this->isTimeValid(t))
  {
    lb = VectorXd::Constant(num_constraint,this->lb);
    ub = VectorXd::Constant(num_constraint,this->ub);
  }
}

void AllBodiesClosestDistanceConstraint::name(const double* t, std::vector<std::string> &name) const
{
  if(this->isTimeValid(t))
  {
    char cnst_name_buffer[100];
    if(t == nullptr)
    {
      sprintf(cnst_name_buffer,"All-to-all closest distance constraint");
    }
    else
    {
      sprintf(cnst_name_buffer,
          "All-to-all closest distance constraint at time %10.4f",*t);
    }
    std::string cnst_name(cnst_name_buffer);
    for(int i = 0;i<this->num_constraint;i++)
    {
      name.push_back(cnst_name);
    }
  }
  else
  {
    name.push_back("");
  }
}

MinDistanceConstraint::MinDistanceConstraint( RigidBodyManipulator* robot,
    double min_distance, const std::vector<int> &active_bodies_idx,
    const std::set<std::string> &active_group_names, const Vector2d &tspan)
  : SingleTimeKinematicConstraint(robot, tspan), min_distance(min_distance),
    active_bodies_idx(active_bodies_idx), active_group_names(active_group_names)
{
  this->num_constraint = 1;
  this->type = RigidBodyConstraint::MinDistanceConstraintType;
};

void
MinDistanceConstraint::eval(const double* t, VectorXd& c, MatrixXd& dc) const
{
  //DEBUG
  //std::cout << "MinDistanceConstraint::eval: START" << std::endl;
  //END_DEBUG
  if(this->isTimeValid(t)) {
    VectorXd dist, scaled_dist, pairwise_costs;
    Matrix3Xd xA, xB, normal;
    MatrixXd ddist_dq, dscaled_dist_ddist, dpairwise_costs_dscaled_dist;
    std::vector<int> idxA;
    std::vector<int> idxB;

    if (active_bodies_idx.size() > 0) {
      if (active_group_names.size() > 0) {
        robot->collisionDetect(dist,normal,xA,xB,idxA,idxB,active_bodies_idx,active_group_names);
      } else {
        robot->collisionDetect(dist,normal,xA,xB,idxA,idxB,active_bodies_idx);
      }
    } else {
      if (active_group_names.size() > 0) {
        robot->collisionDetect(dist,normal,xA,xB,idxA,idxB,active_group_names);
      } else {
        robot->collisionDetect(dist,normal,xA,xB,idxA,idxB);
      }
    }

    int num_pts = static_cast<int>(xA.cols());
    ddist_dq = MatrixXd::Zero(num_pts,robot->num_positions);

    // Compute Jacobian of closest distance vector
    //DEBUG
    //std:: cout << "IntegratedClosestDistanceConstraint::eval_valid: Compute distance Jacobian" << std::endl;
    //END_DEBUG
    scaleDistance(dist,scaled_dist,dscaled_dist_ddist);
    penalty(scaled_dist, pairwise_costs, dpairwise_costs_dscaled_dist);

    std::vector< std::vector<int> > orig_idx_of_pt_on_bodyA(robot->num_bodies);
    std::vector< std::vector<int> > orig_idx_of_pt_on_bodyB(robot->num_bodies);
    for (int k = 0; k < num_pts; ++k) {
      //DEBUG
      //std::cout << "MinDistanceConstraint::eval: First loop: " << k << std::endl;
      //std::cout << "pairwise_costs.size() = " << pairwise_costs.size() << std::endl;
      //std::cout << "pairwise_costs.size() = " << pairwise_costs.size() << std::endl;
      //END_DEBUG
      if (pairwise_costs(k) > 0) {
        orig_idx_of_pt_on_bodyA.at(idxA.at(k)).push_back(k);
        orig_idx_of_pt_on_bodyB.at(idxB.at(k)).push_back(k);
      }
    }
    for (int k = 0; k < robot->num_bodies; ++k) {
      //DEBUG
      //std::cout << "MinDistanceConstraint::eval: Second loop: " << k << std::endl;
      //END_DEBUG
      int l = 0;
      int numA = static_cast<int>(orig_idx_of_pt_on_bodyA.at(k).size());
      int numB = static_cast<int>(orig_idx_of_pt_on_bodyB.at(k).size());
      if(numA+numB==0)
      {
        continue;
      }
      Matrix3Xd x_k(3, numA + numB);
      for (; l < numA; ++l) {
        //DEBUG
        //std::cout << "MinDistanceConstraint::eval: Third loop: " << l << std::endl;
        //END_DEBUG
        x_k.col(l) = xA.col(orig_idx_of_pt_on_bodyA.at(k).at(l));
      }
      for (; l < numA + numB; ++l) {
        //DEBUG
        //std::cout << "MinDistanceConstraint::eval: Fourth loop: " << l << std::endl;
        //END_DEBUG
        x_k.col(l) = xB.col(orig_idx_of_pt_on_bodyB.at(k).at(l-numA));
      }

      auto J_k = robot->forwardJacV(x_k, k, 0, 0, true, 0).value();

      robot->forwardJac(k,x_k,0,J_k);
      l = 0;
      for (; l < numA; ++l) {
        //DEBUG
        //std::cout << "MinDistanceConstraint::eval: Fifth loop: " << l << std::endl;
        //END_DEBUG
        ddist_dq.row(orig_idx_of_pt_on_bodyA.at(k).at(l)) += normal.col(orig_idx_of_pt_on_bodyA.at(k).at(l)).transpose() * J_k.block(3*l,0,3,robot->num_positions);
      }
      for (; l < numA+numB; ++l) {
        //DEBUG
        //std::cout << "MinDistanceConstraint::eval: Sixth loop: " << l << std::endl;
        //END_DEBUG
        ddist_dq.row(orig_idx_of_pt_on_bodyB.at(k).at(l-numA)) += -normal.col(orig_idx_of_pt_on_bodyB.at(k).at(l-numA)).transpose() * J_k.block(3*l,0,3,robot->num_positions);
      }
    }
    //DEBUG
    //std::cout << "MinDistanceConstraint::eval: Set outputs" << std::endl;
    //END_DEBUG
    MatrixXd dcost_dscaled_dist(dpairwise_costs_dscaled_dist.colwise().sum());
    c(0) = pairwise_costs.sum();
    dc = dcost_dscaled_dist*dscaled_dist_ddist*ddist_dq;
  } else {
    c.resize(0);
    dc.resize(0,0);
  }
  //DEBUG
  //std::cout << "MinDistanceConstraint::eval: END" << std::endl;
  //END_DEBUG
};

void MinDistanceConstraint::scaleDistance(const Eigen::VectorXd &dist, Eigen::VectorXd &scaled_dist, Eigen::MatrixXd &dscaled_dist_ddist) const
{
  int nd = static_cast<int>(dist.size());
  double recip_min_dist = 1/this->min_distance;
  scaled_dist = recip_min_dist*dist - VectorXd::Ones(nd,1);
  dscaled_dist_ddist = recip_min_dist*MatrixXd::Identity(nd,nd);
}

void MinDistanceConstraint::penalty(const Eigen::VectorXd &dist, Eigen::VectorXd &cost, Eigen::MatrixXd &dcost_ddist) const
{
  int nd = static_cast<int>(dist.size());
  cost = VectorXd::Zero(nd,1);
  dcost_ddist = MatrixXd::Zero(nd,nd);
  for (int i = 0; i < nd; ++i) {
    if (dist(i) < 0) {
      double exp_recip_dist = exp(1/dist(i));
      cost(i) = -dist(i)*exp_recip_dist;
      dcost_ddist(i,i) = exp_recip_dist*(1/dist(i) - 1);
    }
  }
}

void MinDistanceConstraint::bounds(const double* t, VectorXd& lb, VectorXd& ub) const
{
  lb.resize(this->num_constraint);
  ub.resize(this->num_constraint);
  if(this->isTimeValid(t))
  {
    lb = VectorXd::Zero(num_constraint);
    ub = VectorXd::Zero(num_constraint);
  }
}

void MinDistanceConstraint::name(const double* t, std::vector<std::string> &name) const
{
  if(this->isTimeValid(t))
  {
    char cnst_name_buffer[100];
    if(t == nullptr)
    {
      sprintf(cnst_name_buffer,"Minimum distance constraint");
    }
    else
    {
      sprintf(cnst_name_buffer,
          "Minimum distance constraint at time %10.4f",*t);
    }
    std::string cnst_name(cnst_name_buffer);
    name.push_back(cnst_name);
  }
  else
  {
    name.push_back("");
  }
}

WorldPositionInFrameConstraint::WorldPositionInFrameConstraint(
    RigidBodyManipulator *robot, int body, const Eigen::Matrix3Xd &pts,
    const Eigen::Matrix4d& T_frame_to_world,
    const Eigen::MatrixXd &lb, const Eigen::MatrixXd &ub, const Eigen::Vector2d &tspan)
  : WorldPositionConstraint(robot,body,pts,lb,ub,tspan)
{
  this->T_frame_to_world = T_frame_to_world;
  this->T_world_to_frame = T_frame_to_world.inverse();
  this->type = RigidBodyConstraint::WorldPositionInFrameConstraintType;
}

void WorldPositionInFrameConstraint::evalPositions(Matrix3Xd &pos, MatrixXd &J) const
{
  WorldPositionConstraint::evalPositions(pos, J);
  pos = (this->T_world_to_frame*pos.colwise().homogeneous()).topRows(3);
  auto J_reshaped = Map<MatrixXd>(J.data(),3,n_pts*J.cols());
  J_reshaped = T_world_to_frame.topLeftCorner<3,3>()*J_reshaped;
}

void WorldPositionInFrameConstraint::evalNames(const double* t, std::vector<std::string>& cnst_names) const
{
  std::string time_str;
  if(t!=nullptr)
  {
    char time_str_buffer[20];
    sprintf(time_str_buffer,"at time %5.2f",*t);
    time_str+=std::string(time_str_buffer);
  }
  char cnst_name_str_buffer[500];
  for(int i = 0;i<this->n_pts;i++)
  {
    sprintf(cnst_name_str_buffer,"%s pts(:,%d) x in frame %s",this->body_name.c_str(),i+1,time_str.c_str());
    cnst_names.push_back(std::string(cnst_name_str_buffer));
    sprintf(cnst_name_str_buffer,"%s pts(:,%d) y in frame %s",this->body_name.c_str(),i+1,time_str.c_str());
    cnst_names.push_back(std::string(cnst_name_str_buffer));
    sprintf(cnst_name_str_buffer,"%s pts(:,%d) z in frame %s",this->body_name.c_str(),i+1,time_str.c_str());
    cnst_names.push_back(std::string(cnst_name_str_buffer));
  }
}

WorldPositionInFrameConstraint::~WorldPositionInFrameConstraint()
{
}


PostureChangeConstraint::PostureChangeConstraint(RigidBodyManipulator* robot, const VectorXi &joint_ind, const VectorXd &lb_change, const VectorXd &ub_change, const Vector2d &tspan):MultipleTimeLinearPostureConstraint(robot,tspan)
{
  this->setJointChangeBounds(joint_ind,lb_change,ub_change);
  this->type = RigidBodyConstraint::PostureChangeConstraintType;
}

void PostureChangeConstraint::setJointChangeBounds(const VectorXi &joint_ind, const VectorXd &lb_change, const VectorXd &ub_change)
{
  this->joint_ind = joint_ind;
  this->lb_change = VectorXd(joint_ind.size());
  this->ub_change = VectorXd(joint_ind.size());
  for(int i = 0;i<joint_ind.size();i++)
  {
    double lb_change_min = this->robot->joint_limit_min[joint_ind(i)]-this->robot->joint_limit_max[joint_ind(i)];
    double ub_change_max = this->robot->joint_limit_max[joint_ind(i)]-this->robot->joint_limit_min[joint_ind(i)];
    this->lb_change(i) = (lb_change_min<lb_change(i)?lb_change(i):lb_change_min);
    this->ub_change(i) = (ub_change_max>ub_change(i)?ub_change(i):ub_change_max);
  }
}

int PostureChangeConstraint::getNumConstraint(const double* t, int n_breaks) const
{
  std::vector<bool> valid_flag = this->isTimeValid(t,n_breaks);
  int num_valid_t = this->numValidTime(valid_flag);
  if(num_valid_t>=2) return (num_valid_t-1)*static_cast<int>(this->joint_ind.size());
  else return 0;
}

void PostureChangeConstraint::feval(const double* t, int n_breaks, const MatrixXd &q, VectorXd &c) const
{
  std::vector<bool> valid_flag = this->isTimeValid(t,n_breaks);
  int num_valid_t = this->numValidTime(valid_flag);
  if(num_valid_t>=2)
  {
    VectorXi valid_t_ind;
    this->validTimeInd(valid_flag,valid_t_ind);
    int nc = this->getNumConstraint(t,n_breaks);
    c.resize(nc);
    for(int i = 1;i<num_valid_t;i++)
    {
      for(int j = 0;j<this->joint_ind.size();j++)
      {
        c((i-1)*this->joint_ind.size()+j) = q(this->joint_ind(j),valid_t_ind(i))-q(this->joint_ind(j),valid_t_ind(0));
      }
    }
  }
  else
  {
    c.resize(0);
  }
}

void PostureChangeConstraint::geval(const double* t, int n_breaks, VectorXi &iAfun, VectorXi &jAvar, VectorXd &A) const
{
  std:: vector<bool> valid_flag = this->isTimeValid(t,n_breaks);
  int num_valid_t = this->numValidTime(valid_flag);
  if(num_valid_t>=2)
  {
    int num_joints = static_cast<int>(this->joint_ind.size());
    int nc = this->getNumConstraint(t,n_breaks);
    int nq = this->robot->num_positions;
    iAfun.resize(nc*2);
    jAvar.resize(nc*2);
    A.resize(nc*2);
    VectorXi valid_t_ind;
    this->validTimeInd(valid_flag,valid_t_ind);
    for(int i = 0;i<num_valid_t-1;i++)
    {
      for(int j = 0;j<num_joints;j++)
      {
        iAfun(i*num_joints+j) = i*num_joints+j;
        jAvar(i*num_joints+j) = valid_t_ind(0)*nq+this->joint_ind[j];
        A(i*num_joints+j) = -1;
      }
    }
    for(int i = 1;i<num_valid_t;i++)
    {
      for(int j = 0;j<num_joints;j++)
      {
        int ind = (num_valid_t-1)*num_joints+(i-1)*num_joints+j;
        iAfun(ind) = (i-1)*num_joints+j;
        jAvar(ind) = nq*valid_t_ind(i)+this->joint_ind[j];
        A(ind) = 1.0;
      }
    }
  }
  else
  {
    iAfun.resize(0);
    jAvar.resize(0);
    A.resize(0);
  }
}

void PostureChangeConstraint::name(const double* t, int n_breaks, std::vector<std::string> &name_str) const
{
  std::vector<bool> valid_flag = this->isTimeValid(t,n_breaks);
  int num_valid_t = this->numValidTime(valid_flag);
  VectorXi valid_t_ind;
  this->validTimeInd(valid_flag,valid_t_ind);
  if(num_valid_t>=2)
  {
    for(int i = 1;i<num_valid_t;i++)
    {
      for(int j = 0;j<this->joint_ind.size();j++)
      {
        char cnst_name_buffer[200];
        sprintf(cnst_name_buffer,"Posture change for joint %d at time %5.3f",this->joint_ind[j]+1,t[valid_t_ind(i)]);
        std::string cnst_name(cnst_name_buffer);
        name_str.push_back(cnst_name);
      }
    }
  }
}

void PostureChangeConstraint::bounds(const double* t, int n_breaks, VectorXd &lb, VectorXd &ub) const
{
  std::vector<bool> valid_flag = this->isTimeValid(t,n_breaks);
  int num_valid_t = this->numValidTime(valid_flag);
  if(num_valid_t>=2)
  {
    int nc = this->getNumConstraint(t,n_breaks);
    lb.resize(nc);
    ub.resize(nc);
    int num_joints = static_cast<int>(this->joint_ind.size());
    for(int i = 0;i<num_valid_t-1;i++)
    {
      lb.block(i*num_joints,0,num_joints,1) = this->lb_change;
      ub.block(i*num_joints,0,num_joints,1) = this->ub_change;
    }
  }
  else
  {
    lb.resize(0);
    ub.resize(0);
  }
}

GravityCompensationTorqueConstraint::
GravityCompensationTorqueConstraint(RigidBodyManipulator* robot,
    const VectorXi& joint_indices, const VectorXd& lb, const VectorXd& ub, const Vector2d &tspan)
  : SingleTimeKinematicConstraint(robot, tspan), lb(lb), ub(ub), joint_indices(joint_indices)
{
  this->num_constraint = static_cast<int>(joint_indices.size());
  this->type = RigidBodyConstraint::GravityCompensationTorqueConstraintType;
}

void GravityCompensationTorqueConstraint::eval(const double* t, VectorXd& c, MatrixXd& dc) const
{
  VectorXd qd = VectorXd::Zero(robot->num_velocities);
  robot->doKinematicsNew(robot->cached_q, qd, true, true);
  std::map<int, std::unique_ptr<GradientVar<double, TWIST_SIZE, 1>> > f_ext;
  auto G_gradientvar = robot->inverseDynamics(f_ext, (GradientVar<double, Eigen::Dynamic, 1>*) nullptr, 1);

  c.resize(num_constraint);
  dc.resize(num_constraint, robot->num_positions);

  for (int i = 0; i < num_constraint; ++i) {
    c(i) = G_gradientvar.value()(joint_indices(i));
    dc.row(i) = G_gradientvar.gradient().value().block(joint_indices(i), 0, 1, robot->num_positions);
  }
}

void GravityCompensationTorqueConstraint::name(const double* t, std::vector<std::string>& name) const
{
  if(this->isTimeValid(t))
  {
    char cnst_name_buffer[100];
    if(t == nullptr)
    {
      sprintf(cnst_name_buffer,"Gravity compensation torque constraint");
    }
    else
    {
      sprintf(cnst_name_buffer,
          "Gravity compensation torque constraint at time %10.4f",*t);
    }
    std::string cnst_name(cnst_name_buffer);
    for(int i = 0;i<this->num_constraint;i++)
    {
      name.push_back(cnst_name);
    }
  }
  else
  {
    name.push_back("");
  }
}

void GravityCompensationTorqueConstraint::bounds(const double* t, VectorXd& lb, VectorXd& ub) const
{
  if(this->isTimeValid(t))
  {
    lb = this->lb;
    ub = this->ub;
  } else {
    lb.resize(0);
    ub.resize(0);
  }
}
