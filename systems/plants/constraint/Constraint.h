#ifndef __CONSTRAINT_H__
#define __CONSTRAINT_H__
#include <iostream>
#include "mex.h"
#include "../RigidBodyManipulator.h"
#include <vector>
#include <set>
#include <string>
#include <cstdio>
#include <Eigen/Dense>
#include "../../../util/drakeQuatUtil.h"
#include "../../../util/drakeUtil.h"
#include <sstream>


enum DrakeConstraintType{
  SingleTimeKinematicConstraintType = 1,
  MultipleTimeKinematicConstraintType = 2,
  QuasiStaticConstraintType = 3,
  PostureConstraintType = 4
};

void drakeMexPrintMatrix(const Eigen::MatrixXd &mat);

mxArray* createDrakeConstraintMexPointer(void* ptr, const char* deleteMethod, const char* name);

class Constraint
{
  protected:
    DrakeConstraintType type;
  public:
    Constraint(DrakeConstraintType type){this->type = type;};
    DrakeConstraintType getType() {return type;};
    virtual ~Constraint(void){};
};


class QuasiStaticConstraint: public Constraint
{
  protected:
    RigidBodyManipulator* robot;
    double tspan[2];
    double shrinkFactor;
    bool active;
    int num_bodies;
    int num_pts;
    std::vector<int> bodies;
    std::vector<int> num_body_pts;
    std::vector<Eigen::MatrixXd> body_pts;
  public:
    QuasiStaticConstraint(RigidBodyManipulator* robot,const Eigen::Vector2d &tspan); 
    bool isTimeValid(double* t);
    int getNumConstraint(double* t);
    void eval(double* t,double* weights,Eigen::VectorXd &c, Eigen::MatrixXd &dc); 
    void bounds(double* t,Eigen::VectorXd &lb, Eigen::VectorXd &ub);
    void name(double* t,std::vector<std::string> &name_str);
    virtual ~QuasiStaticConstraint(void);
    bool isActive(){return this->active;};
    RigidBodyManipulator* getRobotPointer(){return robot;};
    int getNumWeights() {return this->num_pts;};
    void addContact(int num_new_bodies, const int* body, const MatrixXd* body_pts);
    void setShrinkFactor(double factor);
    void setActive(bool flag){this->active = flag;};
};

class PostureConstraint: public Constraint
{
  protected:
    double tspan[2];
    double* joint_limit_min;
    double* joint_limit_max;
    RigidBodyManipulator *robot;
  public:
    PostureConstraint(RigidBodyManipulator *model, const Eigen::Vector2d &tspan);
    bool isTimeValid(double* t);
    void setJointLimits(int num_idx, const int* joint_idx, const double* lb, const double* ub);
    void bounds(double* t,double* joint_min, double* joint_max);
    RigidBodyManipulator* getRobotPointer(){return robot;};
    virtual ~PostureConstraint(void);
};

class SingleTimeKinematicConstraint: public Constraint
{
  protected:
    RigidBodyManipulator *robot;
    int num_constraint;
  public:
    double tspan[2];
    SingleTimeKinematicConstraint(RigidBodyManipulator *model, const Eigen::Vector2d &tspan);
    bool isTimeValid(double* t);
    int getNumConstraint(double* t);
    RigidBodyManipulator* getRobotPointer(){return robot;};
    virtual void eval(double* t,Eigen::VectorXd &c, Eigen::MatrixXd &dc) = 0;
    virtual void bounds(double* t, Eigen::VectorXd &lb, Eigen::VectorXd &ub) = 0;
    virtual void name(double* t, std::vector<std::string> &name_str) = 0;
    virtual ~SingleTimeKinematicConstraint(){};
};

class MultipleTimeKinematicConstraint : public Constraint
{
  protected:
    RigidBodyManipulator *robot;
  public:
    double tspan[2];
    MultipleTimeKinematicConstraint(RigidBodyManipulator *model, const Eigen::Vector2d &tspan);
    bool isTimeValid(double* t,int n_breaks);
    RigidBodyManipulator* getRobotPointer(){return robot;};
    virtual int getNumConstraint(double* t,int n_breaks) = 0;
    virtual void eval(double* t, int n_breaks,const Eigen::MatrixXd &q,Eigen::VectorXd &c, Eigen::MatrixXd &dc) = 0;
    virtual void bounds(double* t, int n_breaks, Eigen::VectorXd &lb, Eigen::VectorXd &ub) = 0;
    virtual void name(double* t, int n_breaks, std::vector<std::string> &name_str) = 0;
    virtual ~MultipleTimeKinematicConstraint(){};
};

class PositionConstraint : public SingleTimeKinematicConstraint
{
  protected:
    double* lb;
    double* ub;
    bool* null_constraint_rows;
    Eigen::MatrixXd pts; 
    int n_pts;
    virtual void evalPositions(Eigen::MatrixXd &pos,Eigen::MatrixXd &J) = 0;
  public:
    PositionConstraint(RigidBodyManipulator *model, const Eigen::MatrixXd &pts,Eigen::MatrixXd lb, Eigen::MatrixXd ub, const Eigen::Vector2d &tspan);
    virtual void eval(double* t,Eigen::VectorXd &c, Eigen::MatrixXd &dc);
    virtual void bounds(double* t, Eigen::VectorXd &lb, Eigen::VectorXd &ub);
    virtual void name(double* t, std::vector<std::string> &name_str) = 0;
    virtual ~PositionConstraint();
};

class WorldPositionConstraint: public PositionConstraint
{
  protected:
    int body;
    std::string body_name;
    virtual void evalPositions(Eigen::MatrixXd &pos, Eigen::MatrixXd &J);
  public:
    WorldPositionConstraint(RigidBodyManipulator *model, int body, const Eigen::MatrixXd &pts, Eigen::MatrixXd lb, Eigen::MatrixXd ub, const Eigen::Vector2d &tspan);
    virtual void name(double* t, std::vector<std::string> &name_str);
    virtual ~WorldPositionConstraint();
};

class WorldCoMConstraint: public PositionConstraint
{
  protected:
    int body;
    std::string body_name;
    virtual void evalPositions(Eigen::MatrixXd &pos, Eigen::MatrixXd &J);
  public:
    WorldCoMConstraint(RigidBodyManipulator *model, Eigen::Vector3d lb, Eigen::Vector3d ub, const Eigen::Vector2d &tspan);
    virtual void name(double* t, std::vector<std::string> &name_str);
    virtual ~WorldCoMConstraint(); 
};

class QuatConstraint: public SingleTimeKinematicConstraint
{
  protected:
    double tol;
    virtual void evalOrientationProduct(double &prod, Eigen::MatrixXd &dprod) = 0;
  public:
    QuatConstraint(RigidBodyManipulator *model, double tol, Eigen::Vector2d tspan);
    virtual void eval(double* t, Eigen::VectorXd &c, Eigen::MatrixXd &dc);
    virtual void bounds(double* t, Eigen::VectorXd &lb, Eigen::VectorXd &ub);
    virtual void name(double* t, std::vector<std::string> &name_str) = 0;
    virtual ~QuatConstraint();
};

class WorldQuatConstraint: public QuatConstraint
{
  protected:
    int body;
    std::string body_name;
    Eigen::Vector4d quat_des;
    virtual void evalOrientationProduct(double &prod, Eigen::MatrixXd &dprod);
  public:
    WorldQuatConstraint(RigidBodyManipulator *model, int body, Eigen::Vector4d quat_des, double tol, Eigen::Vector2d tspan);
    virtual void name(double* t, std::vector<std::string> &name_str);
    virtual ~WorldQuatConstraint();
};

class EulerConstraint: public SingleTimeKinematicConstraint
{
  protected:
    double* ub;
    double* lb;
    bool null_constraint_rows[3];
    double* avg_rpy;
    virtual void evalrpy(Eigen::Vector3d &rpy, Eigen::MatrixXd &J) = 0;
  public:
    EulerConstraint(RigidBodyManipulator *model, Vector3d lb, Vector3d ub, Eigen::Vector2d tspan);
    virtual void eval(double* t, Eigen::VectorXd &c, Eigen::MatrixXd &dc);
    virtual void bounds(double* t, Eigen::VectorXd &lb, Eigen::VectorXd &ub);
    virtual void name(double* t,std::vector<std::string> &name_str) = 0;
    virtual ~EulerConstraint();
};

class WorldEulerConstraint: public EulerConstraint
{
  protected:
    int body;
    std::string body_name;
    virtual void evalrpy(Eigen::Vector3d &rpy, Eigen::MatrixXd &J);
  public:
    WorldEulerConstraint(RigidBodyManipulator *model, int body, Vector3d lb, Vector3d ub, Eigen::Vector2d tspan);
    virtual void name(double* t, std::vector<std::string> &name_str);
    virtual ~WorldEulerConstraint();
};

class GazeConstraint : public SingleTimeKinematicConstraint
{
  protected:
    Eigen::Vector3d axis;
    double conethreshold;
  public:
    GazeConstraint(RigidBodyManipulator *model, Eigen::Vector3d axis, double conethreshold, Eigen::Vector2d tspan);
    virtual void eval(double* t,Eigen::VectorXd &c, Eigen::MatrixXd &dc) = 0;
    virtual void bounds(double* t, Eigen::VectorXd &lb, Eigen::VectorXd &ub) = 0;
    virtual void name(double* t, std::vector<std::string> &name_str) = 0;
    virtual ~GazeConstraint(void){};
};

class GazeOrientConstraint : public GazeConstraint
{
  protected:
    double threshold;
    Eigen::Vector4d quat_des;
    virtual void evalOrientation(Eigen::Vector4d &quat, Eigen::MatrixXd &dquat_dq) = 0;
  public:
    GazeOrientConstraint(RigidBodyManipulator* model, Eigen::Vector3d axis, Eigen::Vector4d quat_des, double conethreshold, double threshold, Eigen::Vector2d tspan);
    virtual void eval(double* t, Eigen::VectorXd &c, Eigen::MatrixXd &dc);
    virtual void bounds(double* t,Eigen::VectorXd &lb, Eigen::VectorXd &ub);
    virtual void name(double* t,std::vector<std::string> &name_str) = 0;
    virtual ~GazeOrientConstraint(void){};
};

class WorldGazeOrientConstraint: public GazeOrientConstraint
{
  protected:
    int body;
    std::string body_name;
    virtual void evalOrientation(Eigen::Vector4d &quat, Eigen::MatrixXd &dquat_dq);
  public:
    WorldGazeOrientConstraint(RigidBodyManipulator* model, int body, Eigen::Vector3d axis, Eigen::Vector4d quat_des,double conethreshold, double threshold, Eigen::Vector2d tspan);
    virtual void name(double* t,std::vector<std::string> &name_str);
    virtual ~WorldGazeOrientConstraint(){};
};

class GazeDirConstraint: public GazeConstraint
{
  protected:
    Eigen::Vector3d dir;
    virtual void evalOrientation(Eigen::Vector4d &quat, Eigen::MatrixXd &dquat_dq) = 0;
  public:
    GazeDirConstraint(RigidBodyManipulator* model, Eigen::Vector3d axis, Eigen::Vector3d dir,double conethreshold, Eigen::Vector2d tspan);
    virtual void eval(double*t, Eigen::VectorXd &c, Eigen::MatrixXd &dc);
    virtual void bounds(double* t, Eigen::VectorXd &lb, Eigen::VectorXd &ub);
    virtual void name(double* t, std::vector<std::string> &name_str) = 0;
    virtual ~GazeDirConstraint(void){};
};

class WorldGazeDirConstraint: public GazeDirConstraint
{
  protected:
    int body;
    std::string body_name;
    virtual void evalOrientation(Eigen::Vector4d &quat, Eigen::MatrixXd &dquat_dq);
  public:
    WorldGazeDirConstraint(RigidBodyManipulator* model, int body,Eigen::Vector3d axis, Eigen::Vector3d dir, double conethreshold, Eigen::Vector2d tspan);
    virtual void name(double* t, std::vector<std::string> &name_str);
    virtual ~WorldGazeDirConstraint(void){};
};

class GazeTargetConstraint: public GazeConstraint
{
  protected:
    Eigen::Vector3d target;
    Eigen::Vector4d gaze_origin;
  public:
    GazeTargetConstraint(RigidBodyManipulator* model, Eigen::Vector3d axis, Eigen::Vector3d target, Eigen::Vector4d gaze_origin, double conethreshold, Eigen::Vector2d tspan);
    virtual void eval(double* t, Eigen::VectorXd &c, Eigen::MatrixXd &dc) = 0;
    virtual void bounds(double* t, Eigen::VectorXd &lb, Eigen::VectorXd &ub);
    virtual void name(double* t, std::vector<std::string> &name_str) = 0;
    virtual ~GazeTargetConstraint(void){};
};

class WorldGazeTargetConstraint: public GazeTargetConstraint
{
  protected:
    int body;
    std::string body_name;
  public:
    WorldGazeTargetConstraint(RigidBodyManipulator* model, int body, Eigen::Vector3d axis, Eigen::Vector3d target, Eigen::Vector4d gaze_origin, double conethreshold, Eigen::Vector2d tspan);
    virtual void eval(double* t, Eigen::VectorXd &c, Eigen::MatrixXd &dc);
    virtual void name(double* t, std::vector<std::string> &name_str);
    virtual ~WorldGazeTargetConstraint(void){};
};

class WorldFixedPositionConstraint: public MultipleTimeKinematicConstraint
{
  protected:
    int body;
    std::string body_name;
    Eigen::MatrixXd pts;
  public:
    WorldFixedPositionConstraint(RigidBodyManipulator* model, int body, const Eigen::MatrixXd &pts,const Eigen::Vector2d &tspan);
    virtual int getNumConstraint(double* q, int n_breaks);
    virtual void eval(double* t, int n_breaks,const Eigen::MatrixXd &q,Eigen::VectorXd &c, Eigen::MatrixXd &dc);
    virtual void bounds(double* t,int n_breaks, Eigen::VectorXd &lb, Eigen::VectorXd &ub);
    virtual void name(double* t, int n_breaks,std::vector<std::string> &name_str);
    virtual ~WorldFixedPositionConstraint(void){};
};

class WorldFixedOrientConstraint: public MultipleTimeKinematicConstraint
{
  protected:
    int body;
    std::string body_name;
  public:
    WorldFixedOrientConstraint(RigidBodyManipulator* model, int body, const Eigen::Vector2d &tspan);
    virtual int getNumConstraint(double* q, int n_breaks);
    virtual void eval(double* t, int n_breaks, const Eigen::MatrixXd &q, Eigen::VectorXd &c, Eigen::MatrixXd &dc);
    virtual void bounds(double* t,int n_breaks, Eigen::VectorXd &lb, Eigen::VectorXd &ub);
    virtual void name(double* t, int n_breaks,std::vector<std::string> &name_str);
    virtual ~WorldFixedOrientConstraint(void){};
};

class WorldFixedBodyPostureConstraint: public MultipleTimeKinematicConstraint
{
  protected:
    int body;
    std::string body_name;
  public:
    WorldFixedBodyPostureConstraint(RigidBodyManipulator* model, int body, const Eigen::Vector2d &tspan);
    virtual int getNumConstraint(double* q, int n_breaks);
    virtual void eval(double* t, int n_breaks, const Eigen::MatrixXd &q, Eigen::VectorXd &c, Eigen::MatrixXd &dc);
    virtual void bounds(double* t,int n_breaks, Eigen::VectorXd &lb, Eigen::VectorXd &ub);
    virtual void name(double* t, int n_breaks,std::vector<std::string> &name_str);
    virtual ~WorldFixedBodyPostureConstraint(void){};
};
#endif
