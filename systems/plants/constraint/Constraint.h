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
    std::set<int> m_robotnumset;
    double tspan[2];
    double shrinkFactor;
    bool active;
    int num_bodies;
    int num_pts;
    std::vector<int> bodies;
    std::vector<int> num_body_pts;
    std::vector<Eigen::MatrixXd> body_pts;
    static const std::set<int> defaultRobotNumSet;
  public:
    QuasiStaticConstraint(RigidBodyManipulator* robot, const Eigen::Vector2d &tspan,const std::set<int> &robotnumset = QuasiStaticConstraint::defaultRobotNumSet); 
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
    void updateRobot(RigidBodyManipulator *robot);
    void updateRobotnum(std::set<int> &robotnumset);
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
    virtual void updateRobot(RigidBodyManipulator *robot) = 0;
    virtual ~SingleTimeKinematicConstraint(){};
};

class MultipleTimeKinematicConstraint : public Constraint
{
  protected:
    RigidBodyManipulator *robot;
    int numValidTime(double* t,int n_breaks);
  public:
    double tspan[2];
    MultipleTimeKinematicConstraint(RigidBodyManipulator *model, const Eigen::Vector2d &tspan);
    std::vector<bool> isTimeValid(double* t,int n_breaks);
    RigidBodyManipulator* getRobotPointer(){return robot;};
    virtual int getNumConstraint(double* t,int n_breaks) = 0;
    void eval(double* t, int n_breaks,const Eigen::MatrixXd &q,Eigen::VectorXd &c, Eigen::MatrixXd &dc);
    virtual void eval_valid(double* valid_t, int num_valid_t,const Eigen::MatrixXd &valid_q,Eigen::VectorXd &c, Eigen::MatrixXd &dc_valid) = 0;
    virtual void bounds(double* t, int n_breaks, Eigen::VectorXd &lb, Eigen::VectorXd &ub) = 0;
    virtual void name(double* t, int n_breaks, std::vector<std::string> &name_str) = 0;
    virtual void updateRobot(RigidBodyManipulator *robot) = 0;
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
    virtual void updateRobot(RigidBodyManipulator *robot);
    virtual ~WorldPositionConstraint();
};

class WorldCoMConstraint: public PositionConstraint
{
  protected:
    std::set<int> m_robotnum;
    int body;
    std::string body_name;
    virtual void evalPositions(Eigen::MatrixXd &pos, Eigen::MatrixXd &J);
    static const std::set<int> defaultRobotNumSet;
  public:
    WorldCoMConstraint(RigidBodyManipulator *model, Eigen::Vector3d lb, Eigen::Vector3d ub, const Eigen::Vector2d &tspan, const std::set<int> &robotnum = WorldCoMConstraint::defaultRobotNumSet);
    virtual void name(double* t, std::vector<std::string> &name_str);
    virtual void updateRobot(RigidBodyManipulator *robot);
    void updateRobotnum(const std::set<int> &robotnum);
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
    virtual void updateRobot(RigidBodyManipulator *robot);
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
    virtual void updateRobot(RigidBodyManipulator *robot);
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
    virtual void updateRobot(RigidBodyManipulator *robot);
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
    virtual void updateRobot(RigidBodyManipulator *robot);
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
    virtual void updateRobot(RigidBodyManipulator *robot);
    virtual ~WorldGazeTargetConstraint(void){};
};

class Point2PointDistanceConstraint: public SingleTimeKinematicConstraint
{
  protected:
    int bodyA;
    int bodyB;
    Eigen::MatrixXd ptA;
    Eigen::MatrixXd ptB;
    Eigen::VectorXd lb_square;
    Eigen::VectorXd ub_square;
  public:
    Point2PointDistanceConstraint(RigidBodyManipulator* model, int bodyA, int bodyB, const Eigen::MatrixXd &ptA, const Eigen::MatrixXd &ptB, const VectorXd &lb, const Eigen::VectorXd &ub, const Eigen::Vector2d &tspan);
    virtual void eval(double* t, Eigen::VectorXd &c, Eigen::MatrixXd &dc);
    virtual void name(double* t, std::vector<std::string> &name_str);
    virtual void bounds(double* t, Eigen::VectorXd &lb, Eigen::VectorXd &ub);
    virtual void updateRobot(RigidBodyManipulator *robot);
    virtual ~Point2PointDistanceConstraint(void){};
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
    virtual void eval_valid(double* valid_t, int num_valid_t,const Eigen::MatrixXd &valid_q,Eigen::VectorXd &c, Eigen::MatrixXd &dc_valid);
    virtual void bounds(double* t,int n_breaks, Eigen::VectorXd &lb, Eigen::VectorXd &ub);
    virtual void name(double* t, int n_breaks,std::vector<std::string> &name_str);
    virtual void updateRobot(RigidBodyManipulator *robot);
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
    virtual void eval_valid(double* valid_t, int num_valid_t,const Eigen::MatrixXd &valid_q,Eigen::VectorXd &c, Eigen::MatrixXd &dc_valid);
    virtual void bounds(double* t,int n_breaks, Eigen::VectorXd &lb, Eigen::VectorXd &ub);
    virtual void name(double* t, int n_breaks,std::vector<std::string> &name_str);
    virtual void updateRobot(RigidBodyManipulator *robot);
    virtual ~WorldFixedOrientConstraint(void){};
};

class WorldFixedBodyPoseConstraint: public MultipleTimeKinematicConstraint
{
  protected:
    int body;
    std::string body_name;
  public:
    WorldFixedBodyPoseConstraint(RigidBodyManipulator* model, int body, const Eigen::Vector2d &tspan);
    virtual int getNumConstraint(double* q, int n_breaks);
    virtual void eval_valid(double* valid_t, int num_valid_t,const Eigen::MatrixXd &valid_q,Eigen::VectorXd &c, Eigen::MatrixXd &dc_valid);
    virtual void bounds(double* t,int n_breaks, Eigen::VectorXd &lb, Eigen::VectorXd &ub);
    virtual void name(double* t, int n_breaks,std::vector<std::string> &name_str);
    virtual void updateRobot(RigidBodyManipulator *robot);
    virtual ~WorldFixedBodyPoseConstraint(void){};
};

class AllBodiesClosestDistanceConstraint : public SingleTimeKinematicConstraint
{
  protected:
    double ub;
    double lb;
  public:
    AllBodiesClosestDistanceConstraint(RigidBodyManipulator* model, 
                                       double lb, double ub,
                                       Eigen::Vector2d tspan);
    virtual void eval(double* t,Eigen::VectorXd& c, Eigen::MatrixXd& dc);
    virtual void name(double* t, std::vector<std::string> &name);
    virtual void bounds(double* t, Eigen::VectorXd& lb, Eigen::VectorXd& ub);
    virtual void updateRobot(RigidBodyManipulator *robot);
    virtual ~AllBodiesClosestDistanceConstraint(){};
};

class WorldPositionInFrameConstraint: public WorldPositionConstraint
{
  protected:
    Eigen::Matrix4d T_world_to_frame;
    Eigen::Matrix4d T_frame_to_world;
    virtual void evalPositions(Eigen::MatrixXd &pos, Eigen::MatrixXd &J);
  public:
    WorldPositionInFrameConstraint(RigidBodyManipulator *model, int body, 
        const Eigen::MatrixXd &pts, const Eigen::Matrix4d& T_world_to_frame, 
        Eigen::MatrixXd lb, Eigen::MatrixXd ub, const Eigen::Vector2d &tspan);
    //virtual void updateRobot(RigidBodyManipulator *robot);
    virtual ~WorldPositionInFrameConstraint();
};
#endif
