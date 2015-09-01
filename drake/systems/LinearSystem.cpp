
#include "LinearSystem.h"
#include "drakeUtil.h"

AffineSystem::AffineSystem(const std::string& name,
                           const Eigen::MatrixXd& _Ac,const Eigen::MatrixXd& _Bc,const Eigen::VectorXd& _xcdot0,
                           const Eigen::MatrixXd& _Ad,const Eigen::MatrixXd& _Bd,const Eigen::VectorXd& _xdn0,
                           const Eigen::MatrixXd& _C,const Eigen::MatrixXd& _D,const Eigen::VectorXd& _y0) :
        DrakeSystem(name, _Ac.rows(),_Ad.rows(),_D.cols(),_C.rows()),
        Ac(_Ac), Bc(_Bc), xcdot0(_xcdot0), Ad(_Ad), Bd(_Bd), xdn0(_xdn0), C(_C), D(_D), y0(_y0)
{
  int num_xc = continuous_state_frame->getDim(),
      num_xd = discrete_state_frame->getDim(),
      num_u = input_frame->getDim(),
      num_y = output_frame->getDim();
  int num_x = num_xc+num_xd;

  sizecheck(Ac,num_xc,num_x);
  sizecheck(Bc,num_xc,num_u);
  sizecheck(xcdot0,num_xc,1);
  sizecheck(Ad,num_xd,num_x);
  sizecheck(Bd,num_xc,num_u);
  sizecheck(xdn0,num_xd,1);
  sizecheck(C,num_y,num_x);
  sizecheck(D,num_y,num_u);
  sizecheck(y0,num_y,1);
}


LinearSystem::LinearSystem(const std::string& name,
                           const Eigen::MatrixXd& _Ac,const Eigen::MatrixXd& _Bc,
                           const Eigen::MatrixXd& _Ad,const Eigen::MatrixXd& _Bd,
                           const Eigen::MatrixXd& _C,const Eigen::MatrixXd& _D) :
        AffineSystem(name,
                     _Ac,_Bc,Eigen::VectorXd::Zero(_Ac.rows()),
                     _Ad,_Bd,Eigen::VectorXd::Zero(_Ad.rows()),
                     _C,_D,Eigen::VectorXd::Zero(_C.rows()))
{}