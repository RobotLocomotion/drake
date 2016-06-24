#ifndef _DYN_EIGEN_QUADSOLVE_HPP_
#define _DYN_EIGEN_QUADSOLVE_HPP_

#include <iostream>

/*
   FILE eiquadprog.hh
NOTE: this is a modified of uQuadProg++ package, working with Eigen data structures. 
uQuadProg++ is itself a port made by Angelo Furfaro of QuadProg++ originally developed by 
Luca Di Gaspero, working with ublas data structures. 
The quadprog_solve() function implements the algorithm of Goldfarb and Idnani 
for the solution of a (convex) Quadratic Programming problem
by means of a dual method.
The problem is in the form:
min 0.5 * x G x + g0 x
_s.t.
CE^T x + ce0 = 0
CI^T x + ci0 >= 0
The matrix and vectors dimensions are as follows:
G: n * n
g0: n
CE: n * p
ce0: p
CI: n * m
ci0: m
x: n
The function will return the cost of the solution written in the x vector or
std::numeric_limits::infinity() if the problem is infeasible. In the latter case
the value of the x vector is not correct.
References: D. Goldfarb, _A. Idnani. _A numerically stable dual method for solving
strictly convex quadratic programs. Mathematical Programming 27 (1983) pp. 1-33.
Notes:
1. pay attention in setting up the vectors ce0 and ci0. 
If the constraints of your problem are specified in the form 
_A^T x = b and C^T x >= _d, then you should set ce0 = -b and ci0 = -_d.
2. The matrix G is modified within the function since it is used to compute
the G = L^T L cholesky factorization for further computations inside the function. 
If you need the original matrix G you should make a copy of it and pass the copy
to the function.
The author will be grateful if the researchers using this software will
acknowledge the contribution of this modified function and of Di Gaspero'_s
original version in their research papers.
LICENSE
Copyright (2011) Benjamin Stephens
Copyright (2010) Gael Guennebaud
Copyright (2008) Angelo Furfaro
Copyright (2006) Luca Di Gaspero
This file is a porting of QuadProg++ routine, originally developed
by Luca Di Gaspero, exploiting uBlas data structures for vectors and
matrices instead of native C++ array.
uquadprog is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.
uquadprog is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR _A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with uquadprog; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Cholesky>

namespace Eigen {
  class QP {
    private:
      int _nVar;
      int _nEq;
      int _nIneq;

      LLT<MatrixXd,Lower> _chol;
      MatrixXd _R, _J;
      VectorXd _s, _z, _r, _d, _np, _u, _x_old, _u_old;
      VectorXi _A, _A_old, _iai, _iaexcl;

      
      template<typename Scalar>
        inline Scalar distance(Scalar a, Scalar b)
        {
          Scalar a1, b1, t;
          a1 = std::abs(a);
          b1 = std::abs(b);
          if (a1 > b1) 
          {
            t = (b1 / a1);
            return a1 * std::sqrt(1.0 + t * t);
          }
          else
            if (b1 > a1)
            {
              t = (a1 / b1);
              return b1 * std::sqrt(1.0 + t * t);
            }
          return a1 * std::sqrt(2.0);
        }


      inline void compute_d(VectorXd &_d, const MatrixXd& _J, const VectorXd& _np)
      {
        _d = _J.adjoint() * _np;
      }

      inline void update_z(VectorXd& _z, const MatrixXd& _J, const VectorXd& _d,  int iq)
      {
        _z = _J.rightCols(_z.size()-iq) * _d.tail(_d.size()-iq);
      }

      inline void update_r(const MatrixXd& _R, VectorXd& _r, const VectorXd& _d, int iq) 
      {
        _r.head(iq)= _R.topLeftCorner(iq,iq).triangularView<Upper>().solve(_d.head(iq));
      }

      /* solve_quadprog2 is used for when the Cholesky decomposition of G is pre-computed */
      double solve_quadprog2(LLT<MatrixXd,Lower> &chol,  double c1, const VectorXd & g0,  
          const MatrixXd & CE, const VectorXd & ce0,  
          const MatrixXd & CI, const VectorXd & ci0, 
          VectorXd& x);
      bool add_constraint(MatrixXd& _R, MatrixXd& _J, VectorXd& _d, int& iq, double& R_norm);
      void delete_constraint(MatrixXd& _R, MatrixXd& _J, VectorXi& _A, VectorXd& _u,  int p, int& iq, int l);

    public:
      void resize(int nVar, int nEq, int nIneq);
      /* solve_quadprog is used for on-demand QP solving */
      double solve_quadprog(MatrixXd & G,  const VectorXd & g0,  
          const MatrixXd & CE, const VectorXd & ce0,  
          const MatrixXd & CI, const VectorXd & ci0, 
          VectorXd& x);
  };
}

#endif
