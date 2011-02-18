function [utraj,xtraj,info] = shooting(sys,costFun,finalCostFun,T0,x0,utraj0,con,options)

% Shooting method for trajectory optimization
%
% Input Arguments:
%  sys          - instance of the DynamicalSystem class
%  costfun      - [g,dg] = costFun(t,x,u) or []
%  finalCostFun - [h,dh] = finalCostFun(t,x)  or []
%  T0           - initial duration (tf - t0)
%  x0           - initial state
%  utraj0       - initial input tape (must be defined over at least [t0,t0+T0] )
%  con          - structure of <a href="matlab:help('trajectoryConstraints')">trajectoryConstraints</a>
%  options      - options structure (see below)
% 
% Output Arguments:
%  utraj        - the optimized control trajectory
%  xtraj        - the system trajectory while executing utraj from x0
%  success      - true if optimization is successful, false on failure
%
% Supported options:  (for details see <a href="matlab:help('trajectoryDesignOptions')">trajectoryDesignOptions</a>
%  ode_solver 
%  optimizer
%  grad_method
%  ...
%


t = utraj0.getBreaks()';
u = utraj0.eval(t);



