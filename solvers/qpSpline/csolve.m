% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(quad_form(y0_1, Q0) + quad_form(y0_2, Q0) + quad_form(y0_3, Q0) + quad_form(y0_4, Q0) + quad_form(y0_5, Q0) + quad_form(y0_6, Q0) + quad_form(y1_1, Q1) + quad_form(y1_2, Q1) + quad_form(y1_3, Q1) + quad_form(y1_4, Q1) + quad_form(y1_5, Q1) + quad_form(y1_6, Q1) + quad_form(y2_1, Q2) + quad_form(y2_2, Q2) + quad_form(y2_3, Q2) + quad_form(y2_4, Q2) + quad_form(y2_5, Q2) + quad_form(y2_6, Q2))
%   subject to
%     C0_0 == x0
%     C0_1 == xd0
%     C1_0 == x1
%     C2_0 == x2
%     C0_0 + (t1 - t0)*C0_1 + (t1 - t0)*(t1 - t0)*C0_2 + (t1 - t0)*(t1 - t0)*(t1 - t0)*C0_3 == C1_0
%     C1_0 + (t2 - t1)*C1_1 + (t2 - t1)*(t2 - t1)*C1_2 + (t2 - t1)*(t2 - t1)*(t2 - t1)*C1_3 == C2_0
%     C2_0 + (tf - t2)*C2_1 + (tf - t2)*(tf - t2)*C2_2 + (tf - t2)*(tf - t2)*(tf - t2)*C2_3 == xf
%     C0_1 + 2*(t1 - t0)*C0_2 + 3*(t1 - t0)*(t1 - t0)*C0_3 == C1_1
%     C1_1 + 2*(t2 - t1)*C1_2 + 3*(t2 - t1)*(t2 - t1)*C1_3 == C2_1
%     C2_1 + 2*(tf - t2)*C2_2 + 3*(tf - t2)*(tf - t2)*C2_3 == xdf
%     2*C0_2 + 6*(t1 - t0)*C0_3 == 2*C1_2
%     2*C1_2 + 6*(t2 - t1)*C1_3 == 2*C2_2
%     y0_1(1) == C0_2(1)
%     y0_2(1) == C0_2(2)
%     y0_3(1) == C0_2(3)
%     y0_4(1) == C0_2(4)
%     y0_5(1) == C0_2(5)
%     y0_6(1) == C0_2(6)
%     y0_1(2) == C0_3(1)
%     y0_2(2) == C0_3(2)
%     y0_3(2) == C0_3(3)
%     y0_4(2) == C0_3(4)
%     y0_5(2) == C0_3(5)
%     y0_6(2) == C0_3(6)
%     y1_1(1) == C1_2(1)
%     y1_2(1) == C1_2(2)
%     y1_3(1) == C1_2(3)
%     y1_4(1) == C1_2(4)
%     y1_5(1) == C1_2(5)
%     y1_6(1) == C1_2(6)
%     y1_1(2) == C1_3(1)
%     y1_2(2) == C1_3(2)
%     y1_3(2) == C1_3(3)
%     y1_4(2) == C1_3(4)
%     y1_5(2) == C1_3(5)
%     y1_6(2) == C1_3(6)
%     y2_1(1) == C2_2(1)
%     y2_2(1) == C2_2(2)
%     y2_3(1) == C2_2(3)
%     y2_4(1) == C2_2(4)
%     y2_5(1) == C2_2(5)
%     y2_6(1) == C2_2(6)
%     y2_1(2) == C2_3(1)
%     y2_2(2) == C2_3(2)
%     y2_3(2) == C2_3(3)
%     y2_4(2) == C2_3(4)
%     y2_5(2) == C2_3(5)
%     y2_6(2) == C2_3(6)
%
% with variables
%     C0_0   6 x 1
%     C0_1   6 x 1
%     C0_2   6 x 1
%     C0_3   6 x 1
%     C1_0   6 x 1
%     C1_1   6 x 1
%     C1_2   6 x 1
%     C1_3   6 x 1
%     C2_0   6 x 1
%     C2_1   6 x 1
%     C2_2   6 x 1
%     C2_3   6 x 1
%     y0_1   2 x 1
%     y0_2   2 x 1
%     y0_3   2 x 1
%     y0_4   2 x 1
%     y0_5   2 x 1
%     y0_6   2 x 1
%     y1_1   2 x 1
%     y1_2   2 x 1
%     y1_3   2 x 1
%     y1_4   2 x 1
%     y1_5   2 x 1
%     y1_6   2 x 1
%     y2_1   2 x 1
%     y2_2   2 x 1
%     y2_3   2 x 1
%     y2_4   2 x 1
%     y2_5   2 x 1
%     y2_6   2 x 1
%
% and parameters
%       Q0   2 x 2    PSD
%       Q1   2 x 2    PSD
%       Q2   2 x 2    PSD
%       t0   1 x 1
%       t1   1 x 1
%       t2   1 x 1
%       tf   1 x 1
%       x0   6 x 1
%       x1   6 x 1
%       x2   6 x 1
%      xd0   6 x 1
%      xdf   6 x 1
%       xf   6 x 1
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.Q0, ..., params.xf, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2015-04-06 22:08:40 -0400.
% CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2012 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
