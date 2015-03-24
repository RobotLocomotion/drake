% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(max_accel)
%   subject to
%     max_accel >= abs(2*C1_2)
%     max_accel >= abs(2*C1_2 + 6*t_switch*C1_3)
%     max_accel >= abs(2*C2_2)
%     max_accel >= abs(2*C2_2 + 6*(t_f - t_switch)*C2_3)
%     C1_0 + t_switch*C1_1 + t_switch*t_switch*C1_2 + t_switch*t_switch*t_switch*C1_3 == C2_0
%     C1_1 + 2*t_switch*C1_2 + 3*t_switch*t_switch*C1_3 == C2_1
%     C1_0 == r0
%     C1_1 == rd0
%     C2_0 + (t_f - t_switch)*C2_1 + (t_f - t_switch)*(t_f - t_switch)*C2_2 + (t_f - t_switch)*(t_f - t_switch)*(t_f - t_switch)*C2_3 == rf
%     C2_1 + 2*(t_f - t_switch)*C2_2 + 3*(t_f - t_switch)*(t_f - t_switch)*C2_3 == rdf
%     abs(C2_0 - r_switch) <= r_switch_slack
%     abs(C2_1 - rd_switch) <= rd_switch_slack
%
% with variables
%     C1_0   6 x 1
%     C1_1   6 x 1
%     C1_2   6 x 1
%     C1_3   6 x 1
%     C2_0   6 x 1
%     C2_1   6 x 1
%     C2_2   6 x 1
%     C2_3   6 x 1
% max_accel   1 x 1
%
% and parameters
%       r0   6 x 1
% r_switch   6 x 1
% r_switch_slack   6 x 1    positive
%      rd0   6 x 1
% rd_switch   6 x 1
% rd_switch_slack   6 x 1    positive
%      rdf   6 x 1
%       rf   6 x 1
%      t_f   1 x 1
% t_switch   1 x 1
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.r0, ..., params.t_switch, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2015-03-24 13:12:44 -0400.
% CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2012 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
