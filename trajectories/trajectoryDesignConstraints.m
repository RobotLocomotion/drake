% Help file for trajectory design constraints. 
%
% All of the trajectory design algorithms support all of 
% these constraints (or produce a warning if they do not).
%
% Assuming that the constraint structure is name 'con', valid 
% constraints specified in the structure are:
%
% Input constraints:
%  con.u.lb        forall t, lb <= u(t) <= ub
%  con.u.ub 
%
% State constraints: 
%  con.x.lb        forall t, lb <= x(t) <= ub
%  con.x.ub 
%  con.x.A         forall t, A x(t) <= b
%  con.x.b
%  con.x.Aeq       forall t, Aeq x(t) = beq
%  con.x.beq
%  con.x.c         forall t, c(x(t)) <= 0
%  con.x.ceq       forall t, ceq(x(t)) == 0
%     % to provide gradients of the constraint, these functions should
%     % output  [f,dfdx] = c(x) , where size(dfdx) = [len(f),len(x)]
%
% Initial State Constraints:
%  con.x0.*  
% can take any of the fields described for con.x , but applies only for t = t0.
%
% Final State Constraints:
%  con.xf.*  
% can take any of the fields described for con.x , but applies only for t = tf.
%
% Time Constraints:
%  con.T.lb     lb <= tf - t0 <= ub
%  con.T.ub 
%
% Hybrid Mode Constraints
% For hybrid systems, all of the constraints above can exist inside a
% mode structure, e.g.
%  con.mode(i).mode_num    
%  con.mode(i).u    input (for this mode) constraint structure defined above
%  con.mode(i).x    state (for this mode) constraint structure defined above
%  con.mode(i).x0   initial (for this mode) state constraint structure defined above
%  con.mode(i).xf   final (for this mode) state constraint structure defined above
%  con.mode(i).T    time (for this mode) constraint structure defined above
% Note that the mode indices enforce a total ordering of the modes in the solution 
%
% Periodicity Constraints
%  con.periodic = true       x(t0) = x(tf)
%


% More things to implement / think about:
%  - do i ever need to handle c(x,u) type constraints?
%  - will users want to call c and ceq at the same time, e.g. to save
%    computation time?  it's important at the trajectory level, but probably
%    not at the individual x level, right?=======
% todo:  do i ever need to handle c(t,x,u) type constraints?

