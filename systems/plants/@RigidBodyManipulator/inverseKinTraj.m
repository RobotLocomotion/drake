function [xtraj,info,infeasible_constraint]= inverseKinTraj(obj,q0,qd0,t,q_seed,q_nom,varargin)
% inverseKinSequence(obj,t,q0,qd0,q_seed,q_nom,constraint1,constraint2,constraint3,...,options)
% solve IK
%   min_q sum_i qdd(:,i)'*Qa*qdd(:,i)+qd(:,i)'*Qv*qd(:,i)+(q(:,i)-q_nom(:,i))'*Q*(q(:,i)-q_nom(:,i))]
%   subject to
%          constraint1 at t(i)
%          constraint2 at t(i)
%          ...
%          constraint(k)   at [t(2) t(3) ... t(nT)]
%          constraint(k+1) at [t(2) t(3) ... t(nT)]
%   ....
% using q_seed as the initial guess. 
% Note Qv is default to be zero.
% q is supposed to be a cubic spline, qdd and qd are the acceleration and
% velocity of this cubic spline.
% @param q0            The initial posture at time t(1);
% @param qd0           The initial velocity at time t(1);
% @param t             the sampling time, a row vector
% @param q_seed        the seed guess, a matrix, with q_seed(:,i) as the
%                      seed guess at time t(i+1)
% @param q_nom         the nominal posture, a matrix, with q_nom(:,i) as the
%                      nominal posture at time t(i+1)
% @param constraint    A Constraint object, can be a KinematicConstraint object,
%                      A QuasiStaticConstraint, or a
%                      PostureConstraint object
% @param options       an IKoptions object
% @retval xtraj        the IK trajectory for x = [q;qdot]. A cubic spline.
% @retval info         1,2,3 or 4 are acceptable
%                    1  -- Successful, q is optimal and feasible
%                    2  -- q is feasible
%                    3  -- Solve the problem, but not to the desired accuracy
%                    4  -- SNOPT thinks it is not feasible, but we check the
%                         constraint, and they are satisfied within the tolerance
%                    13 -- the problem is infeasible. Set the debug equalt
%                           to true in the IKoptions by calling setDebug.
%                           And check infeasible_constraint to retrieve the
%                           name of the infeasible constraint
%                    32 -- the problem terminates too early, change the
%                          major iterations limit in options by calling
%                          setMajorIterationsLimit function 
%                    For any other info, it means the IK program is
%                    problematic, please contact the author
% @retval infeasible_constraint
%                    Optional return argument. A cell of constraint names
%                    that are infeasible
ikoptions = varargin{end};
kc = varargin(1:end-1);
ikoptions = ikoptions.setq0(q0);
ikoptions = ikoptions.setqd0(qd0);
q_seed = [ikoptions.q0 q_seed];
q_nom = [ikoptions.q0 q_nom];
if(nargout == 3)
[q,qdot,qddot,info,infeasible_constraint] = inverseKinBackend(obj,2,t,q_seed,q_nom,kc{:},ikoptions);
elseif(nargout == 2)
  [q,qdot,qddot,info] = inverseKinBackend(obj,2,t,q_seed,q_nom,kc{:},ikoptions);
end
xtraj = PPTrajectory(pchipDeriv(t,[q;qdot],[qdot;qddot]));
xtraj = xtraj.setOutputFrame(obj.getStateFrame());
end