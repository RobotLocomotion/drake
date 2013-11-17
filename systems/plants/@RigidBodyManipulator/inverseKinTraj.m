function [xtraj,info,infeasible_constraint]= inverseKinTraj(obj,t,q_seed_traj,q_nom_traj,varargin)
% inverseKinSequence(obj,t,q_seed_traj,q_nom_traj,constraint1,constraint2,constraint3,...,options)
%
% If options.fixInitialState = true
% solve IK
%   min_q sum_i qdd(:,i)'*Qa*qdd(:,i)+qd(:,i)'*Qv*qd(:,i)+(q(:,i)-q_nom(:,i))'*Q*(q(:,i)-q_nom(:,i))]
%   subject to
%          constraint1 at t_samples(i)
%          constraint2 at t_samples(i)
%          ...
%          constraint(k)   at [t_samples(2) t_samples(3) ... t_samples(nT)]
%          constraint(k+1) at [t_samples(2) t_samples(3) ... t_samples(nT)]
%   ....
%
% using q_seed_traj as the initial guess. q(1) would be fixed to
% q_seed_traj.eval(t(1))
% t_samples = unique([t,options.additional_tSamples]);
% In the objective function, the 'q,qd,qdd' are evaluated at knot time t only.
% If options.fixInitialState = false
% solve the same optimization problem, but q(1) and qdot(1) would be
% decision variables, and the constraints would be enforced at time t(1)
% also
% Note Qv is default to be zero.
% q is supposed to be a cubic spline, qdd and qd are the acceleration and
% velocity of this cubic spline.
% @param t             the knot time, a row vector
% @param q_seed_traj   the seed guess, a Trajectory object
% @param q_nom_traj    the nominal posture, a Trajectory object
% @param constraint    A RigidBodyConstraint object, can be a SingleTimeKinematicConstraint object,
%                      A MultipleTimeKinematicConstraint, A QuasiStaticConstraint, or a
%                      PostureConstraint object
% @param options       an IKoptions object
% @retval xtraj        the IK trajectory for x = [q;qdot]. A cubic spline.
% @retval info         1,2,3 or 4 are acceptable
%                      - 1, Successful, q is optimal and feasible
%                      - 2, q is feasible
%                      - 3, Solve the problem, but not to the desired accuracy
%                      - 4, SNOPT thinks it is not feasible, but we check the
%                           constraint, and they are satisfied within the tolerance
%                      - 13, the problem is infeasible. Set the debug equalt
%                            to true in the IKoptions by calling setDebug.
%                            And check infeasible_constraint to retrieve the
%                            name of the infeasible constraint
%                      - 32, the problem terminates too early, change the
%                            major iterations limit in options by calling
%                            setMajorIterationsLimit function
%                      - For any other info, it means the IK program is
%                        problematic, please contact the author
% @retval infeasible_constraint
%                    Optional return argument. A cell of constraint names
%                    that are infeasible
ikoptions = varargin{end};
kc = varargin(1:end-1);
q_seed = q_seed_traj.eval(t);
q_nom = q_nom_traj.eval(t);
% [f,G,iGfun,jGvar,Fupp,Flow,xupp,xlow,iAfun,jAvar,A,velocity_mat,velocity_mat_qd0,velocity_mat_qdf,...
%   accel_mat,accel_mat_qd0,accel_mat_qdf,dqInbetweendqknot,dqInbetweendqd0,dqInbetweendqdf,iCfun_inbetween_cell,jCvar_inbetween_cell] = ...
% inverseKinBackend(obj,2,t,q_seed,q_nom,kc{:},ikoptions);
% save('mexdata.mat','f','G','iGfun','jGvar','Fupp','Flow','xupp','xlow','iAfun','jAvar','A','dqInbetweendqknot','dqInbetweendqd0','dqInbetweendqdf','iCfun_inbetween_cell','jCvar_inbetween_cell');
% ikoptions = ikoptions.setMex(false);
% keyboard;
if(nargout == 3)
  [q,qdot,qddot,info,infeasible_constraint] = inverseKinBackend(obj,2,t,q_seed,q_nom,kc{:},ikoptions);
elseif(nargout <= 2)
  [q,qdot,qddot,info] = inverseKinBackend(obj,2,t,q_seed,q_nom,kc{:},ikoptions);
end
xtraj = PPTrajectory(pchipDeriv(t,[q;qdot],[qdot;qddot]));
xtraj = xtraj.setOutputFrame(obj.getStateFrame());
end