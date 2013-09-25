function [q,info,infeasible_constraint] = inverseKin(obj,q_seed,q_nom,varargin)
% inverseKin(obj,q_seed,q_nom,constraint1,constraint2,constraint3,...,options)
% solve IK 
%   min_q (q-q_nom)'*Q*(q-q_nom)
%   subject to
%          constraint1 at t
%          constraint2 at t 
%   .....
%   using q_seed as the initial guess
% 
% @param q_seed       The seed guess, a column vector
% @param q_nom        The nominal posture, a column vector
% @param constraint   A Constraint class object, currently support KinematicConstraint,
%                     PostureConstraint and QuasiStaticConstraint. There
%                     can be at most 1 QuasiStaticConstraint
% @param ikoptions    An IKoptions object, please refer to IKoptions for detail
% @retval q           the IK solution posture
% @retval info        1,2,3 or 4 are acceptable
%                     1  -- Successful, q is optimal and feasible
%                     2  -- q is feasible
%                     3  -- Solve the problem, but not to the desired accuracy
%                     4  -- SNOPT thinks it is not feasible, but we check the
%                           constraint, and they are satisfied within the tolerance
%                     13 -- the problem is infeasible. Set the debug equalt
%                           to true in the IKoptions by calling setDebug.
%                           And check infeasible_constraint to retrieve the
%                           name of the infeasible constraint
%                     32 -- the problem terminates too early, change the
%                           major iterations limit in options by calling
%                           setMajorIterationsLimit function
%                     For any other info, it means the IK program is
%                     problematic, please contact the author
% @retval infeasible_constraint
%                     Optional return argument. A cell of constraint names
%                     that are infeasible.
% note: keeping typecheck/sizecheck to a minimum because this might have to
% run inside a dynamical system (so should be fast)
if(nargout == 3)
  [q,info,infeasible_constraint]= inverseKinBackend(obj,1,[],q_seed,q_nom,varargin{:});
else
  [q,info]= inverseKinBackend(obj,1,[],q_seed,q_nom,varargin{:});
end
end