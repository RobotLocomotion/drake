function Jdot = forwardJacDot(obj,kinsol,body_or_frame_ind,pts,rotation_type,robotnum)

warnOnce(obj.warning_manager,'Drake:RigidBodyManipulator:Inefficient','Inefficient call. If you''re already computing J using forwardKin, it is faster to call forwardKin with gradient output and compute Jdot from dJ: reshape(reshape(dJ, numel(J), []) * kinsol.vToqdot * kinsol.v, size(J)).');

% same input as [x,J] = forwardKin but returns Jdot
% note: you must have called kinsol = doKinematics with qd passed in as the
% last argument
if nargin<6, robotnum=1; end
if nargin<5, rotation_type=0; end
if nargin<4, pts=[]; end

% todo: zap this after the transition
if isa(body_or_frame_ind,'RigidBody'), error('support for passing in RigidBody objects has been removed.  please pass in the body index'); end

if (kinsol.mex)
  if (obj.mex_model_ptr==0)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is no longer valid because the mex model ptr has been deleted.');
  end
  if  ~isnumeric(pts)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is not valid because it was computed via mex, and you are now asking for an evaluation with non-numeric pts.  If you intended to use something like TaylorVar, then you must call doKinematics with use_mex = false');
  end
  if(body_or_frame_ind == 0)
    Jdot = forwardKinmex(obj.mex_model_ptr,kinsol.q,0,robotnum,true);
  else
    Jdot = forwardKinmex(obj.mex_model_ptr,kinsol.q,body_or_frame_ind,pts,rotation_type,true);
  end
else
  compute_com_jacobian_dot = body_or_frame_ind == 0;
  
  if compute_com_jacobian_dot
    [~,J,dJ] = getCOM(obj,kinsol,robotnum);
  else
    [~,J,dJ] = forwardKin(obj, kinsol, body_or_frame_ind,pts,rotation_type);
  end
  dJ = reshape(dJ, numel(J), []);
  qd = kinsol.vToqdot * kinsol.v;
  Jdot = reshape(dJ * qd, size(J));
end