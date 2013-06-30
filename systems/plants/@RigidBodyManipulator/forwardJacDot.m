function Jdot = forwardJacDot(obj,kinsol,body_ind,pts)

% same input as [x,J] = forwardKin but returns Jdot
% note: you must have called kinsol = doKinematics with qd passed in as the
% last argument

if nargin<4, pts=[]; end

if (kinsol.mex)
  if (isa(body_ind,'RigidBody')) body_ind = find(obj.body==body_ind,1); end

  if (obj.mex_model_ptr==0)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is no longer valid because the mex model ptr has been deleted.');
  end
  if  ~isnumeric(pts)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is not valid because it was computed via mex, and you are now asking for an evaluation with non-numeric pts.  If you intended to use something like TaylorVar, then you must call doKinematics with use_mex = false');
  end
  
  Jdot = forwardKinmex(obj.mex_model_ptr,kinsol.q,body_ind,pts,false,true);
else
  if ~all(abs(kinsol.q-[obj.body.cached_q]')<1e-8)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is not longer valid.  Somebody has called doKinematics with a different q since the solution was computed.  If this happens a lot, I could consider returning the full T tree in kinsol, so I don''t have to rely on this caching mechanism');
  end
  if ~all(abs(kinsol.qd-[obj.body.cached_qd]')<1e-8)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is not longer valid.  Somebody has called doKinematics with a different q since the solution was computed.  If this happens a lot, I could consider returning the full T tree in kinsol, so I don''t have to rely on this caching mechanism');
  end
  
  if (body_ind == 0) 
    nq=getNumDOF(obj);
    
    % return center of mass for the entire model
    m=0;
    Jdot = zeros(3,nq);
    
    for i=1:length(obj.body)
      bm = obj.body(i).mass;
      if (bm>0)
        bc = obj.body(i).com;
        bJdot = forwardJacDot(obj,kinsol,i,bc);
        Jdot = (m*Jdot + bm*bJdot)/(m+bm);
        m = m + bm;
      end
    end
  else
    if (isa(body_ind,'RigidBody')) body=body_ind;
    else body = obj.body(body_ind); end
    
    m = size(pts,2);
    pts = [pts;ones(1,m)];
    
    Jdot = reshape(body.dTdqdot*pts,obj.getNumDOF,[])';
  end
  
end