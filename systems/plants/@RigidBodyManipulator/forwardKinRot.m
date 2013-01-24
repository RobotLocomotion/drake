function [rpy,J] = forwardKinRot(obj,kinsol,body_ind)

  if ~all(abs(kinsol.q-[obj.body.cached_q]')<1e-8)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is not longer valid.  Somebody has called doKinematics with a different q since the solution was computed.  If this happens a lot, I could consider returning the full T tree in kinsol, so I don''t have to rely on this caching mechanism');
  end
  if (kinsol.mex)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol was produced with mex, but we haven''t actually implemented the mexed version of this function yet.  call doKinematics with use_mex = false');
  end
  
  if (isa(body_ind,'RigidBody')) body=body_ind;
  else body = obj.body(body_ind); end
  
  R = body.T(1:3,1:3);

  % NOTE: assumes we're using an X-Y-Z convention to construct R
  roll = atan2(R(3,2),R(3,3));
  pitch = atan2(-R(3,1),sqrt(R(3,2)^2 + R(3,3)^2));
  yaw = atan2(R(2,1),R(1,1));
  rpy = [roll; pitch; yaw];

  if (nargout>1)
    nq = size(body.dTdq,1)/4;
    J = zeros(3,nq);
    % note the unusual format of dTdq 
    % dTdq = [dT(1,:)dq1; dT(1,:)dq2; ...; dT(1,:)dqN; dT(2,dq1) ...]
    
    % droll_dq
    idx = sub2ind(size(body.dTdq),(3-1)*nq+(1:nq),2*ones(1,nq));
    dR32_dq = body.dTdq(idx);
    idx = sub2ind(size(body.dTdq),(3-1)*nq+(1:nq),3*ones(1,nq));
    dR33_dq = body.dTdq(idx);
    sqterm = R(3,2)^2 + R(3,3)^2;
    J(1,:) = (R(3,3)*dR32_dq - R(3,2)*dR33_dq)/sqterm;

    % dpitch_dq
    idx = sub2ind(size(body.dTdq),(3-1)*nq+(1:nq),ones(1,nq));
    dR31_dq = body.dTdq(idx);
    J(2,:) = (-sqrt(sqterm)*dR31_dq + R(3,1)/sqrt(sqterm)*(R(3,2)*dR32_dq + R(3,3)*dR33_dq) )/(R(3,1)^2 + R(3,2)^2 + R(3,3)^2);

    % dyaw_dq
    idx = sub2ind(size(body.dTdq),(1-1)*nq+(1:nq),ones(1,nq));
    dR11_dq = body.dTdq(idx);
    idx = sub2ind(size(body.dTdq),(2-1)*nq+(1:nq),ones(1,nq));
    dR21_dq = body.dTdq(idx);
    sqterm = R(1,1)^2 + R(2,1)^2;
    J(3,:) = (R(1,1)*dR21_dq - R(2,1)*dR11_dq)/sqterm;
    
    if (nargout>2)
      error('ForwardKinRot: second derivatives not implemented yet');
      %if isempty(body.ddTdqdq)
      %  error('you must call doKinematics with the second derivative option enabled');
      %end
    end
  end

end