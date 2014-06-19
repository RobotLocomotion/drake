function Jdot = forwardJacDot(obj,kinsol,body_or_frame_ind,pts,rotation_type,robotnum)

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
  
  if isempty(kinsol.dTdqdot{1})
    error('Drake:RigidBodyManipulator:MissingDerivatives','This kinsol does not have dTdqdot.  You must pass qd into doKinematics before using this method');
  end
  
  if (body_or_frame_ind == 0) 
    nq=getNumPositions(obj);
    
    % return center of mass for the entire model
    m=0;
    Jdot = zeros(3,nq);
    
    for i=1:length(obj.body)
      if(any(obj.body(i).robotnum == robotnum))
        bm = obj.body(i).mass;
        if (bm>0)
          bc = obj.body(i).com;
          bJdot = forwardJacDot(obj,kinsol,i,bc);
          Jdot = (m*Jdot + bm*bJdot)/(m+bm);
          m = m + bm;
        end
      end
    end
  else
    if (body_or_frame_ind < 0)
      frame = obj.frame(-body_or_frame_ind);
      body_ind = frame.body_ind;
      Tframe = frame.T;
    else
      body_ind = body_or_frame_ind;
      Tframe=eye(4);
    end
  
    m = size(pts,2);
    pts = [pts;ones(1,m)];
    
    if rotation_type==0
      Jdot = reshape(kinsol.dTdqdot{body_ind}*Tframe*pts,obj.getNumPositions,[])';
    elseif rotation_type==1
      T = kinsol.T{body_ind}*Tframe;
      R = T(1:3,1:3);
      nq=getNumPositions(obj);
      dTdqdot = kinsol.dTdqdot{body_ind}*Tframe;
      Jx = reshape(dTdqdot*pts,nq,[])';

      Jr = zeros(3,nq);
      % note the unusual format of dTdq 
      % dTdq = [dT(1,:)dq1; dT(1,:)dq2; ...; dT(1,:)dqN; dT(2,dq1) ...]
    
      % droll_dqdot
      idx = sub2ind(size(dTdqdot),(3-1)*nq+(1:nq),2*ones(1,nq));
      dR32_dqdot = dTdqdot(idx);
      idx = sub2ind(size(dTdqdot),(3-1)*nq+(1:nq),3*ones(1,nq));
      dR33_dqdot = dTdqdot(idx);
      sqterm = R(3,2)^2 + R(3,3)^2;
      Jr(1,:) = (R(3,3)*dR32_dqdot - R(3,2)*dR33_dqdot)/sqterm;

      % dpitch_dqdot
      idx = sub2ind(size(dTdqdot),(3-1)*nq+(1:nq),ones(1,nq));
      dR31_dqdot = dTdqdot(idx);
      Jr(2,:) = (-sqrt(sqterm)*dR31_dqdot + R(3,1)/sqrt(sqterm)*(R(3,2)*dR32_dqdot + R(3,3)*dR33_dqdot) )/(R(3,1)^2 + R(3,2)^2 + R(3,3)^2);

      % dyaw_dqdot
      idx = sub2ind(size(dTdqdot),(1-1)*nq+(1:nq),ones(1,nq));
      dR11_dqdot = dTdqdot(idx);
      idx = sub2ind(size(dTdqdot),(2-1)*nq+(1:nq),ones(1,nq));
      dR21_dqdot = dTdqdot(idx);
      sqterm = R(1,1)^2 + R(2,1)^2;
      Jr(3,:) = (R(1,1)*dR21_dqdot - R(2,1)*dR11_dqdot)/sqterm;

      Jtmp = [Jx;Jr];
      Jrow_ind = reshape([reshape(1:3*m,3,m);bsxfun(@times,3*m+(1:3)',ones(1,m))],[],1);
      Jdot = Jtmp(Jrow_ind,:);
    elseif rotation_type==2
      error('forwardJacDot: quaternions not yet supported');
    end
  end
  
end