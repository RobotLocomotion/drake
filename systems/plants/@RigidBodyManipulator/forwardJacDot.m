function Jdot = forwardJacDot(obj,kinsol,body_or_frame_ind,pts,robotnum)

% same input as [x,J] = forwardKin but returns Jdot
% note: you must have called kinsol = doKinematics with qd passed in as the
% last argument
if nargin<5, robotnum=1; end
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
    Jdot = forwardKinmex(obj.mex_model_ptr,kinsol.q,body_or_frame_ind,pts,false,true);
  end
else
  if (body_or_frame_ind == 0) 
    nq=getNumDOF(obj);
    
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
    
    Jdot = reshape(kinsol.dTdqdot{body_ind}*Tframe*pts,obj.getNumDOF,[])';
  end
  
end