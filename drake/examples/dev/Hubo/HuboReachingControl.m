classdef HuboReachingControl < DrakeSystem
% A simple controller that takes in the DesiredHandPosition and the current
% HuboState, and outputs a JointPositionCommand 
%
% But for now, it just takes the HuboState and outputs a 
% JointPositionCommand that regulates the COM to the center of the support
% polygon (and minimizes the joint angles in the nullspace)
  
  methods
    function obj = HuboReachingControl()
      [sys,v,r] = HuboSaggitalPlant();  % todo: handle 3D plants here, too?
      obj = obj@DrakeSystem(0,sys.getNumInputs,sys.getNumStates,sys.getNumInputs,false,true);
      obj.model = r.manip.model;
      obj = setSampleTime(obj,[.01;0]); % update at 100 Hz
      obj = setInputFrame(obj,sys.getStateFrame);
      obj = setOutputFrame(obj,sys.getInputFrame);
      
      obj.rfoot_ind = find(~cellfun(@isempty,strfind({obj.model.body.linkname},'RFoot')));
      obj.lfoot_ind = find(~cellfun(@isempty,strfind({obj.model.body.linkname},'LFoot')));
      
      obj.rhand_ind = find(~cellfun(@isempty,strfind({obj.model.body.linkname},'RPalm')));
      obj.lhand_ind = find(~cellfun(@isempty,strfind({obj.model.body.linkname},'LPalm')));
    end
    
    function q_d = getInitialState(obj)
      q_d = [zeros(7,1); repmat([-.32;.5;-.25],2,1)];
    end
    
    function qa_dn = update(obj,t,qa_d,x)
      % Controller implementation.  
      % See derivation in HuboReachingControl.pdf
      
      n = obj.model.num_positions;
      q = x(1:n); qd=x(n+(1:n));
      
      eta = [.01;1;.01];  
      desired_height = .7;
      
%      doKinematics(obj.model,q);  % getCOM call below also runs kinematics
      [com,Jcom] = obj.model.getCOM([q(1:3);qa_d]);
      
      % assume for now that all feet contact points are on the ground
      % (could check this once force output is available)
      [rgc,Jrgc] = forwardKin(obj.model,obj.rfoot_ind,[obj.model.body(obj.rfoot_ind).contact_pts]);
      [lgc,Jlgc] = forwardKin(obj.model,obj.lfoot_ind,[obj.model.body(obj.lfoot_ind).contact_pts]);
      gc = [rgc,lgc];
      Jgc = [Jrgc;Jlgc];
      if (obj.model.D==3)
        np = 6;
        k = convhull(gc(1:2,:));
        csp = mean(gc(1:2,k));
        P = [eye(2),0];
%        Jcsp = [mean(Jgc(1+(k-1)*3,:));mean(Jgc(2+(k-1)*3,:))];
      else
        np = 3;
        [~,k(2)] = max(gc(1,:));
        [~,k(1)] = min(gc(1,:));
        csp = mean(gc(1,k)); 
        P = [1,0];
%        Jcsp = mean(Jgc(1+(k-1)*2,:));
%        figure(32);
%        line([csp,csp],[0,1],'Color','g','LineWidth',1.5);
      end      
      na = n-np;
      qa = q(np+1:end); qad = qd(np+1:end);
      
      A = [ -pinv(Jgc(:,1:np))*Jgc(:,np+1:end); eye(n-np) ];
      J1 = P*Jcom*A;
      iJ1 = pinv(J1);
      xd1 = eta(1)*(csp - P*com);
%      scope('Hubo','com_err',t,xd1,struct('linespec','r'));
%      scope('Hubo','com_dot',t,P*Jcom*qd,struct('linespec','b'));
%      scope('Hubo','gcdot',t,Jgc*qd,struct('scope_id',3));
      
      qa0 = [zeros(7,1); repmat([-.32;.5;-.25],2,1)];
      qad0 = eta(3)*(qa0-q(np+1:end));

      kp = diag([5,repmat(5,1,6),repmat(200,1,6)]);
      kd=diag([ones(7,1);5*ones(6,1)]);

      qad_d = zeros(13,1); %iJ1*xd1;% + (eye(na)-iJ1*J1)*qd0;
      qad_d(1) = .1*sin(2*pi*t);
%      scope('Hubo','com_dot_d',t,P*Jcom*A*qd_d,struct('linespec','g'));
      qa_dn = qa_d;% - kd*inv(kp)*qad_d; 
      qa_dn(1) = qa_dn(1)+.1*sin(2*pi*t);
%      scope('Hubo','q_dn',t,q_dn,struct('scope_id',2));
      
%      for i=4%1:13
%        scope('Hubo',['qa',num2str(i),'d'],t,A(i,:)*qad,struct('linespec','b','scope_id',i));
%        scope('Hubo',['qa',num2str(i),'d_d'],t,A(i,:)*qad_d,struct('linespec','r','scope_id',i));
%      end
      
      for i=[]%1
        scope('Hubo',['qa',num2str(i),''],t,qa(i),struct('linespec','b','scope_id',i));
        scope('Hubo',['qa',num2str(i),'_d'],t,qa_d(i),struct('linespec','r','scope_id',i));
      end        
    end
    
    function y = output(obj,t,q_d,x)
      y = q_d;
    end
  end
  
  properties
    model
    rfoot_ind
    lfoot_ind
    rhand_ind
    lhand_ind
  end
end