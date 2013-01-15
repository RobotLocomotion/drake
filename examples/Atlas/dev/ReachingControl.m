classdef ReachingControl < MIMODrakeSystem
% A simple controller that takes in the DesiredHandPosition and the current
% arm state, and outputs a joint position command 
%
  methods
    function obj = ReachingControl(sys,manip)
      % @param sys the PD-controlled closed-loop system
      % @param manip the manipulator, so I can call kinematics, etc. 
      
      typecheck(sys,'DrakeSystem');
      typecheck(manip,'RigidBodyManipulator');

      input_frame = MultiCoordinateFrame({manip.constructCOMFrame,manip.getStateFrame});
      
      obj = obj@MIMODrakeSystem(0,manip.featherstone.NB,input_frame,sys.getInputFrame(),false,true);
      obj = setSampleTime(obj,[.01;0]); % update at 100 Hz
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,sys.getInputFrame);
      
      obj.manip = manip;
      obj.rhand_ind = find(~cellfun(@isempty,strfind({manip.body.linkname},'r_hand')));
      obj.lhand_ind = find(~cellfun(@isempty,strfind({manip.body.linkname},'l_hand')));
    end
    
    function q_d0 = getInitialState(obj)
      q_d0 = 0*randn(2,1);
    end
        
    function q_dn = mimoUpdate(obj,t,q_d,com_d,x)
      % Controller implementation.  
      q = x(1:2); qd = x(3:4);
      
      n = obj.manip.featherstone.NB;
      dpos = [1;0;0];
      
      kinsol = doKinematics(obj.manip,q_d);  % open loop 
%      kinsol = doKinematics(obj.manip,q);  % closed loop 

      [pos,J] = forwardKin(obj.manip,kinsol,obj.rhand_ind,[0;0;0]);

      err = dpos - pos;
      scope('Atlas','handpos_err',t,norm(err),struct('linespec','b','scope_id',1));
      qd_d = 4*pinv(J)*err;
      normbound = .05; % norm bound to alleviate the effect of singularities.
      if (norm(qd_d)>normbound), qd_d = normbound*qd_d/norm(qd_d); end  
      
      dt = .01;  % should really call getSampleTime
      q_dn = q_d + dt*qd_d;
    end
    
    function y = mimoOutput(obj,t,q_d,com_d,x)
      y = q_d;
    end
  end
  
  properties
    manip
    rhand_ind
    lhand_ind
  end
end