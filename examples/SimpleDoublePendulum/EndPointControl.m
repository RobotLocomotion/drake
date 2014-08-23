classdef EndPointControl < DrakeSystem
% A simple controller that takes in the DesiredHandPosition and the current
% arm state, and outputs a joint position command 
%
  methods
    function obj = EndPointControl(sys,manip)
      % @param sys the PD-controlled closed-loop system
      % @param manip the manipulator, so I can call kinematics, etc. 
      
      typecheck(sys,'DrakeSystem');
      typecheck(manip,'RigidBodyManipulator');
      
      obj = obj@DrakeSystem(0,sys.getNumInputs,sys.getNumStates,sys.getNumInputs,false,true);
      obj = setSampleTime(obj,[.01;0]); % update at 100 Hz
      obj = setInputFrame(obj,sys.getStateFrame);
      obj = setOutputFrame(obj,sys.getInputFrame);
      
      obj.manip = manip;
    end
    
    function q_d0 = getInitialState(obj)
      q_d0 = 0*randn(2,1);
    end
        
    function q_dn = update(obj,t,q_d,x)
      % Controller implementation.  
      q = x(1:2); qd = x(3:4);
      
      n = obj.manip.num_positions;
      dpos = [1;0];
      
      kinsol = doKinematics(obj.manip,q_d);  % open loop 
%      kinsol = doKinematics(obj.manip,q);  % closed loop 

      [pos,J] = forwardKin(obj.manip,kinsol,3,[0;-1]);

      err = dpos - pos;
      qd_d = 4*pinv(J)*err;
      normbound = 10; % norm bound to alleviate the effect of singularities.
      if (norm(qd_d)>normbound), qd_d = normbound*qd_d/norm(qd_d); end  
      
      dt = .01;  % should really call getSampleTime
      q_dn = q_d + dt*qd_d;

      for i=1:2
        scope('SimpleDoublePendulum',['q',num2str(i),''],t,q(i),struct('linespec','b','scope_id',i));
        scope('SimpleDoublePendulum',['q',num2str(i),'_d'],t,q_d(i),struct('linespec','r','scope_id',i));
      end        
    end
    
    function y = output(obj,t,q_d,x)
      y = q_d;
    end
  end
  
  properties
    manip
  end
end