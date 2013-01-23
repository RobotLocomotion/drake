classdef SimpleAlternatingCOMGoalGenerator < DrakeSystem
  % example alternating foot COM goal generator
   
  methods
    function obj = SimpleAlternatingCOMGoalGenerator(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      
      comframe = AtlasCOM(r);

      obj = obj@DrakeSystem(0,comframe.dim,r.getNumStates,comframe.dim,false,true);
      obj = setSampleTime(obj,[.01;0]); % update at 100 Hz
      obj = setInputFrame(obj,r.getStateFrame);
      obj = setOutputFrame(obj,comframe);

      obj.manip = r;
    end
    
    function com_des = getInitialState(obj)
      com_des = zeros(3,1);
    end
        
    function com_des = update(obj,t,com_des,x)
      nq = getNumDOF(obj.manip);
      q = x(1:nq);

      gc = obj.manip.contactPositions(q);
      
      % assumes minimal contact model for now
      idx = 1:4;
      if t>18
        idx = 1:8;
      elseif t>12
        idx = 1:4;
      elseif t>6
        idx = 5:8;
      end
      
      gc = gc(:,idx);
      % compute desired COM projection
      k = convhull(gc(1:2,:)');
      com_des = [mean(gc(1:2,k),2);0];
    end
    
    function y = output(obj,t,com_des,x)
      y = com_des;
    end
  end
  
  properties
    manip
  end
end