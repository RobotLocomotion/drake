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
      x = obj.manip.getInitialState();
      q = x(1:obj.manip.getNumDOF());
      gc = obj.manip.contactPositions(q);
      cm = obj.manip.getCOM(q);
      
      % compute desired COM projection
      % assumes minimal contact model for now
      k = convhull(gc(1:2,:)');
      com_des = [mean(gc(1:2,k),2);cm(3)];
    end        
    
    function com_des = update(obj,t,com_des,x)
      nq = getNumDOF(obj.manip);
      q = x(1:nq);

      gc = obj.manip.contactPositions(q);
      
      % assumes minimal contact model for now
      ts = 3;
      idx = 1:8;
      if t>3*ts
        idx = 1:8;
      elseif t>2*ts
        idx = 1:4;
      elseif t>ts
        idx = 5:8;
      end
      
      gc = gc(:,idx);
      % compute desired COM projection
      k = convhull(gc(1:2,:)');
      com_des(1:2) = mean(gc(1:2,k),2);
    end
    
    function y = output(obj,t,com_des,x)
      y = com_des;
    end
  end
  
  properties
    manip
  end
end