classdef SimpleEEGoalGenerator < DrakeSystem
  % simple system for generating end effector goals
   
  methods
    function obj = SimpleEEGoalGenerator(r,robot_name,body_id,channel_name)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      typecheck(robot_name,'char');
      typecheck(body_id,'char');

      eeframe = CoordinateFrame(strcat(body_id,'_end_effector_goal'),4,'x');
      
      if nargin > 3
        typecheck(channel_name,'char');
%        eeframe.setDefaultChannel(channel_name);
      end
            
      obj = obj@DrakeSystem(0,eeframe.dim,r.getNumStates,eeframe.dim,false,true);
      obj = setSampleTime(obj,[.01;0]); % update at 100 Hz
      obj = setInputFrame(obj,r.getStateFrame);
      obj = setOutputFrame(obj,eeframe);
    end
    
    function ee_des = getInitialState(obj)
      ee_des = zeros(4,1);
    end
    
    function obj = setGoal(obj,x)
      obj.ee_goal = x;
    end
        
    function x_dn = update(obj,t,ee_des,x)
      x_dn = [obj.active;obj.ee_goal];
    end
    
    function y = output(obj,t,ee_des,x)
      y = ee_des;
    end
  end
  
  properties
    ee_goal
    active = 1
  end
end