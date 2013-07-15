classdef SimpleEEGoalGenerator < DrakeSystem
  % simple system for generating end effector goals
   
  methods
    function obj = SimpleEEGoalGenerator(r,robot_name,body_id,channel_name)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      typecheck(robot_name,'char');
      typecheck(body_id,'char');

      eeframe = CoordinateFrame(strcat(body_id,'_end_effector_goal'),7,'x');
            ee_names = cell(4,1);
      ee_names{1} = strcat(body_id,'_active');
      ee_names{2} = strcat(body_id,'_x');
      ee_names{3} = strcat(body_id,'_y');
      ee_names{4} = strcat(body_id,'_z');
      ee_names{5} = strcat(body_id,'_roll');
      ee_names{6} = strcat(body_id,'_pitch');
      ee_names{7} = strcat(body_id,'_yaw');
      eeframe.setCoordinateNames(ee_names);

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
      ee_des = zeros(7,1);
    end
    
    function obj = setGoal(obj,x)
      obj.ee_goal = x;
    end
        
    function x_dn = update(obj,t,ee_des,x)
      x_dn = [obj.active;obj.ee_goal;0;0;0];
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