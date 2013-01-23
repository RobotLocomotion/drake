classdef EndEffector 
  methods
    function obj = EndEffector(r,robot_name,body_id,xyz_offset,channel_name)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      typecheck(robot_name,'char');
      typecheck(body_id,'char');
      if nargin > 3
        typecheck(xyz_offset,'double');
      else
        xyz_offset = zeros(3,1);
      end
      
      obj.manip = r;
      obj.body_id = body_id;
      obj.body_index = find(~cellfun(@isempty,strfind(r.getLinkNames(),body_id)));
      obj.xyz_offset = xyz_offset;


      coder = EndEffectorGoalCoder(robot_name,body_id);
      obj.frame = LCMCoordinateFrameWCoder(strcat(body_id,'_end_effector_goal'),4,'x',JLCMCoder(coder));
      ee_names = cell(4,1);
      ee_names{1} = strcat(body_id,'_active');
      ee_names{2} = strcat(body_id,'_x');
      ee_names{3} = strcat(body_id,'_y');
      ee_names{4} = strcat(body_id,'_z');
      obj.frame.setCoordinateNames(ee_names);

      if nargin > 4
        typecheck(channel_name,'char');
        obj.frame.setDefaultChannel(channel_name);
      end
      
      obj.P_mask = eye(r.getNumStates()/2);
    end
    
    function [x,J] = doKin(obj,q)
      kinsol = doKinematics(obj.manip,q); 
      [x,J] = forwardKin(obj.manip,kinsol,obj.body_index,obj.xyz_offset);
    end
    
    function obj = setMask(obj,mask)
      sizecheck(mask,obj.manip.getNumStates()/2);
      obj.P_mask = diag(mask);
    end
    
    function obj = setGain(obj,gain)
      typecheck(gain,'double');
      sizecheck(gain,[1 1]);
      obj.gain = gain;
    end

    function obj = setBound(obj,normbound)
      typecheck(normbound,'double');
      sizecheck(normbound,[1 1]);
      obj.normbound = normbound;
    end

  end
  
  properties (SetAccess = protected, GetAccess = public)
    manip
    frame
    body_id
    body_index
    xyz_offset
    P_mask % used to select controllable subsets of joints
    gain = 2.0;
    normbound = 1.0;
  end
end