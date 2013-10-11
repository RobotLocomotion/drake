classdef RigidBodyJointSensor < RigidBodySensor
  % outputs a (vector of) joint positions, given a mask specifying 
  % which DOF to include
  
  methods
    function obj = RigidBodyJointSensor(manip,dof_mask,robotnum)
      typecheck(manip,'RigidBodyManipulator');
      if (length(manip.name)>1) error('still need to handle the multi-robot case. should be easy enough'); end
      sizecheck(dof_mask,[getNumDOF(manip) 1]);
      if (nargin<3) robotnum = 1; end
      
      obj.robot_num = robotnum;
      obj.dof_mask = logical([dof_mask;0*dof_mask]);
    end
    
    function y = output(obj,manip,t,x,u)
      y=x(obj.dof_mask);
    end
    function fr = constructFrame(obj,manip)
      state_frame = manip.getStateFrame;
      c = getCoordinateNames(state_frame);
      fr = CoordinateFrame(['manip.name{obj.robot_num}','JointPosition'],sum(obj.dof_mask),'p',c(obj.dof_mask));
    end
    function tf = isDirectFeedthrough(obj)
      tf=false;
    end
  end
  
  properties
    robot_num;
    dof_mask;
  end
end