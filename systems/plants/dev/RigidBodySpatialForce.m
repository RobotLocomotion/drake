classdef RigidBodySpatialForce < RigidBodyForceElement
  
  properties
    name
    frame_id
  end
  
  methods
    function obj = RigidBodySpatialForce(name,frame_id)
      obj.name = name;
      obj.frame_id = frame_id;
      obj.direct_feedthrough_flag = true;
    end
    
    % f_ext is a (potentially sparse) matrix with manip.getNumBodies columns
    % B is (nq x nu) matrix which contributes a control-affine term 
    %      + B(q,qd)*u 
    % to the manipulator dynamics
    function [f_ext,B] = computeSpatialForce(obj,manip,q,qd)
      f_ext = [];
      B = manip.B*0*q(1); %initialize B_mod
      
      options.use_mex = false;  % because I'm using kinsol.J below (for now)
      kinsol = doKinematics(manip,q,qd,options);
      
      options.rotation_type = 2; % quaternion
      [pose_in_world_frame] = forwardKin(manip,kinsol,obj.frame_id,zeros(3,1),options);

      % convert body frame to world frame
      T_body_to_world = [ quat2rotmat(pose_in_world_frame(4:7)), pose_in_world_frame(1:3); 0 0 0 1];
      AdT_world_to_body = transformAdjoint(homogTransInv(T_body_to_world));

      % now effectively reproduce the computation of C in
      % manipulatorDynamics
      B_joint_wrench = AdT_world_to_body';
      for body_index = findKinematicPath(manip,obj.frame_id,0)
        body = getBody(manip,body_index);
        Ji = kinsol.J{body_index};
        tau = Ji' * B_joint_wrench;
        B(body.velocity_num,obj.input_num) = tau;
      end
      
      % todo: compare this to RigidBodyThrust, which is comparitively much
      % simpler.  Do I have a good way of computing the correct jacobian in
      % one shot?
    end
    
    function fr = constructFrame(obj,manip)
      fr = CoordinateFrame(obj.name,6,'f',{'x','y','z','wx','wy','wz'});
    end
  end

end