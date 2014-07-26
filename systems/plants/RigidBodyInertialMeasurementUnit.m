classdef RigidBodyInertialMeasurementUnit < RigidBodySensor
  % outputs angular orientation, angular velocity, and linear acceleration

  methods
    function obj = RigidBodyInertialMeasurementUnit(manip,frame_id)
      typecheck(frame_id,'double');  % must be a body index
      obj.frame_id = frame_id;
    end

    function y = output(obj,manip,t,x,u)
      nq = getNumPositions(manip);
      q = x(1:nq);
      qd = x(nq+1:end);
      qdd = sodynamics(manip,t,q,qd,u);  % todo: this could be much more efficient if I cached qdd
      
      kinsol = doKinematics(manip,q,false,true,qd);
      
      is_planar = isa(manip,'PlanarRigidBodyManipulator');
      if is_planar, zero_vec = zeros(2,1); 
      else zero_vec = zeros(3,1); end
      
      [x,J] = forwardKin(manip,kinsol,obj.frame_id,zero_vec,1);
      Jdot = forwardJacDot(manip,kinsol,obj.frame_id,zero_vec,0,0);
      
      % x = f(q)
      % xdot = dfdq*dqdt = J*qd
      % xddot = dJdq*qd + J*qdd = Jdot*qd + J*qdd
      
      % note: x,J above have angles, but Jdot does not
      if is_planar
        angle = x(3);
        omega_body = J(3,:)*qd;
        
        accel_base = Jdot*qd + J(1:2,:)*qdd;
        R_base_to_body = rotmat(-angle);
        accel_body = R_base_to_body * accel_base;
        
        y = [ angle; ...
          omega_body; ...
          accel_body ];
      else  % 3D version
        quat_body_to_world = rpy2quat(x(4:6));
        quat_world_to_body = quatConjugate(quat_body_to_world);
        
        rpy = x(4:6);
        rpydot = J(4:6,:)*qd;
        omega_base = rpydot2angularvel(rpy, rpydot); % TODO: replace with computation based on kinsol.twists
        omega_body = quatRotateVec(quat_world_to_body, omega_base);
        
        accel_base = Jdot*qd + J(1:3,:)*qdd; % TODO: possibly replace with computation based on spatial accelerations
        accel_body = quatRotateVec(quat_world_to_body, accel_base);
        
        y = [ quat_body_to_world; ...
          omega_body; ...
          accel_body ];
      end
    end
    
    function fr = constructFrame(obj,manip)
      sensor_frame = getFrame(manip,obj.frame_id);
      body = getBody(manip,sensor_frame.body_ind);
      if isa(manip,'PlanarRigidBodyManipulator')
        fr = CoordinateFrame([strtok(body.linkname,'+'),'IMU'],4,'y', ...
          {'q', ...     % absolute orientation
          'w', ...      % angular rate
          'ax','az'});  % linear acceleration
      else
        fr = CoordinateFrame([strtok(body.linkname,'+'),'IMU'],10,'y', ...
          {'qw','qx','qy','qz', ...  % quaternion orientation
          'wx','wy','wz', ...       % angular velocity vector (omega)
          'ax','ay','az'});         % linear acceleration
      end
    end
    
    function tf = isDirectFeedthrough(obj)
      tf=true;
    end

    function obj = updateBodyIndices(obj,map_from_old_to_new)
      obj.body = map_from_old_to_new(obj.body);
    end
    
  end
  
  methods (Static)
    function obj = parseURDFNode(model,robotnum,node,body_ind,options)
      xyz = zeros(3,1); rpy = zeros(3,1);
      origin = node.getElementsByTagName('pose').item(0);
      if ~isempty(origin)
        if origin.hasAttribute('xyz')
          xyz = reshape(parseParamString(model,robotnum,char(origin.getAttribute('xyz'))),3,1);
        end
        if origin.hasAttribute('rpy')
          rpy = reshape(parseParamString(model,robotnum,char(origin.getAttribute('rpy'))),3,1);
        end
      end
      obj = RigidBodyInertialMeasurementUnit(model,body_ind,xyz,rpy);
    end    
  end
  
  properties
    frame_id
  end
  
end