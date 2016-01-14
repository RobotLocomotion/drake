classdef RigidBodyInertialMeasurementUnit < RigidBodySensor
  % outputs angular orientation, angular velocity, and linear acceleration

  methods
    function obj = RigidBodyInertialMeasurementUnit(manip,body,xyz,rpy)
      typecheck(body,'double');  % must be a body index
      
      if isa(manip,'PlanarRigidBodyManipulator')
        error('need to update kinematics implementation for planar manipulators (bug 1728) before this will work'); 
        if (nargin<3) xyz=zeros(2,1);
        else sizecheck(xyz,[2,1]); end
        if (nargin<4) rpy=0;
        else sizecheck(rpy,1); end
      else
        if (nargin<3) xyz=zeros(3,1);
        else sizecheck(xyz,[3,1]); end
        if (nargin<4) rpy=zeros(3,1);
        else sizecheck(rpy,[3,1]); end
      end      

      if (any(rpy)) error('Drake:RigidBodyInertialMeasurementUnit:RPYNotImplemented', 'non-zero rpy not supported yet'); end  % but it wouldn't be too hard (see ContactForceTorque)
      
      obj.body = body;
      obj.xyz = xyz;
      
      warning('the IMU outputs have not been tested yet (due to bug 1728)'); 
    end  

    function y = output(obj,manip,t,x,u)
      nq = getNumPositions(manip);
      q = x(1:nq);
      qd = x(nq+1:end);
      xdd = dynamics(manip, t, x, u);  % todo: this could be much more efficient if I cached qdd
      qdd = xdd(nq+1:end);
      
      kinsol = doKinematics(manip,q,false,true,qd);
      
      [x,J] = forwardKin(manip,kinsol,obj.body,obj.xyz,1);
      Jdot_times_v = forwardJacDotTimesV(manip, kinsol, obj.body, obj.xyz, 0, 1);
      
      % x = f(q)
      % xdot = dfdq*dqdt = J*qd
      % xddot = dJdq*qd + J*qdd = Jdot*qd + J*qdd
      
      % note: x,J above have angles, but Jdot does not
      if numel(obj.xyz)==2  % 2D version
        angle = x(3);
        omega_body = J(3,:)*qd;
        
        accel_base = Jdot_times_v + J(1:2,:)*qdd;
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
        
        accel_base = Jdot_times_v + J(1:3,:)*qdd; % TODO: possibly replace with computation based on spatial accelerations
        accel_body = quatRotateVec(quat_world_to_body, accel_base);
        
        y = [ quat_body_to_world; ...
          omega_body; ...
          accel_body ];
      end
    end
    
    function fr = constructFrame(obj,manip)
      body = getBody(manip,obj.body);
      if numel(obj.xyz)==2
        fr = CoordinateFrame([strtok(body.linkname,'+'),'IMU'],4,'y', ...
          {'q', ...     % absolute orientation
          'w', ...      % angular rate
          'ax','az'});  % linear acceleration
      else
        fr = CoordinateFrame([strtok(body.linkname,'+'),'IMU'],10,'y', ...
          {'qx','qy','qz','qw', ...  % quaternion orientation
          'wx','wy','wz', ...       % angular rate (omega)
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
    body
    xyz
  end
  
end
