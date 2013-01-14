classdef ContactForceTorqueSensor < RigidBodySensor
  
  properties
    frame;
    tsmanip
    body
    normal_ind=[];
    tangent_ind=[];
  end
  
  methods
    function obj = ContactForceTorqueSensor(tsmanip,body,xyz,rpy)
      typecheck(tsmanip,'TimeSteppingRigidBodyManipulator');
      typecheck(body,'RigidBody');
      
      if isempty(body.contact_pts)
        error('Drake:ContactForceTorqueSensor:NoContactPts','There are no contact points associated with body %s',body.name);
      end

      if isa(body,'PlanarRigidBody')
        typecheck(tsmanip.manip,'PlanarRigidBodyManipulator'); 
        coords{1}=['force_',tsmanip.manip.x_axis_label];
        coords{2}=['force_',tsmanip.manip.y_axis_label];
        coords{3}='torque';
      else
        coords{1}='force_x';
        coords{2}='force_y';
        coords{3}='force_z';
        coords{4}='torque_x';
        coords{5}='torque_y';
        coords{6}='torque_z';
      end
      obj.frame = CoordinateFrame([body.linkname,'ForceTorqueSensor'],length(coords),'f',coords);
      obj.tsmanip = tsmanip;
      obj.body = body;
    end
    
    function obj = compile(obj)
      nL = sum([obj.tsmanip.manip.joint_limit_min~=-inf;obj.tsmanip.manip.joint_limit_max~=inf]); % number of joint limits
      nC = obj.tsmanip.manip.num_contacts;
      nP = 2*obj.tsmanip.manip.num_position_constraints;  % number of position constraints
      nV = obj.tsmanip.manip.num_velocity_constraints;  
      
      % z(nL+nP+(1:nC)) = cN
      n_ind = [];
      
      % z(nL+nP+nC+(1:mC*nC)) = [beta_1;...;beta_mC]
      D_ind = [];
    end
    
    function y = output(obj,t,x,u)
      m = obj.tsmanip.manip;
      z = obj.tsmanip.solveLCP(t,x,u);
      
      normal_forces = z(:,obj.normal_ind);
      % todo: could do this more efficiently by only computing everything
      % below for indices where the normal forces are non-zero

      tangent_forces = z(:,obj.tangent_ind);

      kinsol = doKinematics(m,x(1:m.num_q));
      contact_pos = forwardKin(obj,kinsol,obj.body,body.contact_pts);
      
      [pos,vel,normal] = collisionDetect(obj,contact_pos);
      tangent = m.surfaceTangents(normal);
      
      
    end
    function fr = getFrame(obj)
      fr = obj.frame;
    end
  end
  
end