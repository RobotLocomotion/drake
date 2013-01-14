classdef ContactForceTorqueSensor < RigidBodySensor
  
  properties
    frame;
    tsmanip
    body
    normal_ind=[];
    tangent_ind=[];
    jsign=1;
    T % change from body coordinates to coordinates of the sensor
    xyz;
  end
  
  methods
    function obj = ContactForceTorqueSensor(tsmanip,body,xyz,rpy)
      typecheck(tsmanip,'TimeSteppingRigidBodyManipulator');
      typecheck(body,'RigidBody');
      
      if isempty(body.contact_pts)
        error('Drake:ContactForceTorqueSensor:NoContactPts','There are no contact points associated with body %s',body.name);
      end

      if isa(tsmanip.manip,'PlanarRigidBodyManipulator')
        if (nargin<3) xyz=zeros(2,1);
        else sizecheck(xyz,[2,1]); end
        if (nargin<4) rpy=0;
        else sizecheck(rpy,1); end
        T = inv([rotmat(rpy),xyz; 0,0,1]);
        
        coords{1}=['force_',tsmanip.manip.x_axis_label];
        coords{2}=['force_',tsmanip.manip.y_axis_label];
        coords{3}='torque';
      else
        if (nargin<3) xyz=zeros(3,1);
        else sizecheck(xyz,[3,1]); end
        if (nargin<4) rpy=zeros(3,1);
        else sizecheck(rpy,[3,1]); end
        T = inv([rotz(rpy(3))*roty(rpy(2))*rotx(rpy(1)),xyz; 0,0,0,1]);

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
      obj.T = T;
      obj.xyz = xyz;
      
    end
    
    function obj = compile(obj)
      m = obj.tsmanip.manip;
      nL = sum([m.joint_limit_min~=-inf;m.joint_limit_max~=inf]); % number of joint limits
      nC = m.num_contacts;
      nP = 2*m.num_position_constraints;  % number of position constraints
      nV = m.num_velocity_constraints;  

      body_ind = find(m.body==obj.body,1);
      num_body_contacts = size(obj.body.contact_pts,2);
      contact_ind_offset = size([m.body(1:body_ind-1).contact_pts],2);
      
      % z(nL+nP+(1:nC)) = cN
      obj.normal_ind = nL+nP+contact_ind_offset+(1:num_body_contacts);
      
      mC = 2*length(m.surfaceTangents(m.gravity)); % get number of tangent vectors

      % z(nL+nP+nC+(1:mC*nC)) = [beta_1;...;beta_mC]
      for i=1:mC
        obj.tangent_ind{i} = nL+nP+(mC*nC)+contact_ind_offset+(1:num_body_contacts);
      end
      
      if isa(m,'PlanarRigidBodyManipulator')
        obj.jsign=sign(dot(m.view_axis,[0;-1;0]));
      end
    end
    
    function y = output(obj,t,x,u)
      m = obj.tsmanip.manip;
      z = obj.tsmanip.solveLCP(t,x,u)/obj.tsmanip.timestep;
      
      % todo: could do this more efficiently by only computing everything
      % below for indices where the normal forces are non-zero

      % todo: enable mex here (by implementing the mex version of bodyKin)
      use_mex = false;
      kinsol = doKinematics(m,x(1:m.num_q),false,use_mex);
      contact_pos = forwardKin(m,kinsol,obj.body,obj.body.contact_pts);
      
      [d,N] = size(contact_pos);
      [pos,~,normal] = collisionDetect(m,contact_pos);

      % flip to body coordinates
      pos = sensorKin(obj,kinsol,pos);
      sensor_pos = forwardKin(m,kinsol,obj.body,obj.xyz);
      normal = sensorKin(obj,kinsol,repmat(sensor_pos,1,N)+normal);
      tangent = m.surfaceTangents(normal);

      % compute all individual contact forces in sensor coordinates
      force = repmat(z(obj.normal_ind)',d,1).*normal;
      mC=length(tangent);
      for i=1:mC
        force = force + repmat(z(obj.tangent_ind{i})',d,1).*tangent{i} ...
          - repmat(z(obj.tangent_ind{i+mC})',d,1).*tangent{i}; 
      end
      y = sum(force,2);

      if (d==2)
        torque = sum(cross([pos;zeros(1,N)],[force;zeros(1,N)]),2);
        y(3) = obj.jsign*torque(3);
      else
        y(4:6) = sum(cross(pos,force),2);
      end
    end
    
    function fr = getFrame(obj)
      fr = obj.frame;
    end
    
  end
    
  methods (Access=private)
    function pts = sensorKin(obj,kinsol,pts)
      % convert from global frame to sensor frame
      N = size(pts,2);
      pts = obj.T*[bodyKin(obj.tsmanip.manip,kinsol,obj.body,pts);ones(1,N)];
      pts(end,:)=[];
    end
  end
  
end