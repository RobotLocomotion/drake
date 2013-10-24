classdef ContactForceTorqueSensor < TimeSteppingRigidBodySensor & Visualizer
  
  properties
    frame;
    body;
    normal_ind=[];
    tangent_ind=[];
    jsign=1;
    T % change from body coordinates to coordinates of the sensor
    xyz;
    manip;  % warning: could get stale, but keeping it around for the draw method
  end
  
  methods
    function obj = ContactForceTorqueSensor(tsmanip,body,xyz,rpy)
      typecheck(body,'double');  % must be a body index
      
      if tsmanip.twoD
        if (nargin<3) xyz=zeros(2,1);
        else sizecheck(xyz,[2,1]); end
        if (nargin<4) rpy=0;
        else sizecheck(rpy,1); end
        T = inv([rotmat(rpy),xyz; 0,0,1]);
        fr = CoordinateFrame('DefaultForceTorqueFrame',getNumStates(tsmanip)+3,'f');
      else
        if (nargin<3) xyz=zeros(3,1);
        else sizecheck(xyz,[3,1]); end
        if (nargin<4) rpy=zeros(3,1);
        else sizecheck(rpy,[3,1]); end
        T = inv([rotz(rpy(3))*roty(rpy(2))*rotx(rpy(1)),xyz; 0,0,0,1]);
        fr = CoordinateFrame('DefaultForceTorqueFrame',getNumStates(tsmanip)+6,'f');
      end      

      obj = obj@Visualizer(fr);
      obj.body = body;
      obj.T = T;
      obj.xyz = xyz;
    end
    
    function tf = isDirectFeedthrough(obj)
      tf = true;
    end
    
    function obj = compile(obj,tsmanip,manip)
      if (tsmanip.position_control) error('need to update this method for this case'); end

      body = getBody(manip,obj.body);
      
      if isempty(body.contact_pts)
        error('Drake:ContactForceTorqueSensor:NoContactPts','There are no contact points associated with body %s',body.linkname);
      end

      if tsmanip.twoD
        coords{1}=['force_',manip.x_axis_label];
        coords{2}=['force_',manip.y_axis_label];
        coords{3}='torque';
      else
        coords{1}='force_x';
        coords{2}='force_y';
        coords{3}='force_z';
        coords{4}='torque_x';
        coords{5}='torque_y';
        coords{6}='torque_z';
      end
      if isempty(obj.frame) || obj.frame.dim ~= length(coords)
        obj.frame = CoordinateFrame([body.linkname,'ForceTorqueSensor'],length(coords),'f',coords);
      end
      obj = setInputFrame(obj,MultiCoordinateFrame({getStateFrame(tsmanip),obj.frame}));
      
      nL = sum([manip.joint_limit_min~=-inf;manip.joint_limit_max~=inf]); % number of joint limits
      nC = manip.num_contacts;
      nP = 2*manip.num_position_constraints;  % number of position constraints
      nV = manip.num_velocity_constraints;  

      num_body_contacts = size(body.contact_pts,2);
      contact_ind_offset = size([manip.body(1:obj.body-1).contact_pts],2);
      
      % z(nL+nP+(1:nC)) = cN
      obj.normal_ind = nL+nP+contact_ind_offset+(1:num_body_contacts);
      
      mC = 2*length(manip.surfaceTangents([1;zeros(manip.dim-1,1)])); % get number of tangent vectors

      % z(nL+nP+nC+(1:mC*nC)) = [beta_1;...;beta_mC]
      for i=1:mC
        obj.tangent_ind{i} = nL+nP+(mC*nC)+contact_ind_offset+(1:num_body_contacts);
      end
      
      if isa(manip,'PlanarRigidBodyManipulator')
        obj.jsign=sign(dot(manip.view_axis,[0;-1;0]));
      end
      obj.manip = manip;
    end
    
    function draw(obj,t,xft)
      if length(obj.xyz)~=2
        error('only implemented for the planar case so far');
      end
      
      xft = splitCoordinates(getInputFrame(obj),xft);
      x = xft{1}; ft = xft{2};

      kinsol = doKinematics(obj.manip,x(1:obj.manip.getNumDOF),false,false);
      
      body_pts = kinsol.T{obj.body}\[obj.xyz, obj.xyz+.001*ft(1:2); 1 1];  % convert force from sensor coords to body coords
      body_pts = body_pts(1:2,:);
      
      world_pts = forwardKin(obj.manip,kinsol,obj.body,body_pts);
      
      figure(1); clf; xlim([-2 2]); ylim([-1 3]); axis equal;
      axisAnnotation('arrow',world_pts(1,:),world_pts(2,:),'Color','r','LineWidth',2);
    end
    
    function y = output(obj,tsmanip,manip,t,x,u)
      z = tsmanip.solveLCP(t,x,u)/tsmanip.timestep;
      
      % todo: could do this more efficiently by only computing everything
      % below for indices where the normal forces are non-zero

      body = manip.body(obj.body);
      
      % todo: re-enable mex here when i implement the planar mex version of
      % bodykin
      kinsol = doKinematics(manip,x(1:manip.getNumDOF),false,~isa(manip,'PlanarRigidBodyManipulator'));
      contact_pos = forwardKin(manip,kinsol,obj.body,body.contact_pts);
      
      [d,N] = size(contact_pos);
      [pos,~,normal] = collisionDetect(manip,contact_pos);

      % flip to body coordinates
      pos = sensorKin(obj,manip,kinsol,pos);
      sensor_pos = forwardKin(manip,kinsol,obj.body,obj.xyz);
      normal = sensorKin(obj,manip,kinsol,repmat(sensor_pos,1,N)+normal);
      tangent = manip.surfaceTangents(normal);

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
    
    function fr = constructFrame(obj,manip)
      fr = obj.frame;
    end
    
    function obj = updateBodyIndices(obj,map_from_old_to_new)
      obj.body = map_from_old_to_new(obj.body);
    end

  end
    
  methods (Access=private)
    function pts = sensorKin(obj,manip,kinsol,pts)
      % convert from global frame to sensor frame
      N = size(pts,2);
      pts = obj.T*[bodyKin(manip,kinsol,obj.body,pts);ones(1,N)];
      pts(end,:)=[];
    end
  end
  
end