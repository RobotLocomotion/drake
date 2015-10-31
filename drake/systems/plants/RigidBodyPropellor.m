classdef RigidBodyPropellor < RigidBodyForceElement

  properties
    kinframe
    axis
    scale_factor_thrust = 1; % scale factor between input and thrust
    scale_factor_moment = 1; % scale factor between input and moment due to aerodynamic drag
  end
  
  methods
    function obj = RigidBodyPropellor(frame_id, axis, scale_factor_thrust, scale_factor_moment, limits)
      
      obj.kinframe = frame_id;
      obj.axis = axis/norm(axis);      
      
      obj.direct_feedthrough_flag = true;
      obj.input_num = 0;
      obj.input_limits = [-inf inf];
      if (nargin > 2)
        obj.scale_factor_thrust = scale_factor_thrust;
      end
      if (nargin > 3)
        obj.scale_factor_moment = scale_factor_moment;
      end
      if (nargin > 4)
        obj.input_limits = limits;
      end
    end %constructor
    
    function [force, B_mod, dforce, dB_mod] = computeSpatialForce(obj,manip,q,qd)
      %B_mod maps the input to generalized forces.
      
      force = sparse(6,getNumBodies(manip))*q(1);
      B_mod = manip.B*0*q(1); %initialize B_mod

      options.in_terms_of_qdot = false;
      if (nargout>2)  % then compute gradients
        % thrust
        kinsol = doKinematics(manip,q,true);
        [x,J,dJ] = forwardKin(manip,kinsol,obj.kinframe,zeros(3,1),options);
        [axis_world,Jaxis_world] = forwardKin(manip,kinsol,obj.kinframe,obj.axis);
        daxis_world = Jaxis_world-J;
        axis_world = axis_world-x;

        dforce = sparse(6*getNumBodies(manip),getNumStates(manip));

        nq = getNumPositions(manip); nu = getNumInputs(manip);
        dB_mod = sparse(nq*nu,getNumStates(manip));
        dB_mod((obj.input_num-1)*nq + (1:nq),1:nq) = obj.scale_factor_thrust*(J'*daxis_world + reshape(dJ'*axis_world,nq,nq));
      
        % aerodynamic drag moment
        N = null(obj.axis');
        n1 = N(:,1);
        n2 = N(:,2);
        [x1,J1,dJ1] = forwardKin(manip,kinsol,obj.kinframe,n1,options);
        [axis_world_m1,Jaxis_world_m1] = forwardKin(manip,kinsol,obj.kinframe,n2+n1);
        daxis_world_m1 = Jaxis_world_m1 - J1;
        axis_world_m1 = axis_world_m1 - x1;
        
        [x2,J2,dJ2] = forwardKin(manip,kinsol,obj.kinframe,-n1,options);
        [axis_world_m2,Jaxis_world_m2] = forwardKin(manip,kinsol,obj.kinframe,-n2-n1);
        daxis_world_m2 = Jaxis_world_m2 - J2;
        axis_world_m2 = axis_world_m2 - x2;  
      
        dB_mod_m1 = sparse(nq*nu,getNumStates(manip));
        dB_mod_m1((obj.input_num-1)*nq + (1:nq),1:nq) = 0.5*obj.scale_factor_moment*(J1'*daxis_world_m1 + reshape(dJ1'*axis_world_m1,nq,nq));

        dB_mod_m2 = sparse(nq*nu,getNumStates(manip));
        dB_mod_m2((obj.input_num-1)*nq + (1:nq),1:nq) = 0.5*obj.scale_factor_moment*(J2'*daxis_world_m2 + reshape(dJ2'*axis_world_m2,nq,nq));

        dB_mod = dB_mod + dB_mod_m1 + dB_mod_m2;
        
        
      else
        % thrust
        kinsol = doKinematics(manip,q);
        [x,J] = forwardKin(manip,kinsol,obj.kinframe,zeros(3,1),options);
        axis_world = forwardKin(manip,kinsol,obj.kinframe,obj.axis);
        axis_world = axis_world-x;
        
        % aerodynamic drag moment
        N = null(obj.axis');
        n1 = N(:,1);
        n2 = N(:,2);
        [x1,J1] = forwardKin(manip,kinsol,obj.kinframe,n1,options);
        axis_world_m1 = forwardKin(manip,kinsol,obj.kinframe,n2+n1);
        axis_world_m1 = axis_world_m1 - x1;
        
        [x2,J2] = forwardKin(manip,kinsol,obj.kinframe,-n1,options);
        axis_world_m2 = forwardKin(manip,kinsol,obj.kinframe,-n2-n1);
        axis_world_m2 = axis_world_m2 - x2;        
                
      end
      
      % apply force along the z-axis of the reference frame
      B_mod(:,obj.input_num) = obj.scale_factor_thrust*J'*axis_world;
      
      % apply a couple corresponding to the moment due to aerodynamic drag
      B_mod(:,obj.input_num) = B_mod(:,obj.input_num) + 0.5*obj.scale_factor_moment*J1'*axis_world_m1;
      B_mod(:,obj.input_num) = B_mod(:,obj.input_num) + 0.5*obj.scale_factor_moment*J2'*axis_world_m2;

    end
    
    function varargout = addPropellorVisualShapeToBody(varargin)
      errorDeprecatedFunction('addPropellorVisualGeometryToBody');
    end
    
    function manip = addPropellorVisualGeometryToBody(obj, manip, body, diameter)
      % Adds a visual geometry of the propellor to the model on the body given for
      % drawing the in a visualizer.
      %
      % @param model manipulator the wing is part of
      % @param body body to add the visual geometry to
      % @param diameter diameter of the propellor to draw
      % @param axis the propellor produces thrust on
      %
      % @retval model updated model

      prop_width = 0.05;
      
      % repect the axis the propellor produces thrust on
      
      % a cyclindar always starts facing up so rotate it so that it is
      % facing in the direction of the thrust axis
      
      origin = [0; 0; 0];
      default_axis = [0; 0; 1];
      
      if default_axis == obj.axis
        rotmat = eye(3);
      else
        
        a = default_axis;
        b = obj.axis;
        
        % adapted from http://math.stackexchange.com/a/897677
        % first, generate the basis in which to rotate
        % with column vectors: [ normalized projection of b onto a, 
        % normalized vector rejection of b onto a, and cross(b,a)
        
        u = a;
        
        v = (b - dot(a,b)*a) / norm(b - dot(a,b)*a);
        
        w = cross(b,a);
        
        Fi = [ u v w ];
        
        
        
        % build the matrix to rotate in that basis
        
        G = [ dot(a,b)   -norm(cross(a,b))   0; ...
            norm(cross(a,b)) dot(a,b)         0; ...
                0              0               1];

        
        rotmat = Fi * G * inv(Fi); 

      end

      pts = [origin; 1];

      T = manip.getFrame(obj.kinframe).T;
      R = T(1:3,1:3);

      
      xyz_rpy = [T(1:3,:)*pts; repmat(rotmat2rpy(R*rotmat),1, 1)];
      
      xyz = xyz_rpy(1:3);
      
      
      rpy = xyz_rpy(4:6);
      
      
      geometry = RigidBodyCylinder(diameter/2, prop_width, xyz, rpy);
      geometry = geometry.setColor([ .5 .5 .5 ]);
      manip = manip.addVisualGeometryToBody(body, geometry);

    end
    
  end
  
  methods (Static)
    function [model,obj] = parseURDFNode(model,name,robotnum,node,options)
      elnode = node.getElementsByTagName('parent').item(0);
      parent = findLinkId(model,char(elnode.getAttribute('link')),robotnum);
      
      xyz = zeros(3,1); rpy = zeros(3,1);
      elnode = node.getElementsByTagName('origin').item(0);
      if ~isempty(elnode)
        if elnode.hasAttribute('xyz')
          xyz = reshape(parseParamString(model,robotnum,char(elnode.getAttribute('xyz'))),3,1);
        end
        if elnode.hasAttribute('rpy')
          rpy = reshape(parseParamString(model,robotnum,char(elnode.getAttribute('rpy'))),3,1);
        end
      end
      fr = RigidBodyFrame(parent,xyz,rpy,[name,'_frame']);
      [model,frame_id] = addFrame(model,fr);
      
      scaleFacThrust = 1;
      scaleFacMoment = 1;
      if node.hasAttribute('scale_factor_thrust')
        scaleFacThrust = parseParamString(model,robotnum,char(node.getAttribute('scale_factor_thrust')));
      end
      
      if node.hasAttribute('scale_factor_moment')
        scaleFacMoment = parseParamString(model,robotnum,char(node.getAttribute('scale_factor_moment')));
      end
      
      prop_axis = [1; 0; 0];
      elnode = node.getElementsByTagName('axis').item(0);
      if ~isempty(elnode)
        prop_axis = reshape(parseParamString(model,robotnum,char(elnode.getAttribute('xyz'))),3,1);
        prop_axis = prop_axis/(norm(prop_axis)+eps); % normalize
      end
        
      
      limits = [-inf,inf];
      if node.hasAttribute('lower_limit')
        limits(1) = parseParamString(model,robotnum,char(node.getAttribute('lower_limit')));
      end
      if node.hasAttribute('upper_limit')
        limits(2) = parseParamString(model,robotnum,char(node.getAttribute('upper_limit')));
      end
      
      obj = RigidBodyPropellor(frame_id, prop_axis, scaleFacThrust, scaleFacMoment, limits); 
      obj.name = name;
      
      visual_geometry = parseParamString(model,robotnum,char(node.getAttribute('visual_geometry')));
      visual_diameter = parseParamString(model,robotnum,char(node.getAttribute('visual_diameter')));
      
      if isempty(visual_geometry)
        visual_geometry = 1;
      end
      
      if isempty(visual_diameter)
        visual_diameter = 0.2;
      end
      
      if visual_geometry
        model = obj.addPropellorVisualGeometryToBody(model, parent, visual_diameter);
      end
      
      
    end
  end
  
end
