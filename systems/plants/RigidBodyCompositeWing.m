classdef RigidBodyCompositeWing < RigidBodyForceElement
  % RigidBodyCompositeWing is a container class for one or more
  % RigidBodyWing or RigidBodyWingWithControlSurface's.
  % This allows us to handle wings with control surfaces as multiple
  % smaller parts of a wing.
  %
  % When reading a URDF, we automatically create the relevant objects
  % based on if the wing has control surfaces or not.
  
  properties
    
    contained_wings = {}; % array of RigidBodyWing's that make up this wing
    
    contained_wing_left_edges; % sorted array of points along the wing denoting the left edge of each contained_wing. Index is the same as obj.contained_wings.
    
    control_surfaces; % array of control surfaces on this wing, sorted by order appearing in the URDF
    
    kinframe; % frame_id for the wing (note: there are different ones for the contained_wings based on their actual position)
    
  end
  

  methods
    function obj = RigidBodyCompositeWing(frame_id, profile, chord, span, stallAngle, velocity)
      % This constructor supports no arguments or the arguments above.
      % Given the parameters above, we create one RigidBodyWing with the
      % parameters described there.
      
      if nargin > 0
      
        this_contained_wing = RigidBodyWing(frame_id, profile, chord, span, stallAngle, velocity);
      
        obj.contained_wings{length(obj.contained_wings) + 1} = this_contained_wing;
        
        obj.contained_wing_left_edges = 0;
        
        obj.kinframe = frame_id;
        
      else
        % with no arguments, we were probably called by the URDF constructor
        
        % no op
        
      end
    end
    
    
    function [force, B_force_or_dforce, dforce ] = computeSpatialForce(obj,manip,q,qd)
      % Calls the appropriate RigidBodyWing.computeSpatialForce
      % for all of the contained wings, adds the results, and returns.
      %
      % @param manip RigidBodyManipulator we are a part of
      % @param q state vector
      % @param qd time derivative of state vector
      %
      % @retval force forces produced by the wing
      % @retval B_force_or_dforce if we have a control surface anywhere,
      %   this is the B matrix that maps control input to force.
      %   Otherwise, this is the gradient of the forces.
      %
      % @retval dforce gradient of forces if we do have a control surface
      %   anywhere

      
      if length(obj.contained_wings) < 1
        error('computeSpatialForce called with no contained wings.');
      end
      
      control_surface_flag = false;
      
      for i = 1 : length(obj.contained_wings)
        
        if (obj.contained_wings{i}.has_control_surface)
          
          if control_surface_flag
            error('multiple control surfaces on one wing not supported.');
          end
          
          control_surface_flag = true;
          
          [this_force, B_force, this_dforce, dB_force ] = obj.contained_wings{i}.computeSpatialForce(manip,q,qd);
          
          if i == 1
            force = this_force;
            dforce = this_dforce;
          else
            force = force + this_force;
            dforce = dforce + this_dforce;
          
          end
          
          
        else
        
          [this_force, this_dforce] = obj.contained_wings{i}.computeSpatialForce(manip, q, qd);
          if i == 1
            force = this_force;
            dforce = this_dforce;
          else
            force = force + this_force;
            dforce = dforce + this_dforce;
          
          end
        end
        
      end
      
      if control_surface_flag
        % we have a control surface
        B_force_or_dforce = B_force;
      else
        B_force_or_dforce = dforce;
      end
      
    end
        
    function obj = setInputNum(obj, input_num)
      % override RigidBodyForceElement.setInputNum to support contained_wings.
      
      obj.input_num = input_num;
      
      for i = 1 : length(obj.contained_wings)
        if obj.contained_wings{i}.has_control_surface
          obj.contained_wings{i} = obj.contained_wings{i}.setInputNum(input_num);
        end
      end
    end
    
    
    function drawWing(obj, manip, q, qd, fill_color)
      % Draws the wing onto the current figure in the state
      % given by q and qdot
      %
      % @param manip manipulator the wing is part of
      % @param q state vector
      % @param qd q-dot (state vector derivatives)
      % @param fill_color @default 1
      
      if nargin < 5
        fill_color = 1;
      end
      
      for i = 1 : length(obj.contained_wings)
        obj.contained_wings{i}.drawWing(manip, q, qd, fill_color);
      end
    end
    
    
    function [CL, CD, CM, dCL, dCD, dCM] = coeffs(obj, aoa)
      % Returns dimensionalized coefficient of lift, drag, and pitch
      % moment for a given angle of attack
      %
      % @param aoa angle of attack to get coefficients at
      %
      % @retval CL coefficient of lift
      % @retval CD coefficient of drag
      % @retval CM pitch moment
      %
      % @retval dCL derivative of coefficient of lift
      % @retval dCL derivative of coefficient of drag
      % @retval dCL derivative of pitch moment
      
      if length(obj.contained_wings) == 1
        
        if (nargout > 3)
          [CL, CD, CM, dCL, dCD, dCM] = obj.contained_wings{1}.coeffs(aoa);
        else
          [CL, CD, CM] = obj.contained_wings{1}.coeffs(aoa);
        end
        
      else
        error('Not supported for wings with more than one contained wing.');
      end
      
      
      
    end

  end

  methods (Static)
    function [model,obj] = parseURDFNode(model,robotnum,node,options)
      % Build a RigidBodyCompositeWing from a URDF.
      
      name = char(node.getAttribute('name'));
      name = regexprep(name, '\.', '_', 'preservecase');

      elNode = node.getElementsByTagName('parent').item(0);
      parent = findLinkInd(model,char(elNode.getAttribute('link')),robotnum);

      xyz=zeros(3,1); rpy=zeros(3,1);
      elnode = node.getElementsByTagName('origin').item(0);
      if ~isempty(elnode)
        if elnode.hasAttribute('xyz')
          xyz = reshape(parseParamString(model,robotnum,char(elnode.getAttribute('xyz'))),3,1);
        end
        if elnode.hasAttribute('rpy')
          rpy = reshape(parseParamString(model,robotnum,char(elnode.getAttribute('rpy'))),3,1);
        end
      end
      [model,this_frame_id] = addFrame(model,RigidBodyFrame(parent,xyz,rpy,[name,'_frame']));
      
      % search for control surfaces
      control_surface_nodes = node.getElementsByTagName('control_surface');
      
      profile = char(node.getAttribute('profile'));
      chord = parseParamString(model,robotnum,char(node.getAttribute('chord')));
      span = parseParamString(model,robotnum,char(node.getAttribute('span')));
      stall_angle = parseParamString(model,robotnum,char(node.getAttribute('stall_angle')));
      nominal_speed = parseParamString(model,robotnum,char(node.getAttribute('nominal_speed')));
      
      left_edge_of_wing = xyz - [0; span/2; 0];
      
      if isempty(control_surface_nodes.item(0))
        % no control surfaces, only a normal wing
        % create that object and return.
        
        obj = RigidBodyWing(this_frame_id, profile, chord, span, stall_angle, nominal_speed);
        obj.name = name;
        
        return;
        
      else
        % has control surfaces
        
        if ~strcmpi(profile, 'flat plate')
            error('wings with control surfaces that are not flat plates is unimplemneted at the current time.');
        end
        
        % load control surface data
        count = 0;
        
        control_surfaces_array = [];

        this_surface = control_surface_nodes.item(count);
        while ~isempty(this_surface)
        
          
          surface_name = char(this_surface.getAttribute('name'));
          surface_chord = parseParamString(model,robotnum,char(this_surface.getAttribute('chord')));
          surface_span = parseParamString(model,robotnum,char(this_surface.getAttribute('span')));
          surface_left_edge_position_along_wing = parseParamString(model,robotnum,char(this_surface.getAttribute('left_edge_position_along_wing')));
          surface_min_deflection = parseParamString(model,robotnum,char(this_surface.getAttribute('min_deflection')));
          surface_max_deflection = parseParamString(model,robotnum,char(this_surface.getAttribute('max_deflection')));
          
          assert(surface_span > 0, ['Span must be > 0 on control surface "' surface_name '"']);
          assert(surface_chord > 0, ['Chord must be > 0 on control surface "' surface_name '"']);
          assert(surface_left_edge_position_along_wing >= 0, ['Left edge position must be >= 0 on control surface "' surface_name '"']);
          
          assert(surface_span + surface_left_edge_position_along_wing <= span, ['Control surface runs off the right end of the wing on surface "' surface_name '"']);
          
          for other_surface = control_surfaces_array
            if strcmp(other_surface.name, surface_name)
              error(['Duplicate control surface name "' surface_name '"']);
            end
          end
          
          control_surfaces_array = [ control_surfaces_array ControlSurface(surface_name, surface_chord, surface_span, surface_left_edge_position_along_wing, surface_min_deflection, surface_max_deflection) ];

          
          count = count + 1;
          this_surface = control_surface_nodes.item(count);
          
        end
        
        % now we split the wing up into multiple segments that either
        % contain one of the control surfaces or don't
        
        % do some sanity checks to make sure that the control surfaces do
        % not overlap
        for surface = control_surfaces_array
          for surface2 = control_surfaces_array
            
            if ~strcmp(surface.name, surface2.name)
              
              left1 = surface.left_edge_position_along_wing;
              right1 = left1 + surface.span;
              
              left2 = surface2.left_edge_position_along_wing;
              
              if (left1 < left2 && right1 > left2)
                error(['Control surfaces "' surface.name '" and "' surface2.name '" overlap.']);
              end
              
            end
          end
        end % end sanity checks
        
        
        % check for the case where the entire wing has a control surface
        if length(control_surfaces_array) == 1 && control_surfaces_array(1).left_edge_position_along_wing == 0 ...
            && control_surfaces_array(1).span == span
          
          % this wing has only one control surface and it spans the entire
          % wing, so we can just return a single
          % RigidBodyWingWithControlSurface object
          
          
          obj = RigidBodyWingWithControlSurface(this_frame_id, profile, chord, span, stall_angle, nominal_speed, control_surfaces_array(1));
          obj.name = name;
          
          return;
          
        end
        
        

        % if we are here,  that means we have a more complex wing that
        % consists of componet parts.  Thus, we create a
        % RigidBodyCompositeWing
        
        obj = RigidBodyCompositeWing();
      
        obj.kinframe = this_frame_id;
        obj.control_surfaces = control_surfaces_array;
        
        % split the wing up starting from the left edge
        
        remaining_surfaces = obj.control_surfaces;
        point_along_wing = 0;
        
        while (span - point_along_wing > 1e-6) % not always exactly less than
        
          % find the minimum value of the control surface left edges
          min_left_edge_value = inf;
          min_left_edge_index = -1;
          
          for i = 1 : length(remaining_surfaces)
            % need a loop heree because min() doesn't support two matricies
            % with two output arguments
            if (remaining_surfaces(i).left_edge_position_along_wing < min_left_edge_value)
              min_left_edge_value = remaining_surfaces(i).left_edge_position_along_wing;
              min_left_edge_index = i;
            end
          end

          if (min_left_edge_value == point_along_wing)
            % control surface starts at the left edge
            
            obj.direct_feedthrough_flag = true;

            % span of this part of the wing is the span of the control
            % surface

            this_surface = obj.control_surfaces(min_left_edge_index);
            this_span = this_surface.span;
            
            
            contained_wing_center_body_frame = [0; this_span / 2 + point_along_wing; 0];

            
            contained_wing_frame_xyz = left_edge_of_wing + obj.getContainedWingFrameXYZ(contained_wing_center_body_frame, rpy);

            [model,contained_wing_frame_id] = addFrame(model,RigidBodyFrame(parent,contained_wing_frame_xyz,rpy,[this_surface.name '_contained_wing' num2str(length(obj.contained_wings)) '_of_' name '_frame']));
            
            this_contained_wing = RigidBodyWingWithControlSurface(contained_wing_frame_id, profile, chord, this_span, stall_angle, nominal_speed, this_surface);
            
            
            obj.input_limits = [ obj.input_limits [ this_surface.min_deflection; this_surface.max_deflection] ];
            
            % remove this surface from consideration
            remaining_surfaces(min_left_edge_index) = [];
            

          else
            % make a non-control surfaced wing since the control surface
            % does not start at the left edge

            if (min_left_edge_value == inf)
              this_span = span - point_along_wing;
            else
              this_span = min_left_edge_value - point_along_wing;
            end
            
            contained_wing_center_body_frame = [0; this_span / 2 + point_along_wing; 0];
            
            contained_wing_frame_xyz = left_edge_of_wing + obj.getContainedWingFrameXYZ(contained_wing_center_body_frame, rpy);
            
            [model,contained_wing_frame_id] = addFrame(model,RigidBodyFrame(parent,contained_wing_frame_xyz,rpy,['contained_wing' num2str(length(obj.contained_wings)) '_of_' name '_frame']));

            this_contained_wing = RigidBodyWing(contained_wing_frame_id, profile, chord, this_span, stall_angle, nominal_speed);
          end
          
          
          obj.contained_wings{length(obj.contained_wings) + 1} = this_contained_wing;
          obj.contained_wing_left_edges = [ obj.contained_wing_left_edges point_along_wing ];
          
          point_along_wing = point_along_wing + this_span;
          
        end
              
        
        
      end
      obj.name = name;
    end
    
    function contained_wing_frame_xyz = getContainedWingFrameXYZ(contained_wing_center_body_frame, rpy)
      % figure out where the center of this contained wing is, accounting
      % for possible roll/pitch/yaw in the spec

      rot_matrix = [ cos(rpy(3))*cos(rpy(2)), cos(rpy(3))*sin(rpy(2))*sin(rpy(1)) - sin(rpy(3))*cos(rpy(1)), cos(rpy(3))*sin(rpy(2))*cos(rpy(1)) + sin(rpy(3))*sin(rpy(1)); ...
                     sin(rpy(3))*cos(rpy(2)), sin(rpy(3))*sin(rpy(2))*sin(rpy(1)) + cos(rpy(3))*cos(rpy(1)), sin(rpy(3))*sin(rpy(2))*cos(rpy(1)) - cos(rpy(3))*sin(rpy(1)); ...
                     -sin(rpy(2)),    cos(rpy(2))*sin(rpy(1)),    cos(rpy(2))*cos(rpy(1))   ];



      contained_wing_frame_xyz = rot_matrix * contained_wing_center_body_frame;
      
    end
    
  end

end
