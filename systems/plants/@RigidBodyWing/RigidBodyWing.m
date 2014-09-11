classdef RigidBodyWing < RigidBodyForceElement
  % RigidBodyWing is a container class for one or more RigidBodySubWing's.
  % This allows us to handle wings with control surfaces as multiple
  % smaller parts of a wing.
  %
  % On initialization, we create the appropriate RigidBodySubWing objects.
  
  properties
    
    subwings = {}; % array of RigidBodySubWing's that make up this wing
    
    subwing_left_edges; % sorted array of points along the wing denoting the left edge of each subwing. Index is the same as obj.subwings.
    
    control_surfaces; % array of control surfaces on this wing, sorted by order appearing in the URDF
    
    kinframe; % frame_id for the wing (note: there are different ones for the subwings based on their actual position)
    
  end
  

  methods
    function obj = RigidBodyWing(frame_id, profile, chord, span, stallAngle, velocity)
      % This constructor supports no arguments or the arguments above.
      % Given the parameters above, we create one RigidBodySubWing with the
      % parameters described there.
      
      if nargin > 0
      
        thisWing = RigidBodySubWing(frame_id, profile, chord, span, stallAngle, velocity);
      
        obj.subwings{length(obj.subwings) + 1} = this_subwing;
        
        obj.subwing_left_edges = 0;
        
        obj.kinframe = frame_id;
        
      else
        % with no arguments, we were probably called by the URDF constructor
        
        % no op
        
      end
    end
    
    
    function [force, B_force, dforce, dB_force ] = computeSpatialForce(obj,manip,q,qd)
      % Calls the appropriate RigidBodySubWing.computeSpatialForce
      % for all of the subwings, adds the results, and returns.
      
      if length(obj.subwings) < 1
        error('computeSpatialForce called with no subwings.');
      end
      
      control_surface_flag = false;
      
      for i = 1 : length(obj.subwings)
        
        if (obj.subwings{i}.has_control_surface)
          
          if control_surface_flag
            error('multiple control surfaces on one wing not supported.');
          end
          
          control_surface_flag = true;
          
          [force, B_force, dforce, dB_force ] = obj.subwings{i}.computeSpatialForce(manip,q,qd);
          
        else
        
          [this_force, this_dforce] = obj.subwings{i}.computeSpatialForce(manip, q, qd);
          if i == 1
            force = this_force;
            dforce = this_dforce;
          else
            force = force + this_force;
            dforce = dforce + this_dforce;
          
          end
        end
        
      end
      
    end
        
    function obj = setInputNum(obj, input_num)
      % override RigidBodyForceElement.setInputNum to support subwings.
      
      obj.input_num = input_num;
      
      for i = 1 : length(obj.subwings)
        if obj.subwings{i}.has_control_surface
          obj.subwings{i} = obj.subwings{i}.setInputNum(input_num);
        end
      end
    end
    

  end

  methods (Static)
    function [model,obj] = parseURDFNode(model,robotnum,node,options)
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
      
      obj = RigidBodyWing();
      
      obj.kinframe = this_frame_id;
      
      profile = char(node.getAttribute('profile'));
      chord = parseParamString(model,robotnum,char(node.getAttribute('chord')));
      span = parseParamString(model,robotnum,char(node.getAttribute('span')));
      stall_angle = parseParamString(model,robotnum,char(node.getAttribute('stall_angle')));
      nominal_speed = parseParamString(model,robotnum,char(node.getAttribute('nominal_speed')));
      
      left_edge_of_wing = xyz - [0; span/2; 0];
      
      if isempty(control_surface_nodes.item(0))
        % no control surfaces
        
        this_subwing = RigidBodySubWing(obj.kinframe, profile, chord, span, stall_angle, nominal_speed);
        
        obj.subwings{length(obj.subwings) + 1} = this_subwing;
        obj.subwing_left_edges = 0;
        
      else
        % deal with control surfaces
        
        % load control surface data
        count = 0;

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
          
          for other_surface = obj.control_surfaces
            if strcmp(other_surface.name, surface_name)
              error(['Duplicate control surface name "' surface_name '"']);
            end
          end
          
          obj.control_surfaces = [ obj.control_surfaces ControlSurface(surface_name, surface_chord, surface_span, surface_left_edge_position_along_wing, surface_min_deflection, surface_max_deflection) ];

          
          count = count + 1;
          this_surface = control_surface_nodes.item(count);
          
        end
        
        % now we split the wing up into multiple segments that either
        % contain one of the control surfaces or don't
        
        % do some sanity checks to make sure that the control surfaces do
        % not overlap
        for surface = obj.control_surfaces
          for surface2 = obj.control_surfaces
            
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
            
            this_subwing_center_y = this_span / 2 + point_along_wing;
            
            
            subwing_frame_xyz = left_edge_of_wing + [0; this_subwing_center_y; 0];
            [model,subwing_frame_id] = addFrame(model,RigidBodyFrame(parent,subwing_frame_xyz,rpy,[this_surface.name '_subwing' num2str(length(obj.subwings)) '_of_' name '_frame']));
            
            this_subwing = RigidBodySubWingWithControlSurface(subwing_frame_id, profile, chord, this_span, stall_angle, nominal_speed, this_surface);
            
            
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
            
            this_subwing_center_y = this_span / 2 + point_along_wing;
            
            subwing_frame_xyz = left_edge_of_wing + [0; this_subwing_center_y; 0];
            [model,subwing_frame_id] = addFrame(model,RigidBodyFrame(parent,subwing_frame_xyz,rpy,['subwing' num2str(length(obj.subwings)) '_of_' name '_frame']));

            this_subwing = RigidBodySubWing(subwing_frame_id, profile, chord, this_span, stall_angle, nominal_speed);
          end
          
          
          obj.subwings{length(obj.subwings) + 1} = this_subwing;
          obj.subwing_left_edges = [ obj.subwing_left_edges point_along_wing ];
          
          point_along_wing = point_along_wing + this_span;
          
        end
              
        
        
      end
      obj.name = name;
    end
    
  end

end
