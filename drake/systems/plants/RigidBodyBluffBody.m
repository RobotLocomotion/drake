classdef RigidBodyBluffBody < RigidBodyForceElement
  % Element that provides an aerodynamic drag force at a point
  
  properties
    
    coefficient_drag_x; % coefficient of drag in the x (forward) direction
    coefficient_drag_y; % coefficient of drag in the y (left/right) direction
    coefficient_drag_z; % coefficient of drag in the z (up/down) direction
    
    %Air density for 20 degC dry air, at sea level
    rho = 1.204;
    
    area_x; % area of the drag surface in x
    area_y; % area of the drag surface in y
    area_z; % area of the drag surface in z
    
    visual_geometry; % true if drawing from the URDF properties
    parent_id; % id of parent body from the URDF
    
    kinframe;
    
  end
  
  
  methods
    
    function obj = RigidBodyBluffBody(frame_id, coefficient_drag_x, coefficient_drag_y, coefficient_drag_z, area_x, area_y, area_z)
      % Provides an aerodynamic drag force at a point
      %
      % Drag force is computed as:
      %  \f$ F_d = \frac{1}{2} \rho v^2 C_d A \f$
      %
      % where:
      % <pre>
      %  F_d: drag force
      %  rho: air density
      %  v: velocity
      %  C_d: coefficient of drag
      %  A: area
      % </pre>
      %  
      %
      % @param frame_id
      % @param coefficient_drag_x coefficient of drag in the x (forward) direction
      % @param coefficient_drag_y coefficient of drag in the y (left/right) direction
      % @param coefficient_drag_z coefficient of drag in the z (up/down) direction
      % @param area_x of the object in drag in the x direction
      % @param area_y of the object in drag in the y direction
      % @param area_z of the object in drag in the z direction
      %
      % @retval obj newly constructed object

      obj.coefficient_drag_x = coefficient_drag_x;
      obj.coefficient_drag_y = coefficient_drag_y;
      obj.coefficient_drag_z = coefficient_drag_z;
      
      obj.area_x = area_x;
      obj.area_y = area_y;
      obj.area_z = area_z;
      
      obj.kinframe = frame_id;
      
      
    end
    
    function [force] = computeSpatialForce(obj,manip,q,qd)
      % Computes the force and derivative of force for the aerodynamic
      % drag on this object.
      %
      % @param manip RigidBodyManipulator we are a part of
      % @param q state vector
      % @param qd time derivative of state vector
      %
      % @retval force force produced by the drag
      % @retval dforce gradient of force produced by the drag
      
      
      frame = getFrame(manip,obj.kinframe);
      kinsol = doKinematics(manip,q,true,true,qd);
      
      % Compute airspeed over this point in xyz
      
      wingvel_struct = RigidBodyWing.computeWingVelocity(obj.kinframe, manip, q, qd, kinsol);
      
      % convert world velocity to velocity in the body frame
      
      origin = zeros(3, 1);
      
      body_origin = manip.bodyKin(kinsol, obj.kinframe, origin);
      body_wingvel = manip.bodyKin(kinsol, obj.kinframe, wingvel_struct.wingvel_world_xyz);
      
      body_wingvel = body_wingvel - body_origin;
      
      %keyboard
      
      velocity_x = body_wingvel(1);
      velocity_y = body_wingvel(2);
      velocity_z = body_wingvel(3);
      
      force_x = -sign(velocity_x) * 0.5 * obj.rho * velocity_x * velocity_x * obj.coefficient_drag_x * obj.area_x;
      force_y = -sign(velocity_y) * 0.5 * obj.rho * velocity_y * velocity_y * obj.coefficient_drag_y * obj.area_y;
      force_z = -sign(velocity_z) * 0.5 * obj.rho * velocity_z * velocity_z * obj.coefficient_drag_z * obj.area_z;
      
      
      
      f_body = [force_x; force_y; force_z];
      
      % map body forces back to world coordinates
      world_origin = manip.forwardKin(kinsol, obj.kinframe, origin);
      f = manip.forwardKin(kinsol, obj.kinframe, f_body);
      f = f - world_origin;
      
      f = manip.cartesianForceToSpatialForce(kinsol, frame.body_ind, zeros(3,1), f);
      
      force = sparse(6, getNumBodies(manip)) * q(1); % q(1) for taylorvar
      
      force(:, frame.body_ind) = f;
      
      % TODO
      dforce = 0;
      
      
    end
    
    function varargout = addBluffBodyVisualShapeToBody(varargin)
      errorDeprecatedFunction('addBluffBodyVisualGeometryToBody');
    end
    
    function model = addBluffBodyVisualGeometryToBody(obj, model, body)
      % Adds a visual geometry of the bluff body to the model on the body
      % given for drawing in a visualizer.
      %
      % @param model manipulator the wing is part of
      % @param body body to add the visual geometry to
      %
      % @retval model updated model

      min_size = 0.001;
      
      size_x = max(min_size, sqrt(obj.area_x));
      size_y = max(min_size, sqrt(obj.area_y));
      size_z = max(min_size, sqrt(obj.area_z));
      
      box_size = [ size_x size_y size_z ];
      
      geometry = RigidBodyBox(box_size, model.getFrame(obj.kinframe).T);
      geometry = geometry.setColor([1 0 0]); % red
      geometry.name = [obj.name '_urdf_geometry'];
      
      model = model.addVisualGeometryToBody(body, geometry);

    end
    
    function [ obj, model ] = onCompile(obj, model)
      % Update visual geometry on compile.  This should be called after a parameter
      % update that may change the automatic drawing of geometry (ie the area
      % of the drag force is changed).
      %
      % @param model RigidBodyManipulator this is a part of
      %
      % @retval obj updated object, which you need to put back into
      % the updated model
      % @retval model updated model
      
      
      if (obj.visual_geometry)
        
        
        % remove any existing geometry for this body
        model = model.removeVisualGeometryFromBody(obj.parent_id, [ obj.name '_urdf_geometry' ]);
        
        % add new geometry
        model = addBluffBodyVisualGeometryToBody(obj, model, obj.parent_id);
        
        
      end
      
    end
    
  end
  
  methods (Static)
    
    function [model,obj] = parseURDFNode(model,name,robotnum,node,options)
      % Parse URDF node for drag object.
      
      elNode = node.getElementsByTagName('parent').item(0);
      parent = findLinkId(model,char(elNode.getAttribute('link')),robotnum);
      
      % get the xyz position of the element
      xyz=zeros(3,1);
      elnode = node.getElementsByTagName('origin').item(0);
      if ~isempty(elnode)
        if elnode.hasAttribute('xyz')
          xyz = reshape(parseParamString(model,robotnum,char(elnode.getAttribute('xyz'))),3,1);
        end
      end
      
      rpy = zeros(3,1);
      [model,this_frame_id] = addFrame(model,RigidBodyFrame(parent,xyz,rpy,[name,'_frame']));
      
      
      
      % get the drag coefficients of the element
      cdrag_x = parseParamString(model,robotnum,char(node.getAttribute('coefficient_drag_x')));
      cdrag_y = parseParamString(model,robotnum,char(node.getAttribute('coefficient_drag_y')));
      cdrag_z = parseParamString(model,robotnum,char(node.getAttribute('coefficient_drag_z')));
      
      this_area_x = parseParamString(model,robotnum,char(node.getAttribute('area_x')));
      this_area_y = parseParamString(model,robotnum,char(node.getAttribute('area_y')));
      this_area_z = parseParamString(model,robotnum,char(node.getAttribute('area_z')));
      
      visual_geometry_urdf = parseParamString(model,robotnum,char(node.getAttribute('visual_geometry')));
      
      if isempty(visual_geometry_urdf)
        % visual geometry defaults to on
        visual_geometry_urdf = 1;
      end
      
      obj = RigidBodyBluffBody(this_frame_id, cdrag_x, cdrag_y, cdrag_z, this_area_x, this_area_y, this_area_z);
      obj.name = name;
      
      obj.visual_geometry = visual_geometry_urdf;
      obj.parent_id = parent;
      
      % bind parameters, because if there is a parameter on area, we need
      % to know what it is (right now) for correct drawing
      obj = obj.bindParams(model, double(model.getParams()));
      bound_frame = model.getFrame(this_frame_id).bindParams(model, double(model.getParams()));
      model = model.setFrame(this_frame_id, bound_frame);
      
      if (obj.visual_geometry)
        model = addBluffBodyVisualGeometryToBody(obj, model, parent);
      end
      
      
      
    end
    
  end
  
  
end
