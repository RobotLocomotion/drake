classdef RigidBodyDragForce < RigidBodyForceElement
  % Element that provides an aerodynamic drag force at a point
  
  properties
    
    coefficient_drag_x; % coefficient of drag in the x (forward) direction
    coefficient_drag_y; % coefficient of drag in the y (left/right) direction
    coefficient_drag_z; % coefficient of drag in the z (up/down) direction
    
    kinframe;
    
  end
  
  
  methods
    
    function obj = RigidBodyDragForce(frame_id, coefficient_drag_x, coefficient_drag_y, coefficient_drag_z)
      % Provides an aerodynamic drag force at a point
      %
      % @param frame_id
      % @param coefficient_drag_x coefficient of drag in the x (forward) direction
      % @param coefficient_drag_y coefficient of drag in the y (left/right) direction
      % @param coefficient_drag_z coefficient of drag in the z (up/down) direction
      %
      % @retval obj newly constructed object

      obj.coefficient_drag_x = coefficient_drag_x;
      obj.coefficient_drag_y = coefficient_drag_y;
      obj.coefficient_drag_z = coefficient_drag_z;
      
      obj.kinframe = frame_id;
      
      
    end
    
    function [force, dforce] = computeSpatialForce(obj,manip,q,qd)
      % Computes the force and derivative of force for the aerodynamic
      % drag on this object.
      %
      % @param manip RigidBodyManipulator we are a part of
      % @param q state vector
      % @param qd time derivative of state vector
      %
      % @retval force force produced by the drag
      % @retval dforce gradient of force produced by the drag
      
      
      % Compute airspeed over this point in xyz
      
      
      
      force = 0;
      dforce = 0;
      
      
    end
    
    
  end
  
  methods (Static)
    
    function [model,obj] = parseURDFNode(model,robotnum,node,options)
      % Parse URDF node for drag object.
      
      name = char(node.getAttribute('name'));
      name = regexprep(name, '\.', '_', 'preservecase');

      elNode = node.getElementsByTagName('parent').item(0);
      parent = findLinkInd(model,char(elNode.getAttribute('link')),robotnum);
      
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
      
      
      obj = RigidBodyDragForce(this_frame_id, cdrag_x, cdrag_y, cdrag_z);
      obj.name = name;
      
    end
    
  end
  
  
end