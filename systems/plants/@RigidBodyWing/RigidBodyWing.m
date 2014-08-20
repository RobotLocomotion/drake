classdef RigidBodyWing < RigidBodyForceElement
  % RigidBodyWing is a container class for one or more RigidBodySubWing's.
  % This allows us to handle wings with control surfaces as multiple
  % smaller parts of a wing.
  %
  % On initialization, we create the appropriate RigidBodySubWing objects.
  
  properties
    
    subwings;
    
  end
  

  methods
    function obj = RigidBodyWing(frame_id, profile, chord, span, stallAngle, velocity)
      % This constructor supports no arguments or the arguments above.
      % Given the parameters above, we create one RigidBodySubWing with the
      % parameters described there.
      
      if nargin > 0
      
        thisWing = RigidBodySubWing(frame_id, profile, chord, span, stallAngle, velocity);
      
        obj.subwings(end) = thisWing;
        
      else
        % with no arguments, this is probably the URDF constructor
        
        % no op
        
      end
    end
    
    
    function [force, dforce] = computeSpatialForce(obj,manip,q,qd)
      % Calls the appropriate RigidBodySubWing.computeSpatialForce
      % for all of the subwings, adds the results, and returns.
      
      if length(obj.subwings) < 1
        error('computeSpatialForce called with no subwings.');
      end
      
      for i = 1 : length(obj.subwings)
        
        [this_force, this_dforce] = obj.subwings(i).computeSpatialForce(manip, q, qd);
        
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
          if any(rpy), error('rpy not implemented yet for wings'); end
        end
      end
      [model,frame_id] = addFrame(model,RigidBodyFrame(parent,xyz,rpy,[name,'_frame']));

      % search for control surfaces
      control_surface_nodes = node.getElementsByTagName('control_surface');
      
      if isempty(control_surface_nodes.item(0))
        
        profile = char(node.getAttribute('profile'));
        chord = parseParamString(model,robotnum,char(node.getAttribute('chord')));
        span = parseParamString(model,robotnum,char(node.getAttribute('span')));
        stall_angle = parseParamString(model,robotnum,char(node.getAttribute('stall_angle')));
        nominal_speed = parseParamString(model,robotnum,char(node.getAttribute('nominal_speed')));

        this_subwing = RigidBodySubWing(frame_id, profile, chord, span, stall_angle, nominal_speed);
        obj = RigidBodyWing();
        obj.subwings = [obj.subwings this_subwing ];
        
      else
        % deal with control surfaces
        
        % TODO
        
        obj = RigidBodyWing();
        
      end
    end
  end

end
