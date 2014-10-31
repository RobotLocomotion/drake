classdef RigidBodyFrame
  % A number of RigidBodyElements (e.g. sensors, actuators) require 
  % their own Cartesian frame relative to a RigidBody's frame.  This
  % class provides the common utilities for this
  
  methods
    function obj = RigidBodyFrame(body_ind,xyz,rpy,name)
      sizecheck(body_ind,1);
      sizecheck(xyz,[3 1]);
      sizecheck(rpy,[3 1]);
      obj.body_ind = body_ind;
      obj.T = [rotz(rpy(3))*roty(rpy(2))*rotx(rpy(1)),xyz; 0,0,0,1];
      if nargin>3
        typecheck(name,'char');
        obj.name = name;
      end
    end
    
    function obj = updateBodyIndices(obj,map_from_old_to_new)
      obj.body_ind = map_from_old_to_new(obj.body_ind);
    end    
    
    function obj = updateForRemovedLink(obj,model,body_ind)
      if (obj.body_ind == body_ind)
        obj.T = model.body(body_ind).Ttree*obj.T;
        obj.body_ind = model.body(body_ind).parent;
      end
    end
    
    function obj = updateBodyCoordinates(obj,body_ind,T_old_body_to_new_body)
      if (obj.body_ind == body_ind)
        obj.T = obj.T*T_old_body_to_new_body;
      end
    end
    
    function obj = bindParams(obj, model, pval)
      % Checks for parameters inside this frame and binds them to real
      % values
      %
      % @param model model we are a part of
      % @param pval values to set
      %
      % @retval obj updated frame object
      
      if isa(obj.T, 'msspoly')
        % this is a paramter
        obj.param_binding_T = obj.T;
        
        % bind it to a value
        fr = getParamFrame(model);
        obj = obj.updateParams(fr.getPoly(), pval);
      end
      
      
    end
    
    function obj = updateParams(obj, poly, pval)
      % Checks for parameters inside this frame and binds them to real
      % values
      %
      % @param poly parameter frame's polynomials (from frame.getPoly())
      % @param pval input values for the parameters
      %
      % @retval obj updated frame
      
      
      % first, check inside this frame for parameters
        

      % check to see if we have data for this frame
      if ~isempty(obj.param_binding_T)
        obj.T = double(subs(obj.param_binding_T, poly, pval));
      end
      
    end
    
  end

  properties
    name
    body_ind
    T
    param_binding_T = []; % if this frame has parameters, the msspoly representation is stored here
  end
end

