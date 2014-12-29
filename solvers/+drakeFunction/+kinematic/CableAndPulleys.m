classdef CableAndPulleys < drakeFunction.kinematic.Kinematic

  methods
    function obj = CableAndPulleys(rbm,name)
      obj = obj@drakeFunction.kinematic.Kinematic(rbm,CoordinateFrame([name,'_length'],1,'l',{'length'}));
    end

    function obj = addPulley(obj,frame,xyz,axis,radius,number_of_wraps)
      % note: the order of addition DOES MATTER

      p.frame = frame;
      p.xyz = xyz;
      p.axis = axis;
      p.radius = radius;
      p.number_of_wraps = number_of_wraps;
      
      obj.pulley = horzcat(obj.pulley, p);
    end
    
    function [length,J] = eval(obj,q)
      kinsol = obj.rbm.doKinematics(q);

      length = 0;
      J = 0*q';
      for i=1:numel(obj.pulley)
        [pt,dpt] = forwardKin(obj.rbm,kinsol,obj.pulley(i).frame,obj.pulley(i).xyz);
        
        if i>1
          vec = pt-last_pt;
          dvec = dpt-last_dpt;
          C = sqrt(vec'*vec);
          dC = vec'*dvec/C;
          
          d1 = 2*obj.pulley(i-1).radius;
          d2 = 2*obj.pulley(i).radius;
          if d1>0 || d2>0,
            obj.rbm.warning_manager.warnOnce('Drake:CableAndPulleys:RadiusNotImplementedYet','radius logic not implemented yet. pretending that radius = 0');
          end
          length = length+C;
          J = J+dC;
%          crossover = -dot(obj.pulley(i-1).axis,obj.pulley(i).axis);
%          if (dot(obj.pulley(i-1)
        end
        
        last_pt = pt; last_dpt = dpt;
      end
      
    end
    
    function [vertex,edge] = getSegments(obj,q)
      % this method is intended to make is easy to display the cable
      % @retval vertices is a 3-by-n list of 3D points (in world
      % coordinates)
      % @retval edges is a 2-by-m list of integer indices into vertices -
      % one column for each line segment
      % Note: we use this format because there is not necessarily a line
      % that should be drawn between each sequential vertex (e.g. it would
      % draw a line directly through a pulley).  
      
      kinsol = obj.rbm.doKinematics(q);

      vertex = [];
      edge = [];
      for i=1:numel(obj.pulley)
        pt = forwardKin(obj.rbm,kinsol,obj.pulley(i).frame,obj.pulley(i).xyz);
        
        if i>1
          d1 = 2*obj.pulley(i-1).radius;
          d2 = 2*obj.pulley(i).radius;
          if d1>0 || d2>0,
            obj.rbm.warning_manager.warnOnce('Drake:CableAndPulleys:RadiusNotImplementedYet','radius logic not implemented yet. pretending that radius = 0');
          end
          edge = horzcat(edge,[i-1;i]);
        end
        
        last_pt = pt; 
        vertex = horzcat(vertex,pt);
      end
    end
    
    function obj = updateBodyIndices(obj,map_from_old_to_new)
      for i=1:numel(obj.pulley)
        obj.pulley(i).frame = map_from_old_to_new(obj.pulley(i).frame);
      end
    end
    
    function obj = updateForRemovedLink(obj,model,body_ind)
      for i=1:numel(obj.pulley)
        if (obj.pulley(i).frame == body_ind)
          obj.pulley(i).xyz = model.body(body_ind).Ttree(1:end-1,:)*[obj.pulley(i).xyz;1];
          obj.pulley(i).frame = model.body(body_ind).parent;
        end
      end
    end
    
    function obj = updateBodyCoordinates(obj,body_ind,T_old_body_to_new_body)
      error('need to implement this.  (see changeRootLink)');
    end
    
  end

  properties
    pulley = [];
  end
end