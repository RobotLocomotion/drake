classdef CableAndPulleys < drakeFunction.kinematic.Kinematic

  methods
    function obj = CableAndPulleys(rbm,name,terminatorA_frame,terminatorA_xyz,terminatorB_frame,terminatorB_xyz)
      obj = obj@drakeFunction.kinematic.Kinematic(rbm,CoordinateFrame([name,'_length'],1,'l',{'length'}));
      obj.terminatorA_frame = terminatorA_frame;
      obj.terminatorA_xyz = terminatorA_xyz;
      obj.terminatorB_frame = terminatorB_frame;
      obj.terminatorB_xyz = terminatorB_xyz;
    end

    function obj = addPulley(obj,frame,xyz,axis,radius,number_of_wraps,crossover)
      % note: the order of addition DOES MATTER

      p.frame = frame;
      p.xyz = xyz;
      p.axis = axis;
      p.radius = radius;
      p.number_of_wraps = number_of_wraps;
      p.crossover = crossover;
      
      obj.pulley = horzcat(obj.pulley, p);
    end
    
    function [length_squared,J] = eval(obj,q)
      kinsol = obj.rbm.doKinematics(q);
      
      [terminatorA,JA] = forwardKin(obj.rbm,kinsol,obj.terminatorA_frame,obj.terminatorA_xyz);

      
      
      [terminatorB,JB] = forwardKin(obj.rbm,kinsol,obj.terminatorB_frame,obj.terminatorB_xyz);
      
      vec = terminatorA - terminatorB;
      Jvec = JA - JB;

      length_squared = vec'*vec;
      J = 2*vec'*Jvec;
    end
    
    function obj = updateBodyIndices(obj,map_from_old_to_new)
      obj.terminatorA_frame = map_from_old_to_new(obj.terminatorA_frame);
      obj.terminatorB_frame = map_from_old_to_new(obj.terminatorB_frame);
    end
    
    function obj = updateForRemovedLink(obj,model,body_ind)
      if (obj.terminatorA_frame == body_ind)
        obj.terminatorA_xyz = model.body(body_ind).Ttree(1:end-1,:)*[obj.terminatorA_xyz;1];
        obj.terminatorA_frame = model.body(body_ind).parent;
      end
      if (obj.terminatorB_frame == body_ind)
        obj.terminatorB_xyz = model.body(body_ind).Ttree(1:end-1,:)*[obj.terminatorB_xyz;1];
        obj.terminatorB_frame = model.body(body_ind).parent;
      end
    end
    
    function obj = updateBodyCoordinates(obj,body_ind,T_old_body_to_new_body)
      error('need to implement this.  (see changeRootLink)');
    end
    
  end

  properties
    terminatorA_frame  % link id or frame id for one end of the cable
    terminatorA_xyz    % position in terminatorA_frame
    
    terminatorB_frame  % link id or frame id for the other end
    terminatorB_xyz    % position in terminatorB_frame
    
    pulley = [];
  end
end