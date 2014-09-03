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

      if (nargout>2)  % then compute gradients
        % thrust
        kinsol = doKinematics(manip,q,true);
        [x,J,dJ] = forwardKin(manip,kinsol,obj.kinframe,zeros(3,1));
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
        [x1,J1,dJ1] = forwardKin(manip,kinsol,obj.kinframe,n1);
        [axis_world_m1,Jaxis_world_m1] = forwardKin(manip,kinsol,obj.kinframe,n2+n1);
        daxis_world_m1 = Jaxis_world_m1 - J1;
        axis_world_m1 = axis_world_m1 - x1;
        
        [x2,J2,dJ2] = forwardKin(manip,kinsol,obj.kinframe,-n1);
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
        [x,J] = forwardKin(manip,kinsol,obj.kinframe,zeros(3,1));
        axis_world = forwardKin(manip,kinsol,obj.kinframe,obj.axis);
        axis_world = axis_world-x;
        
        % aerodynamic drag moment
        N = null(obj.axis');
        n1 = N(:,1);
        n2 = N(:,2);
        [x1,J1] = forwardKin(manip,kinsol,obj.kinframe,n1);
        axis_world_m1 = forwardKin(manip,kinsol,obj.kinframe,n2+n1);
        axis_world_m1 = axis_world_m1 - x1;
        
        [x2,J2] = forwardKin(manip,kinsol,obj.kinframe,-n1);
        axis_world_m2 = forwardKin(manip,kinsol,obj.kinframe,-n2-n1);
        axis_world_m2 = axis_world_m2 - x2;        
                
      end
      
      % apply force along the z-axis of the reference frame
      B_mod(:,obj.input_num) = obj.scale_factor_thrust*J'*axis_world;
      
      % apply a couple corresponding to the moment due to aerodynamic drag
      B_mod(:,obj.input_num) = B_mod(:,obj.input_num) + 0.5*obj.scale_factor_moment*J1'*axis_world_m1;
      B_mod(:,obj.input_num) = B_mod(:,obj.input_num) + 0.5*obj.scale_factor_moment*J2'*axis_world_m2;

    end
    
  end
  
  methods (Static)
    function [model,obj] = parseURDFNode(model,robotnum,node,options)
      name = char(node.getAttribute('name'));
      name = regexprep(name, '\.', '_', 'preservecase');
      
      elnode = node.getElementsByTagName('parent').item(0);
      parent = findLinkInd(model,char(elnode.getAttribute('link')),robotnum);
      
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
      
      axis = [1; 0; 0];
      elnode = node.getElementsByTagName('axis').item(0);
      if ~isempty(elnode)
        axis = reshape(parseParamString(model,robotnum,char(elnode.getAttribute('xyz'))),3,1);
        axis = axis/(norm(axis)+eps); % normalize
      end
      
      limits = [-inf,inf];
      if node.hasAttribute('lower_limit')
        limits(1) = parseParamString(model,robotnum,char(node.getAttribute('lower_limit')));
      end
      if node.hasAttribute('upper_limit')
        limits(2) = parseParamString(model,robotnum,char(node.getAttribute('upper_limit')));
      end
      
      obj = RigidBodyPropellor(frame_id, axis, scaleFacThrust, scaleFacMoment, limits); 
      obj.name = name;
    end
  end
  
end
