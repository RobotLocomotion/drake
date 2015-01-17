classdef ChineseYoYo < HybridDrakeSystem
  
  properties
    in_contact
    no_contact
  end
  
  methods
    function obj = ChineseYoYo
      in_contact = PlanarRigidBodyManipulator('tension.urdf');

      in_contact = PlanarRigidBodyManipulator('tension.urdf');

      % manually remove the ball from the pulley system:
      pulley_constraint = in_contact.position_constraints{1};
      cable_length_fcn = pulley_constraint.fcn;
      cable_length_fcn.pulley(2)=[];
      pulley_constraint = DrakeFunctionConstraint(pulley_constraint.lb, pulley_constraint.ub, cable_length_fcn);
      no_contact = in_contact.updatePositionEqualityConstraint(1,pulley_constraint);
      
      obj = obj@HybridDrakeSystem(0,1+getNumOutputs(in_contact));
      obj = setInputFrame(obj,getInputFrame(in_contact));
      obj = setOutputFrame(obj,MultiCoordinateFrame.constructFrame({CoordinateFrame('mode',1,'m',{'m'}),getOutputFrame(in_contact)}));
      
      % add the modes, but with an extra output containing the mode
      % numbers
      nmy = getNumOutputs(no_contact);
      tf = AffineTransform(getOutputFrame(no_contact),getOutputFrame(obj),[zeros(1,nmy);eye(nmy)],[1;zeros(nmy,1)]);
      obj = addMode(obj,cascade(no_contact,tf));
      tf = AffineTransform(getOutputFrame(no_contact),getOutputFrame(obj),[zeros(1,nmy);eye(nmy)],[2;zeros(nmy,1)]);
      obj = addMode(obj,cascade(in_contact,tf));
      
      obj = addTransition(obj,1,@collisionGuard,@collisionDynamics,false,true,2);
      obj = addTransition(obj,2,@aerialGuard,@collisionDynamics,false,true,1);
      
      obj.in_contact = in_contact;
      obj.no_contact = no_contact;
      obj = setStateFrame(obj,getOutputFrame(obj));
    end
    
    function [g,dg] = collisionGuard(obj,t,x,u)
      % todo: generalize this using line segements of the cable constraint?
      g = x(3) - 4;  % x(3) = load_z
      dg = [0,0,0,1,0,0,0];
    end
    
    function [g,dg] = aerialGuard(obj,t,x,u)
      g = 4 - x(3);
      dg = -[0,0,0,1,0,0,0];
    end
    
    function [xp,mode,status] = collisionDynamics(obj,mode,t,xm,u)
      if (mode==1)
        % then i have a collision
        mode = 2;

        q = xm(1:obj.no_contact.num_positions);
        v = xm(obj.no_contact.num_positions+1:end);
        H = manipulatorDynamics(obj.no_contact,q,v);
        Hinv = inv(H);

        [phi,J] = obj.in_contact.position_constraints{1}.eval(q);
        lb = obj.in_contact.position_constraints{1}.lb;
        ub = obj.in_contact.position_constraints{1}.ub;
        
        % solve zero post-transition velocities for all active constraints
        J = J*vToqdot(obj.no_contact, q);
        vp = (eye(obj.no_contact.num_velocities)-Hinv*J'*pinv(J*Hinv*J')*J)*v;
        xp = [q;vp];
      else % mode==2
        mode = 1;
        xp = xm;
      end
      status = 0;
    end
    
    function x0 = getInitialState(obj)
      % give a feasible initial state, otherwise the constraint solver will
      % barf
      x0 = Point(getStateFrame(obj));
      x0.m = 1;
      x0.load_x = 1;
      x0.load_z = 4.5;
      x0 = double(x0);
      x0(2:end) = resolveConstraints(obj.no_contact,x0(2:end));
    end
    
    function v = constructVisualizer(obj)
      v1 = constructVisualizer(obj.no_contact);
      v2 = constructVisualizer(obj.in_contact);
      v1.xlim = [-5 5];
      v1.ylim = [-.2 6.2];
      v2.xlim = v1.xlim; v2.ylim = v1.ylim;
      
      function mydraw(t,y)
        mode = y(1); y=y(2:end);
        if mode==1,
          v1.drawWrapper(t,y);
        elseif mode==2
          v2.drawWrapper(t,y);
        else
          error('bad mode number');
        end
      end
      v = FunctionHandleVisualizer(getOutputFrame(obj),@mydraw);
    end
    
  end
  
  methods (Static)
    function runPassive()
      r = ChineseYoYo();
      v = r.constructVisualizer();
      
      x0 = getInitialState(r);
      v.drawWrapper(0,x0);
      ytraj = simulate(r,[0 3],x0);
      v.playback(ytraj,struct('slider',true));
    end
  end
end
