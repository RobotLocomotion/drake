classdef ChineseYoYo_twopulleys < HybridDrakeSystem
  
  properties
    in_contact
    no_contact
  end
  
  methods
    function obj = ChineseYoYo_twopulleys
      in_contact = PlanarRigidBodyManipulator('chineseyoyo_twopulleys.urdf');
      
      % manually remove the ball from the pulley system:
      pulley_constraint = in_contact.position_constraints{1};
      cable_length_fcn = pulley_constraint.fcn;
      cable_length_fcn.pulley(2)=[];
      pulley_constraint = DrakeFunctionConstraint(pulley_constraint.lb, pulley_constraint.ub, cable_length_fcn);
      no_contact = in_contact.updatePositionEqualityConstraint(1,pulley_constraint);
      
      modeStates = 1; %number of discrete state variables, here only one mode variable
      obj = obj@HybridDrakeSystem(getNumInputs(in_contact),...
        getNumOutputs(in_contact) + modeStates);
      obj = setInputFrame(obj,getInputFrame(in_contact));
      
      % construct an output frame which has the mode number as the first
      % output, followed by the state of the current mode
      obj = setOutputFrame(obj,MultiCoordinateFrame.constructFrame({CoordinateFrame('mode',1,'m',{'m'}),getOutputFrame(in_contact)}));
      
      % add the modes, using transforms to produce the extra output containing the mode number
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
      % detect when the ball makes contact with the string
      % currently implemented as: z position of the ball <= 4
      % todo: generalize this using line segements of the cable constraint?
      g = x(3) - 4;  % x(3) = load_z
      dg = [0,0,0,1,0,0,0];
    end
    
    function [g,dg] = aerialGuard(obj,t,x,u)
      % detect when the ball leaves the string
      % currently implemented as: z position of the ball > 4
      % todo: generalize this
      g = 4 - x(3);
      dg = -[0,0,0,1,0,0,0];
    end
    
    function [xp,modep,status] = collisionDynamics(obj,mode,t,xm,u)
      q = xm(1:obj.no_contact.num_positions);
      v = xm(obj.no_contact.num_positions+1:end);
      H = manipulatorDynamics(obj.no_contact,q,v);
      Hinv = inv(H);

      if (mode==1)
        % then i have a collision
        modep = 2;

        [phi,J] = obj.in_contact.position_constraints{1}.eval(q);
      else % mode==2
        modep = 1;
        
        % note: this shouldn't do any mechanical work (phido0t should already 
        % be zero at the transition, because the in_contact phi is equivalent 
        % to the no_contact phi when the ball is tangent to the cable), but 
        % let's put it in for numerical reasons
        [phi,J] = obj.no_contact.position_constraints{1}.eval(q);
      end
      % solve zero post-transition velocities for all active constraints
      kinsol = obj.no_contact.doKinematics(q);
      J = J*vToqdot(obj.no_contact, kinsol);  % note: this will be the same for both modes
      vp = (eye(obj.no_contact.num_velocities)-Hinv*J'*pinv(J*Hinv*J')*J)*v;
      xp = [q;vp];
      
      status = 0;
    end
    
    function x0 = getInitialState(obj)
      % give a feasible initial state, otherwise the constraint solver will
      % barf
      x0 = Point(getStateFrame(obj));
      x0.m = 1;
      x0.load_x = -0.25;  % was 1
%       x0.load_x = -0.0585;
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
    
    function [T,U] = energy(obj,x)
      ind = findLinkId(obj.no_contact,'tensioner');
      [T,U] = energy(obj.no_contact,x(2:end));
    end
    
  end
  
  methods (Static)
    function runPassive()
      r = ChineseYoYo();
      v = r.constructVisualizer();
      
      x0 = getInitialState(r);
      v.drawWrapper(0,x0);
      [ytraj,xtraj] = simulate(r,[0 10],x0);
      v.playback(ytraj,struct('slider',true));
      
      if (1) 
        % energy / cable length plotting 
        % note, set alpha=0 in Manipulator/computeConstraintForce to reveal
        % some artifacts, especially at the release guard when
        % the pulley effectively hits a hard-stop.  this is due to the fact
        % that phidot>0 during the in_contact phase.  it's hard to tell if
        % this is numerical artifact or a bad gradient in CableLength.
        tt=getBreaks(xtraj);E=tt;cable_length=tt;
        nq=getNumPositions(r.no_contact);
        for i=1:length(tt)
          x = xtraj.eval(tt(i));
%           if i >= length(tt)/20 keyboard; end
          [T,U] = energy(r,x);
          E(i)= T+U;
          if (x(1)==1)
            cable_length(i)=r.no_contact.position_constraints{1}.fcn.eval(x(2:(1+nq)));
          else
            cable_length(i)=r.in_contact.position_constraints{1}.fcn.eval(x(2:(1+nq)));
          end
        end
        figure(1); clf;
        subplot(2,1,1); plot(tt,E, 'LineWidth', 2);
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$\mathcal{H}$', 'Interpreter', 'LaTeX', 'FontSize', 15)
        subplot(2,1,2); plot(tt,cable_length, 'LineWidth', 2);
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$\phi(q)$', 'Interpreter', 'LaTeX', 'FontSize', 15)
        
        t = linspace(xtraj.tspan(1), xtraj.tspan(end), 1001);
        x = eval(xtraj, t);
        figure(2), clf
        subplot(2,2,1)
        plot(t,x(2,:), 'LineWidth', 2)
        axis('tight')
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$\theta$', 'Interpreter', 'LaTeX', 'FontSize', 15)
        subplot(2,2,2)
        plot(t,x(3,:), 'LineWidth', 2)
        axis('tight')
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$x$', 'Interpreter', 'LaTeX', 'FontSize', 15)
        subplot(2,2,3:4)
        plot(t, x(4,:), 'LineWidth', 2)
        axis('tight')
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$z$', 'Interpreter', 'LaTeX', 'FontSize', 15)
      end
    end
  end
end
