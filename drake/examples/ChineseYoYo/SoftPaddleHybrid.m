classdef SoftPaddleHybrid < HybridDrakeSystem
  
  properties
    in_contact
    no_contact
    radius
    paddleId
    dxzPaddleFramePos
    dxzPaddleFrameNeg
    baseId
    emptyState
    options
  end
  
  methods
    function obj = SoftPaddleHybrid()
      in_contact = PlanarRigidBodyManipulator('SoftPaddle.urdf');
      taylorvar = false;

      %TODO: Set input frame
      %obj = setInputFrame(obj,CoordinateFrame('AcrobotInput',1,'u',{'tau'}));
      
      % manually remove the ball from the pulley system:
      pulley_constraint = in_contact.position_constraints{1};
      cable_length_fcn = pulley_constraint.fcn;
      
      %Update in contact
      if(taylorvar)
        pulley_constraint = DrakeFunctionConstraint(pulley_constraint.lb, ...
          pulley_constraint.ub, cable_length_fcn);
        pulley_constraint = setName(pulley_constraint,cable_length_fcn.name);
        in_contact = in_contact.updatePositionEqualityConstraint(1,pulley_constraint);
      end
      
      %Update no contact
      cable_length_fcn.pulley = cable_length_fcn.pulley([1 3 4]); %removing the disc with id 2 from the constraint
      pulley_constraint = DrakeFunctionConstraint(pulley_constraint.lb, ...
        pulley_constraint.ub, cable_length_fcn); %construct a new function constraint
      pulley_constraint = setName(pulley_constraint,cable_length_fcn.name);
      if(~taylorvar)
        pulley_constraint.grad_level = 2; %declare that the second derivative is provided
        pulley_constraint.grad_method = 'user';
      end
      
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
      tf = AffineTransform(getOutputFrame(in_contact),getOutputFrame(obj),[zeros(1,nmy);eye(nmy)],[2;zeros(nmy,1)]);
      obj = addMode(obj,cascade(in_contact,tf));
      
      obj = addTransition(obj,1,@collisionGuard,@collisionDynamics,false,true,2);
      obj = addTransition(obj,2,@aerialGuard,@collisionDynamics,false,true,1);
      
      obj.in_contact = in_contact;
      obj.no_contact = no_contact;
      obj = setStateFrame(obj,getOutputFrame(obj));
      
      %Precalculations for guards:
      obj.radius = 1;
      obj.paddleId = obj.no_contact.findLinkId('paddle'); % from findLinkId or findFrameInd)
      obj.dxzPaddleFramePos = [0,0,1];
      obj.dxzPaddleFrameNeg = [0,0,-1];
      obj.baseId = obj.no_contact.findLinkId('world+base');
      obj.options.base_or_frame_id = obj.baseId;
      obj.emptyState = Point(getStateFrame(obj));
      
    end
    
    function [g,dg] = collisionGuard(obj,t,x,u)
      % detect when the ball makes contact with the string
      % currently implemented as: z position of the ball <= 3.5
      % todo: generalize this using line segements of the cable constraint?
      
      
      npos=obj.no_contact.num_positions;
      %obj.rbm.doKinematics(q,nargout>2);
      kinsol=doKinematics(obj.no_contact, x(1:npos));
      
      % transform state into inertial frame
      pos_disc_global = [x(3);0;x(4)]; %TODO find state to inertial frame a proper function call
      
      %get point from frame
      [pos_disc_in_paddle_frame] = bodyKin(obj.no_contact,kinsol,obj.paddleId,pos_disc_global);
      
      %heightjoint = findPositionIndices(obj.in_contact,'load_z');
      g = pos_disc_in_paddle_frame(3)-obj.radius; %todo: make this more generic
      
      dxz = forwardKin(obj.no_contact,kinsol,obj.paddleId,obj.dxzPaddleFramePos,obj.options);		
      
      dg = obj.emptyState;
      dg.load_x =dxz(1);
      dg.load_z =dxz(3);
      
      dg = double(dg)';
      
    end
    
    function [g,dg] = aerialGuard(obj,t,x,u)
      % detect when the ball leaves the string
      % currently implemented as: z position of the ball > 3.5
      % todo: generalize this

      npos=obj.in_contact.num_positions;
      %obj.rbm.doKinematics(q,nargout>2);
      kinsol=doKinematics(obj.in_contact, x(1:npos));
      
      % transform state into inertial frame
      pos_disc_global = [x(3);0;x(4)]; %TODO find state to inertial frame a function call
      
      %get point from frame
      [pos_disc_in_paddle_frame] = bodyKin(obj.in_contact,kinsol,obj.paddleId,pos_disc_global);
      
      %heightjoint = findPositionIndices(obj.in_contact,'load_z');
      g = - pos_disc_in_paddle_frame(3) + obj.radius;
          
      dxz = forwardKin(obj.in_contact,kinsol,obj.paddleId,obj.dxzPaddleFrameNeg,obj.options);		
      
      dg = obj.emptyState;
      dg.load_x =dxz(1);
      dg.load_z =dxz(3);
      
      dg = double(dg)';
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
%       x0.load_x = -0.0378;  % was -0.045
      x0.load_x = -0.045;
      x0.load_z = 4.5;
      x0 = double(x0);
%       x0(2:end) = resolveConstraints(obj.in_contact,x0(2:end));
      x0(2:end) = resolveConstraints(obj.no_contact,x0(2:end));
      
      
%       x0(2:end) = [0.00015647; 0.14351; -0.14346; 4.4639; 0.00016919; -0.00050156; -0.084561; -1.6321];
      
    end
    
    function xDes = calcStateInContact(obj,x_load,z_load,paddle_angle)
      % give a feasible initial state, otherwise the constraint solver will
      % barf
      xDes = Point(getStateFrame(obj));
      xDes.m = 0; % in contact
      xDes.paddle_angle = paddle_angle;
      xDes.load_x = x_load; 
      xDes.load_z = z_load;
      xDes = double(xDes);
      xDes(2:end) = resolveConstraints(obj.in_contact,xDes(2:end));
    end
    
    function v = constructVisualizer(obj)
      v1 = constructVisualizer(obj.no_contact);
      v2 = constructVisualizer(obj.in_contact);
      v1.xlim = [-5 5];
      v1.ylim = [-.2 6.2];
      v2.xlim = v1.xlim; v2.ylim = v1.ylim;
      
      function mydraw(t,y)
        mode = y(1); y=y(2:end);
        if mode==1
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
    
%     function [c, dc] = evaluateConstraint(obj,x)
%         psi = x(1);
%         R = [cos(psi), 0, sin(psi); 0, 1, 0; -sin(psi), 0, cos(psi)];
%         c = [-3; 1; 3] + R*[x(3); 0; 0];
%         dc = [-x(3)*sin(psi), 0, 0, 0; 0, 0, 0, 0; x(3)*cos(psi)];
%     end
    
    function [utraj,xtraj]=shootOffTrajectory(obj)
        zInitial = 4.5;
        tStrike = sqrt(2/9.81)*sqrt(zInitial-4);
        vStrike = -9.81*tStrike;
        x0 = [0; 0.1434; -0.5; 4; 0; 0; 0; vStrike];
        
        % Get the desired xf in the inertial frame
        
        
%         xf1_in_inertial = forwardKin(obj, kinsol, obj.paddleId, [-0.5,0,1], obj.options);
%         xf2_in_inertial = forwardKin(obj, kinsol, obj.paddleId, [0.5,0,1], obj.options);
        
        lb = [-0.02; 0.13; -0.51; 3.9650; 0; 0; 0; 0];
        ub = [0.02; 0.15; 0.51; 4.0349; 0; 0; 0; 0];
        
        xTargetInitial = [0; 0.14; 0.03; 4; 0; 0; 0; 0];
        tf0 = 0.75; % initial guess
        
        N = 21;
        prog = DircolTrajectoryOptimization(obj.in_contact,N,[0.4 1.2]);
        prog = prog.addStateConstraint(ConstantConstraint(x0),1);
        prog = prog.addStateConstraint(BoundingBoxConstraint(lb,ub),N);   % Change this constraint
        prog = prog.addRunningCost(@cost);
        prog = prog.addFinalCost(@finalCost);
        
        traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xTargetInitial)]));   % Need to initialize well
        
        for attempts=1:10
            tic
            [xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init);
            toc
            if info==1, break; end
        end
        
        function [g,dg] = cost(dt,x,u)
            R = 1;
            g = sum((R*u).*u,1);
            dg = [zeros(1,1+size(x,1)),2*u'*R];
            return;
            
            xd = repmat([pi;0;0;0],1,size(x,2));
            xerr = x-xd;
            xerr(1,:) = mod(xerr(1,:)+pi,2*pi)-pi;
            
            Q = diag([10,10,1,1]);
            R = 100;
            g = sum((Q*xerr).*xerr + (R*u).*u,1);
            
            if (nargout>1)
                dgddt = 0;
                dgdx = 2*xerr'*Q;
                dgdu = 2*u'*R;
                dg = [dgddt,dgdx,dgdu];
            end
        end
        
        function [h,dh] = finalCost(t,x)
%             h = t;
%             dh = [1,zeros(1,size(x,1))];
%             return;
            
%             xd = repmat([pi;0;0;0],1,size(x,2));
            xerr = x-xd;
            xerr(1,:) = mod(xerr(1,:)+pi,2*pi)-pi;
            
            Qf = 100*diag([10,10,1,1]);
            h = sum((Qf*xerr).*xerr,1);
            h = 10*(x(3)-0.03)^2  + x(7)^2;
            
            if (nargout>1)
                dh = [0, 2*xerr'*Qf];
                dh = [0, 0, 0, 20*(x(3)-0.03), 0, 0, 2*x(7), 0];
            end
        end
        
    end

    function [utraj,xtraj]=inContactTrajectory(obj)
      zInitial = 4.5;
      tStrike = sqrt(2/9.81)*sqrt(zInitial-4);
      g = -9.81;
      vStrike = g*tStrike;
      xdrop = -0.5;
      x0 =             [ 0; 0.1434; -0.5;  3.9; 0; 0; 0; vStrike];
      xTargetInitial = [ 0; 0.14;    0.03; 3.9; 0; 0; 0; 0];
      
      % Get the desired xf in the inertial frame
      
      %         xf1_in_inertial = forwardKin(obj, kinsol, obj.paddleId, [-0.5,0,1], obj.options);
      %         xf2_in_inertial = forwardKin(obj, kinsol, obj.paddleId, [0.5,0,1], obj.options);
      
      to_options.lambda_bound = 21; %ToDo Check this?
      N = 11;
      Tmin = 0.1;
      Tmax = 1.2;
      traj_opt = ConstrainedDircolTrajectoryOptimization(obj.in_contact,N,[Tmin Tmax],to_options);
      
      delta = 1e-3; % if x0 is correct
      traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(x0-delta,x0+delta),1); % at the first time step constrain the state
      
      xLoadFinal_min = -0.3;
      xLoadFinal_max = 0.51;
      lb = [-0.5; -0.2; xLoadFinal_min; 3.9650; 0; 0; 0; 0];
      ub = [0.5; 1; xLoadFinal_max; 4.0349; 0; 0; 0; 0];
      
      traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(lb,ub),N);   % end constraint
      q = x0(1:4);
      qp = x0(5:8);
      [H,C,B] = manipulatorDynamics(obj.no_contact, q, qp);
            
      %ubegin_min = C(1)-100;
      %ubegin_max = C(1);
      %traj_opt = traj_opt.addInputConstraint(BoundingBoxConstraint(ubegin_min,ubegin_max),ceil(N/5));
      
      tf0 = (Tmax+Tmin)/2; % initial guess
      t_init = linspace(0,tf0,N);
      u = -60*ones(1,length(t_init)); %randomness can help
      %u = 40*cos(20*t_init);
      traj_init.x = PPTrajectory(foh([0 tf0],[double(x0),double(xTargetInitial)]));   % Need to initialize well
      %traj_init.x = PPTrajectory(foh(t_init,repmat(x0,1,N))); % same state all the way through
      traj_init.u = PPTrajectory(foh(t_init,u));
      traj_opt = traj_opt.setSolverOptions('snopt','print','snopt.out');
      traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',200);
      traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',500000);
      traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',500000);
      
      
      % traj_opt = traj_opt.setCheckGrad(true);
      
      traj_opt = traj_opt.addRunningCost(@running_cost_fun);
      traj_opt = traj_opt.addFinalCost(@final_cost_fun);
      
      [xtraj,utraj,ltraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);
      
      
      function [f,df] = running_cost_fun(h,x,u)
        R = 1;
        fixPoint = -0.029;
        kx = 90;
        kpsi = 5;
        kpsid = 1;
        fMult = ( (u+50)'*R*(u+50) + kpsi* x(1)'*x(1)+ kpsid* x(5)'*x(5) + kx*(x(3)-fixPoint)^2 );
        f = h * fMult;
        df = [fMult 2*h*kpsi*x(1) 0 2*h*kx*(x(3)-fixPoint) 0 2*h*kpsid*x(5) zeros(1,3) 2*h*(u+50)'*R];
      end
      
      
      function [f,df] = final_cost_fun(T,x)
        
        fixPoint = -0.029;
        kx = 10;
        kxd = 0;
        
        f = kx*(x(3)-fixPoint)^2 + kxd*(x(7))^2;
        df = [0 0 0 2*kx*(x(3)-fixPoint) 0 0 0 2*kxd*x(7) 0];
      end
      
      
      %       function [g,dg] = running_cost_fun(t,x,u)
      %         R = 1;
      %         g = sum((R*u).*u,1);
      %         dg = [zeros(1,1+size(x,1)),2*u'*R];
      %         return;
      %
      %         xd = repmat([pi;0;0;0],1,size(x,2));
      %         xerr = x-xd;
      %         xerr(1,:) = mod(xerr(1,:)+pi,2*pi)-pi;inContactTrajectory
      %
      %         Q = diag([10,10,1,1]);
      %         R = 100;
      %         g = sum((Q*xerr).*xerr + (R*u).*u,1);
      %
      %         if (nargout>1)
      %           dgddt = 0;
      %           dgdx = 2*xerr'*Q;
      %           dgdu = 2*u'*R;
      %           dg = [dgddt,dgdx,dgdu];
      %         end
      %       end
      %
      %       function [h,dh] = final_cost_fun(t,x)
      %         %             h = t;
      %         %             dh = [1,zeros(1,size(x,1))];
      %         %             return;
      %
      %         %             xd = repmat([pi;0;0;0],1,size(x,2));
      %         xerr = x-xd;
      %         xerr(1,:) = mod(xerr(1,:)+pi,2*pi)-pi;
      %
      %         Qf = 100*diag([10,10,1,1]);
      %         h = sum((Qf*xerr).*xerr,1);
      %         h = 10*(x(3)-0.03)^2  + x(7)^2;
      %
      %         if (nargout>1)
      %           dh = [0, 2*xerr'*Qf];
      %           dh = [0, 0, 0, 20*(x(3)-0.03), 0, 0, 2*x(7), 0];
      %         end
      %       end
      %
    end
  end
  
  methods (Static)
    
    function runFreeFall()
      r = SoftPaddleHybrid();
      freefall = r.no_contact;
      
      v = freefall.constructVisualizer();
      
      x0 = getInitialState(r);
      v.drawWrapper(0,x0(2:end));
      [ytraj,xtraj] = simulate(freefall,[0 1],x0(2:end));
      v.playback(ytraj,struct('slider',true));
    end
    function runPassiveWithoutSimulation()
      %r = SoftPaddleHybrid();
      %v = r.constructVisualizer();
      
      %x0 = getInitialState(r);
      %v.drawWrapper(0,x0);
      %skipping simulation step
      load('sphtraj.mat'); %loading in old simulation
      v.playback(ytraj,struct('slider',true));
      tt=getBreaks(xtraj);
      cl=tt;
      nq=getNumPositions(r.no_contact);
      dcl=zeros(length(tt),1,nq);
      ddcl=zeros(length(tt),1,nq*nq);
      for i=1:length(tt)
        x = xtraj.eval(tt(i));
        %           if i >= length(tt)/20 keyboard; end
        if (x(1)==1) %flight mode
          [cl(i),dcl(i,1,:),ddcl(i,1,:)]=r.no_contact.position_constraints{1}.fcn.eval(x((1:nq)+1));
        else
          [cl(i),dcl(i,1,:),ddcl(i,1,:)]=r.in_contact.position_constraints{1}.fcn.eval(x((1:nq)+1));
        end
      end
      
      figure(3); clf;
      subplot(2,1,1); plot(tt,dcl(:,1,4), 'LineWidth', 2);
      xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
      ylabel('$\dot{\phi(q)}$', 'Interpreter', 'LaTeX', 'FontSize', 15)
      subplot(2,1,2); plot(tt,ddcl(:,1,16), 'LineWidth', 2);
      xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
      ylabel('$\ddot{\phi(q)}$', 'Interpreter', 'LaTeX', 'FontSize', 15)
      
    end
    
    function f = fun(x)
        r = SoftPaddleHybrid();
%         u = SoftPaddleController();
        x_load = x;
        z_load = 3.9;
        paddle_angle = 0;
        x0 = r.calcStateInContact(x_load,z_load,paddle_angle);
        f = dynamics(r.modes{2},0,x0(2:end),0);
        f = f(7);
    end
    
    function [xFixed,fval,exitflag,output,jacobian] = findFixedPoint(xGuess)
        r = SoftPaddleHybrid();
        f = @(q) r.fun(q);
        [xFixed, fval, exitflag, output, jacobian] = fsolve(f,xGuess);
    end
    
    
    function [utraj,xtraj] = getRefTrajectory()
      obj = SoftPaddleHybrid();
      x0 = getInitialState(obj);
      [ytraj,xtraj] = simulate(obj,[0 5],x0);
      utraj = 0;
    end
    
    function gradTestCableLength()
      r = SoftPaddleHybrid();
      numtest = 10; %change this to test for various points
      cl=zeros(1,numtest); %init a cable length output
      nq=getNumPositions(r.no_contact);
      dcl=zeros(numtest,1,nq);
      ddcl=zeros(numtest,1,nq*nq);
      x_load = -2 + (2-(-2)) .*rand(numtest,1); % x position of load within -2 to 2 range (uniform dist)
      z_load = 2.5 + (3.9 - 2.5) .*rand(numtest,1); % z position of load within 2.5 to 3.9 range (uniform dist)
      paddle_angle = -pi/4 + (pi/4-(-pi/4)) .*rand(numtest,1); % angle of paddle in range -pi/4 to pi/4 range (uniform dist)
      
      %       paddle_angle = 0.15365;
      %       x_load = 0.79569;
      %       z_load = 4.5;
      %
      %       paddle_angle = 0.40945;
      %       x_load = 0.18235;
      %       z_load = 2.7739;
      %       load('collisionProb.mat','xInterest');
      %load('troubleX.mat','xDes');
      %       xTest = xInterest(:,1:3);
      %       numtest = size(xTest,2);
      %cableLengthFunction = r.in_contact.position_constraints{1}.fcn;
      for i = 1:numtest
        xDes = r.calcStateInContact(x_load(i),z_load(i),paddle_angle(i));
        %        xDes = xTest(:,i);
        %        [cl(i),dcl(i,1,:),ddcl(i,1,:)]=r.in_contact.position_constraints{1}.fcn.eval(x((1:nq)+1))
%         gradTest(@(q) (r.in_contact.position_constraints{1}.fcn.eval(q)),xDes((1:nq)+1));%,struct('input_names',{{'xDes'}},'output_name','dlength'));
%         if(i== numtest)
          q = xDes((1:nq)+1);
          v = rand(4,1);
          [l,dl, ddl] = geval(@(q) r.in_contact.position_constraints{1}.fcn.eval(q), q, struct('grad_method','','grad_level',1));
          [lH,dlH,ddlH] = eval(r.in_contact.position_constraints{1}.fcn,q);
          [a,b,c,Jdotv,dJdotv] = evalWithJdot(r.in_contact.position_constraints{1}.fcn,q,v);

          ddl = reshape(ddl,4,[]);
          ddlH = reshape(ddlH,4,[]);
          if norm(dl-dlH) > 1e-10
            fprintf('Wrong calculation\n')
          else
            fprintf('Good for now\n')
          end
%         end
        
      end
    end
    
    function runPassive(x0)
      r = SoftPaddleHybrid();
      v = r.constructVisualizer();
      
      if nargin < 1
        x0 = getInitialState(r);
      end
      v.drawWrapper(0,x0);
      [ytraj,xtraj] = simulate(r,[0 1],x0);
      v.playback(ytraj,struct('slider',true));
      %v.playbackAVI(ytraj,'soft_juggler_passive')
      %save('sphtraj.mat','r','v','x0','ytraj','xtraj');
      
      if (0) 
        % energy / cable length plotting 
        % note, set alpha=0 in Manipulator/computeConstraintForce to reveal
        % some artifacts, especially at the release guard when
        % the pulley effectively hits a hard-stop.  this is due to the fact
        % that phidot>0 during the in_contact phase.  it's hard to tell if
        % this is numerical artifact or a bad gradient in CableLength.
        tt=getBreaks(xtraj);
        E=tt;
        cl=tt;
        nq=getNumPositions(r.no_contact);
        dcl=zeros(length(tt),1,nq);
        ddcl=zeros(length(tt),1,nq*nq);
        for i=1:length(tt)
          x = xtraj.eval(tt(i));
%           if i >= length(tt)/20 keyboard; end
          [T,U] = energy(r,x);
          E(i)= T+U;
          if (x(1)==1) %flight mode
            [cl(i),dcl(i,1,:),ddcl(i,1,:)]=r.no_contact.position_constraints{1}.fcn.eval(x((1:nq)+1));
          else
            [cl(i),dcl(i,1,:),ddcl(i,1,:)]=r.in_contact.position_constraints{1}.fcn.eval(x((1:nq)+1));
          end
        end
        figure(1); clf;
        subplot(2,1,1); plot(tt,E, 'LineWidth', 2);
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$\mathcal{H}$', 'Interpreter', 'LaTeX', 'FontSize', 15)
        subplot(2,1,2); plot(tt,cl, 'LineWidth', 2);
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$\phi(q)$', 'Interpreter', 'LaTeX', 'FontSize', 15)
        
        figure(3); clf;
        subplot(3,1,1); plot(tt,cl, 'LineWidth', 2);
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$\phi(q)$', 'Interpreter', 'LaTeX', 'FontSize', 15)
        subplot(3,1,2); plot(tt,dcl(:,1,4), 'LineWidth', 2);
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$\dot{\phi}(q)$', 'Interpreter', 'LaTeX', 'FontSize', 15)
        subplot(3,1,3); plot(tt,ddcl(:,1,16), 'LineWidth', 2);
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$\ddot{\phi}(q)$', 'Interpreter', 'LaTeX', 'FontSize', 15)
        
        t = linspace(xtraj.tspan(1), xtraj.tspan(end), 1001);
        x = eval(xtraj, t);
        figure(2), clf
        subplot(2,2,1)
        plot(t,x(3,:), 'LineWidth', 2)
        axis('tight')
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$\theta$', 'Interpreter', 'LaTeX', 'FontSize', 15)
        subplot(2,2,2)
        plot(t,x(4,:), 'LineWidth', 2)
        axis('tight')
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$x$', 'Interpreter', 'LaTeX', 'FontSize', 15)
        subplot(2,2,3:4)
        plot(t, x(5,:), 'LineWidth', 2)
        axis('tight')
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$z$', 'Interpreter', 'LaTeX', 'FontSize', 15)
      end
    end
    function runConstraintDirCol()
      
      sph = SoftPaddleHybrid();
      %newtraj={[ConstantTrajectory(1);xtraj.traj{1}],[ConstantTrajectory(2);xtraj.traj{2}]}
      v = sph.constructVisualizer();
      tic 
      [utraj, xtraj] = sph.inContactTrajectory();
      toc
      newtraj = [ConstantTrajectory(2);xtraj]; %set mode to be 2
      newtraj = newtraj.setOutputFrame(sph.getOutputFrame);
      playback(v,newtraj,struct('slider','true'))
      tt=getBreaks(xtraj);
      T = tt(end);
      t = linspace(0,T,1001);
      x = eval(xtraj,tt);
      u = eval(utraj,tt);
      figure(1),clf
      subplot(3,2,1)
      plot(tt,x(1,:))
      axis('tight')
      ylabel('$\psi$','Interpreter','LaTeX')
      subplot(3,2,2)
      plot(tt,x(2,:))
      axis('tight')
      ylabel('$\theta$','Interpreter','LaTeX')
      subplot(3,2,3)
      plot(tt,x(3,:))
      axis('tight')
      ylabel('$x$','Interpreter','LaTeX')
      subplot(3,2,4)
      plot(tt,x(4,:))
      axis('tight')
      ylabel('$z$','Interpreter','LaTeX')
      subplot(3,2,5:6)
      plot(tt,u(:))
      axis('tight')
      ylabel('$u$','Interpreter','LaTeX')
    end
  end
end
