classdef Atlas < TimeSteppingRigidBodyManipulator
  
  methods
    
    function obj=Atlas(urdf,options)
      typecheck(urdf,'char');

      if nargin < 2
        options = struct();
      end
      if ~isfield(options,'dt')
        options.dt = 0.001;
      end
      if ~isfield(options,'floating')
        options.floating = true;
      end
      
      addpath(fullfile(getDrakePath,'examples','Atlas','frames'));
  
      w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
      obj = obj@TimeSteppingRigidBodyManipulator(urdf,options.dt,options);
      warning(w);
      
      if options.floating
        % could also do fixed point search here
        obj = obj.setInitialState(obj.resolveConstraints(zeros(obj.getNumStates(),1)));
      else
        % TEMP HACK to get by resolveConstraints
        for i=1:length(obj.manip.body), obj.manip.body(i).contact_pts=[]; end
        obj.manip = compile(obj.manip);
        obj = obj.setInitialState(zeros(obj.getNumStates(),1));
      end
    end
    
    function obj = compile(obj)
      obj = compile@TimeSteppingRigidBodyManipulator(obj);

      atlas_state_frame = AtlasState(obj);
      tsmanip_state_frame = obj.getStateFrame();
      if isa(tsmanip_state_frame,'MultiCoordinateFrame')
        tsmanip_state_frame.frame{1} = atlas_state_frame;
        state_frame = tsmanip_state_frame;
      else
        state_frame = atlas_state_frame;
      end
      obj.manip = obj.manip.setStateFrame(atlas_state_frame);
      obj.manip = obj.manip.setOutputFrame(atlas_state_frame);
      obj = obj.setStateFrame(state_frame);
      obj = obj.setOutputFrame(state_frame);
    
      input_frame = AtlasInput(obj);
      obj = obj.setInputFrame(input_frame);
      obj.manip = obj.manip.setInputFrame(input_frame);
    end

    function obj = setInitialState(obj,x0)
      if isa(x0,'Point')
        obj.x0 = double(x0); %.inFrame(obj.getStateFrame));
      else
        typecheck(x0,'double');
        sizecheck(x0,obj.getNumStates());
        obj.x0 = x0;
      end
    end
    
    function x0 = getInitialState(obj)
      x0 = obj.x0;
    end    
    
    function [xstar,ustar,zstar] = getFixedPoint(obj,options)
      if nargin < 2 || ~isfield(options,'visualize')
        options.visualize = false;
      end
      
      x0 = Point(obj.getStateFrame());
      x0 = resolveConstraints(obj,x0);
      u0 = zeros(obj.getNumInputs(),1);

      nq = obj.getNumDOF();
      nu = obj.getNumInputs();
      nz = obj.getNumContacts()*3;
      z0 = zeros(nz,1);
      q0 = x0(1:nq);
    
      problem.x0 = [q0;u0;z0];
      problem.objective = @(quz) 0; % feasibility problem
      problem.nonlcon = @(quz) mycon(quz);
      problem.solver = 'fmincon';

      if options.visualize
        v = obj.constructVisualizer;
        %problem.options=optimset('DerivativeCheck','on','GradConstr','on','Algorithm','interior-point','Display','iter','OutputFcn',@drawme,'TolX',1e-14,'MaxFunEvals',5000);
        problem.options=optimset('GradConstr','on','Algorithm','interior-point','Display','iter','OutputFcn',@drawme,'TolX',1e-14,'MaxFunEvals',5000);
      else
        problem.options=optimset('GradConstr','on','Algorithm','interior-point','TolX',1e-14,'MaxFunEvals',5000);
      end
      
      lb_z = -1e6*ones(nz,1);
      lb_z(3:3:end) = 0; % normal forces must be >=0
      ub_z = 1e6*ones(nz,1);
    
      [jl_min,jl_max] = obj.getJointLimits();
      % force search to be close to starting position
      problem.lb = [max(q0-0.05,jl_min+0.01); obj.umin; lb_z];
      problem.ub = [min(q0+0.05,jl_max-0.01); obj.umax; ub_z];
      %problem.lb(2) = 0.0; % body z

      [quz_sol,~,exitflag] = fmincon(problem);
      success=(exitflag==1);
      xstar = [quz_sol(1:nq); zeros(nq,1)];
      ustar = quz_sol(nq+(1:nu));
      zstar = quz_sol(nq+nu+(1:nz));
      if (~success)
        error('failed to find fixed point');
      end

      function stop=drawme(quz,optimValues,state)
        stop=false;
        v.draw(0,[quz(1:nq); zeros(nq,1)]);
      end

      function [c,ceq,GC,GCeq] = mycon(quz)
        q=quz(1:nq);
        u=quz(nq+(1:nu));
        z=quz(nq+nu+(1:nz));

        [~,C,B,~,dC,~] = obj.manip.manipulatorDynamics(q,zeros(nq,1));
        [phiC,JC] = obj.contactConstraints(q);
        [~,J,dJ] = obj.contactPositions(q);
        
        % ignore friction constraints for now
        c = 0;
        GC = zeros(nq+nu+nz,1); 
        
        dJz = zeros(nq,nq);
        for i=1:nq
            dJz(:,i) = dJ(:,(i-1)*nq+1:i*nq)'*z;
        end
        
        ceq = [C-B*u-J'*z; phiC];
        GCeq = [[dC(1:nq,1:nq)-dJz,-B,-J']',[JC'; zeros(nu+nz,length(phiC))]]; 
      end
    end
      
  end
  
  properties (SetAccess = protected, GetAccess = public)
    x0
  end
end
