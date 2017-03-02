classdef TimeSteppingRigidBodyManipulator < DrakeSystem
  % A discrete time system which simulates (an Euler approximation of) the
  % manipulator equations, with contact / limits resolved using the linear
  % complementarity problem formulation of contact in Stewart96.

  properties (Access=protected)
    manip  % the CT manipulator
    sensor % additional TimeSteppingRigidBodySensors (beyond the sensors attached to manip)
    dirty=true;
  end

  properties (SetAccess=protected)
    timestep
    twoD=false
    position_control=false;
    LCP_cache;
    enable_fastqp; % whether we use the active set LCP
    lcmgl_contact_forces_scale = 0;  % <=0 implies no lcmgl
    z_inactive_guess_tol = .01;
    multiple_contacts = false;
    gurobi_present = false;
  end

  methods
    function obj=TimeSteppingRigidBodyManipulator(manipulator_or_urdf_filename,timestep,options)
      if (nargin<3) options=struct(); end
      if ~isfield(options,'twoD') options.twoD = false; end
      
      if ispc
        error('Drake:MissingDependency:PathLCP','PathLCP is known to fail on windows.  See https://github.com/RobotLocomotion/drake/issues/569');
      end
        
      typecheck(timestep,'double');
      sizecheck(timestep,1);

      if isempty(manipulator_or_urdf_filename) || ischar(manipulator_or_urdf_filename)
        % then make the corresponding manipulator
        w = warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
        if options.twoD
          manip = PlanarRigidBodyManipulator(manipulator_or_urdf_filename,options);
        else
          manip = RigidBodyManipulator(manipulator_or_urdf_filename,options);
        end
        warning(w);
      else
        manip = manipulator_or_urdf_filename;
      end

      typecheck(manip,'RigidBodyManipulator');
      obj = obj@DrakeSystem(0,manip.getNumStates(),manip.getNumInputs(),manip.getNumOutputs(),manip.isDirectFeedthrough(),manip.isTI());
      obj.manip = manip;
      if isa(manip,'PlanarRigidBodyManipulator')
        obj.twoD = true;
      end

      if isfield(options, 'multiple_contacts')
        typecheck(options.multiple_contacts, 'logical');
        obj.multiple_contacts = options.multiple_contacts;
      end

      if ~isfield(options,'enable_fastqp')
        obj.enable_fastqp = checkDependency('fastqp');
      else
        typecheck(options.enable_fastqp,'logical');
        obj.enable_fastqp = options.enable_fastqp;
        if obj.enable_fastqp && ~checkDependency('fastqp')
          warning('Drake:TimeSteppingRigidBodyManipulator:MissingDependency','You seem to be missing fastQP. Disabling active-set LCP update.')
          obj.enable_fastqp = false;
        end
      end

      if isfield(options,'z_inactive_guess_tol')
        % you might consider setting this if the system consistently
        % gives the "ResolvingLCP" warning
        obj.z_inactive_guess_tol = options.z_inactive_guess_tol;
      end

      if isfield(options,'lcmgl_contact_forces_scale')
        obj.lcmgl_contact_forces_scale = options.lcmgl_contact_forces_scale;
        if obj.lcmgl_contact_forces_scale>0,
          checkDependency('lcmgl');
        end
      end

      obj.timestep = timestep;
      obj.LCP_cache = SharedDataHandle(struct('t',[],'x',[],'u',[],'nargout',[], ...
        'z',[],'Mqdn',[],'wqdn',[], 'possible_contact_indices',[],'possible_limit_indices',[], ...
        'dz',[],'dMqdn',[],'dwqdn',[],'contact_data',[],'fastqp_active_set',[]));

      obj = setSampleTime(obj,[timestep;0]);

      obj = compile(obj);
    end

    function checkDirty(obj)
      if (obj.dirty)
        error('You''ve changed something about this model and need to manually compile it.  Use obj=compile(obj).');
      end
    end

    function manip = getManipulator(obj)
      manip = obj.manip;
    end
    
    function setManipulator(obj,manip)
      obj.manip = manip;
    end

    function y = output(obj,t,x,u)
      checkDirty(obj);

      if ~isDirectFeedthrough(obj)
        u=[];
      end
      if isa(obj.getStateFrame(),'MultiCoordinateFrame')
        x_manip = double(Point(obj.getStateFrame(),x).inFrame(obj.manip.getStateFrame()));
      else
        x_manip = x;
      end
      y = obj.manip.output(t,x_manip,u);
      for i=1:length(obj.sensor)
        y = [y; obj.sensor{i}.output(obj,i+1,t,x,u)];
      end
    end

    function model = enableIdealizedPositionControl(model, flag)
      index = getActuatedJoints(model.manip);
      if length(unique(index))~=length(index)
        error('idealized position control currently assumes one actuator per joint');
      end
      model.position_control = logical(flag);
      model.dirty = true;
    end

    function model = compile(model)
      w = warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
      model.gurobi_present = checkDependency('gurobi');
      model.manip = model.manip.compile();
      warning(w);

      model = setNumDiscStates(model,model.manip.getNumContStates());
      model = setNumInputs(model,model.manip.getNumInputs());

      if (model.position_control)
        index = getActuatedJoints(model.manip);
        pdff = pdcontrol(model,eye(model.num_u),eye(model.num_u));
        model = setInputLimits(model,model.manip.joint_limit_min(index),model.manip.joint_limit_max(index));
        model = setInputFrame(model,getInputFrame(pdff));
      else
        model = setInputLimits(model,model.manip.umin,model.manip.umax);
        model = setInputFrame(model,getInputFrame(model.manip));
      end
      model = setStateFrame(model,getStateFrame(model.manip));

      if ~isempty(model.sensor)
        feedthrough = model.manip.isDirectFeedthrough;
        outframe{1} = getOutputFrame(model.manip);
        stateframe{1} = getStateFrame(model.manip);
        for i=1:length(model.sensor)
          model.sensor{i} = model.sensor{i}.compile(model,model.manip);
          outframe{i+1} = model.sensor{i}.constructFrame(model);
          if isa(model.sensor{i},'TimeSteppingRigidBodySensorWithState')
            stateframe{i+1} = model.sensor{i}.constructStateFrame(model);
          end
          feedthrough = feedthrough || model.sensor{i}.isDirectFeedthrough;
        end
        fr = MultiCoordinateFrame.constructFrame(outframe);
        state_fr = MultiCoordinateFrame.constructFrame(stateframe);
        if ~isequal_modulo_transforms(fr,getOutputFrame(model))
          model = setNumOutputs(model,fr.dim);
          model = setOutputFrame(model,fr);
        end
        if ~isequal_modulo_transforms(state_fr,getStateFrame(model))
          model = setNumDiscStates(model,state_fr.dim);
          model = setStateFrame(model,state_fr);
        end
        model = setDirectFeedthrough(model,feedthrough);
      else
        model = setNumOutputs(model,getNumOutputs(model.manip));
        model = setOutputFrame(model,getOutputFrame(model.manip));
        model = setDirectFeedthrough(model,model.manip.isDirectFeedthrough);
      end
      model.LCP_cache.data.t = NaN;
      model.LCP_cache.data.x = NaN(model.getNumStates(),1);
      model.LCP_cache.data.u = NaN(model.getNumInputs(),1);
      model.LCP_cache.data.nargout = NaN;
      model.dirty = false;
    end

    function x0 = getInitialState(obj)
      if ~isempty(obj.initial_state)
        x0 = obj.initial_state;
        return;
      end
      
      x0 = obj.manip.getInitialState();
      for i=1:length(obj.sensor)
        if isa(obj.sensor{i},'TimeSteppingRigidBodySensorWithState')
          x0 = [x0; obj.sensor{i}.getInitialState(obj)];
        end
      end
    end

    function [xdn,df] = update(obj,t,x,u)
      if (nargout>1)
        [obj,z,Mvn,wvn,dz,dMvn,dwvn] = solveLCP(obj,t,x,u);
      else
        [obj,z,Mvn,wvn] = solveLCP(obj,t,x,u);
      end

      num_q = obj.manip.num_positions;
      q=x(1:num_q); v=x((num_q+1):end);
      h = obj.timestep;

      if isempty(z)
        vn = wvn;
      else
        vn = Mvn*z + wvn;
      end      
      
      kinsol = obj.manip.doKinematics(q);
      vToqdot = obj.manip.vToqdot(kinsol);
      qdn = vToqdot*vn;
      qn = q+ h*qdn;
      % Find quaternion indices
      quat_bodies = obj.manip.body([obj.manip.body.floating] == 2);      
      quat_positions = [quat_bodies.position_num];
      for i=1:size(quat_positions,2)
        quat_dot = qdn(quat_positions(4:7,i));
        if norm(quat_dot) > 0 
          % Update quaternion by following geodesic
          qn(quat_positions(4:7,i)) = q(quat_positions(4:7,i)) + quat_dot/norm(quat_dot)*tan(norm(h*quat_dot));
          qn(quat_positions(4:7,i)) = qn(quat_positions(4:7,i))/norm(qn(quat_positions(4:7,i)));
        end
      end
      xdn = [qn;vn];

      if (nargout>1)  % compute gradients
        if isempty(z)
          dqdn = dwvn;
        else
          dqdn = matGradMult(dMvn,z) + Mvn*dz + dwvn;
        end
        df = [ [zeros(num_q,1), eye(num_q), zeros(num_q,num_q+obj.num_u)]+h*dqdn; dqdn ];
      end

      for i=1:length(obj.sensor)
        if isa(obj.sensor{i},'TimeSteppingRigidBodySensorWithState')
          if (nargout>1)
            [obj,xdn_sensor,df_sensor] = update(obj.sensor{i},obj,t,x,u);
          else
            [obj,xdn_sensor] = update(obj.sensor{i},obj,t,x,u);
          end
          xdn = [xdn;xdn_sensor];
          if (nargout>1)
            df = [df; df_sensor];
          end
        end
      end
    end

    function hit = cacheHit(obj,t,x,u,num_args_out)
      hit = (t==obj.LCP_cache.data.t && all(x==obj.LCP_cache.data.x) && ...
             all(u==obj.LCP_cache.data.u) && num_args_out <= obj.LCP_cache.data.nargout);
    end

    function [obj, z, Mqdn, wqdn] = solveMexLCP(obj, t, x, u)
        num_q = obj.manip.num_positions;
        q=x(1:num_q);
        v=x(num_q+(1:obj.manip.num_velocities));
        kinsol = doKinematics(obj, q, v);
        [H,C,B] = manipulatorDynamics(obj.manip, q, v);
        [phiC,normal,d,xA,xB,idxA,idxB,mu,n,D] = obj.manip.contactConstraints(kinsol, obj.multiple_contacts);
        [z, Mqdn, wqdn, possible_contact_indices, possible_jointlimit_indices] = solveLCPmex(obj.manip.mex_model_ptr, kinsol.mex_ptr, u, phiC, n, D, obj.timestep, obj.z_inactive_guess_tol, obj.LCP_cache.data.z, H, C, B, obj.enable_fastqp);
        possible_contact_indices = logical(possible_contact_indices);
        contact_data.normal = normal(:,possible_contact_indices);
        
        for i=1:length(d)
            contact_data.d{i} = d{i}(:,possible_contact_indices);
        end
        
        contact_data.xA = xA(:,possible_contact_indices);
        contact_data.xB = xB(:,possible_contact_indices);
        contact_data.idxA = idxA(possible_contact_indices);
        contact_data.idxB = idxB(possible_contact_indices);
        obj.LCP_cache.data.contact_data = contact_data;
        obj.LCP_cache.data.z = z;
        obj.LCP_cache.data.possible_limit_indices = logical(possible_jointlimit_indices)';
    end

    function [obj,z,Mvn,wvn,dz,dMvn,dwvn] = solveLCP(obj,t,x,u)
      if (nargout<5 && obj.gurobi_present && obj.manip.only_loops && obj.manip.mex_model_ptr~=0 && ~obj.position_control)
        [obj,z,Mvn,wvn] = solveMexLCP(obj,t,x,u);
        return;
      end
      
%       global active_set_fail_count
      % do LCP time-stepping
      % todo: implement some basic caching here
      if cacheHit(obj,t,x,u,nargout)
        z = obj.LCP_cache.data.z;
        Mvn = obj.LCP_cache.data.Mqdn;
        wvn = obj.LCP_cache.data.wqdn;
        if nargout > 4
          dz = obj.LCP_cache.data.dz;
          dMvn = obj.LCP_cache.data.dMqdn;
          dwvn = obj.LCP_cache.data.dwqdn;
        end
      else

        obj.LCP_cache.data.t = t;
        obj.LCP_cache.data.x = x;
        obj.LCP_cache.data.u = u;
        obj.LCP_cache.data.nargout = nargout;

        num_q = obj.manip.getNumPositions;
        num_v = obj.manip.getNumVelocities;
        q=x(1:num_q); v=x(num_q+(1:num_v));

        kinematics_options.compute_gradients = nargout > 4;
        kinsol = doKinematics(obj, q, [], kinematics_options);
        vToqdot = obj.manip.vToqdot(kinsol);
        qd = vToqdot*v;
        h = obj.timestep;

        if (nargout<5)
          [H,C,B] = manipulatorDynamics(obj.manip,q,v);
          if (obj.num_u>0 && ~obj.position_control)
            tau = B*u - C;
          else
            tau = -C;
          end
        else
          [H,C,B,dH,dC,dB] = manipulatorDynamics(obj.manip,q,v);
          if (obj.num_u>0 && ~obj.position_control)
            tau = B*u - C;
            dtau = [zeros(num_v,1), matGradMult(dB,u) - dC, B];
          else
            tau = -C;
            dtau = [zeros(num_v,1), -dC, zeros(size(B))];
          end
        end

        if (obj.position_control)
          pos_control_index = getActuatedJoints(obj.manip);
          nL = 2*length(pos_control_index);
        else
          nL = sum([obj.manip.joint_limit_min~=-inf;obj.manip.joint_limit_max~=inf]); % number of joint limits
        end
        nContactPairs = obj.manip.getNumContactPairs;
        nP = obj.manip.num_position_constraints;  % number of position constraints
        nV = obj.manip.num_velocity_constraints;
        Big = 1e20;

        % Set up the LCP:
        % z >= 0, Mz + w >= 0, z'*(Mz + w) = 0
        % for documentation below, use slack vars: s = Mz + w >= 0
        %
        % use qn = q + h*qdn
        % where H(q)*(qdn - qd)/h = B*u - C(q) + J(q)'*z
        %  or qdn = qd + H\(h*tau + J'*z)
        %  with z = [h*cL; h*cP; h*cN; h*beta{1}; ...; h*beta{mC}; lambda]
        %
        % and implement equation (7) from Anitescu97, by collecting
        %   J = [JL; JP; n; D{1}; ...; D{mC}; zeros(nC,num_q)]

        % a "possible" index is one that is close enough to contact or the
        % limit to have a chance at being active.  now we don't even
        % construct the M and w matrices for contacts/limits that seem
        % impossible to reach within this timestep.
        possible_limit_indices = [];
        possible_contact_indices = [];
        possible_indices_changed = false;

        while (1)

        if (nL > 0)
          if (obj.position_control)
            phiL = q(pos_control_index) - u;
            JL = sparse(1:obj.manip.num_u,pos_control_index,1,obj.manip.num_u,obj.manip.num_positions);
            phiL = [phiL;-phiL]; JL = [JL;-JL];
            % dJ = 0 by default, which is correct here
            dJL = zeros(length(phiL),num_q^2);
          else
            if (nargout>4)
              [phiL,JL,dJL] = obj.manip.jointLimitConstraints(q);
            else
              [phiL,JL] = obj.manip.jointLimitConstraints(q);
            end
            if isempty(possible_limit_indices)
              possible_limit_indices = (phiL + h*JL*qd) < obj.z_inactive_guess_tol;
            end
            nL = sum(possible_limit_indices);

            % phi_check and J_check are the "impossible indices"
            % which get checked at the end of the method (to make sure
            % that they did not somehow make contact or hit the limit)
            phi_check = phiL(~possible_limit_indices);
            J_check = JL(~possible_limit_indices,:);
            phiL = phiL(possible_limit_indices);
            JL = JL(possible_limit_indices,:);
            if (nargout>4)
              dJL = dJL(possible_limit_indices,:);
            end
            if isempty(obj.LCP_cache.data.possible_limit_indices) || any(obj.LCP_cache.data.possible_limit_indices~=possible_limit_indices)
              possible_indices_changed = true;
            end
            obj.LCP_cache.data.possible_limit_indices=possible_limit_indices;
          end
        else
          phi_check = zeros(0,1);
          J_check = zeros(0,num_q);
          JL = zeros(0,num_q^2);
        end
        
        has_contacts = (nContactPairs > 0);
        
        if has_contacts
          if (nargout>4)
            [phiC,normal,d,xA,xB,idxA,idxB,mu,n,D,dn,dD] = obj.manip.contactConstraints(kinsol, obj.multiple_contacts);
          else
            [phiC,normal,d,xA,xB,idxA,idxB,mu,n,D] = obj.manip.contactConstraints(kinsol, obj.multiple_contacts);
          end
          if ~isempty(phiC)
              if isempty(possible_contact_indices)
                possible_contact_indices = (phiC+h*n*qd) < obj.z_inactive_guess_tol;
              end

              nC = sum(possible_contact_indices);
              mC = length(D);

              J_size = [nL + nP + (mC+2)*nC,num_q];
              J = zeros(J_size)*q(1); % *q(1) is for taylorvar
              lb = zeros(nL+nP+(mC+2)*nC,1);
              ub = Big*ones(nL+nP+(mC+2)*nC,1);
              D = vertcat(D{:});
              % just keep the likely contacts (and store data to check the unlikely):
              phi_check = [phi_check;phiC(~possible_contact_indices)];
              J_check = [J_check; n(~possible_contact_indices,:)];
              phiC = phiC(possible_contact_indices);
              n = n(possible_contact_indices,:);
              D = D(repmat(possible_contact_indices,mC,1),:);
              mu = mu(possible_contact_indices,:);

              if isempty(obj.LCP_cache.data.possible_contact_indices) || ...
                  numel(obj.LCP_cache.data.possible_contact_indices)~= numel(possible_contact_indices) || ...
                  any(obj.LCP_cache.data.possible_contact_indices~=possible_contact_indices)
                  possible_indices_changed = true;
              end
              
              obj.LCP_cache.data.possible_contact_indices=possible_contact_indices;

              J(nL+nP+(1:nC),:) = n;
              J(nL+nP+nC+(1:mC*nC),:) = D;

              if nargout>4
                dJ = zeros(prod(J_size), num_q); % was sparse, but reshape trick for the transpose below didn't work
                possible_contact_indices_found = find(possible_contact_indices);
                n_size = [numel(possible_contact_indices), num_q];
                col_indices = 1 : num_q;
                dn = getSubMatrixGradient(reshape(dn, [], num_q), possible_contact_indices_found, col_indices, n_size);
                dJ = setSubMatrixGradient(dJ, dn, nL+nP+(1:nC), 1 : J_size(2), J_size);
                D_size = size(D);
                dD_matrix = zeros(prod(D_size), num_q);
                row_start = 0;
                for i = 1 : mC
                  dD_possible_contact = getSubMatrixGradient(dD{i}, possible_contact_indices_found, col_indices, n_size);
                  dD_matrix = setSubMatrixGradient(dD_matrix, dD_possible_contact, row_start + (1 : nC), col_indices, D_size);
                  row_start = row_start + nC;
                end 
                dD = dD_matrix;
                dJ = setSubMatrixGradient(dJ, dD, nL+nP+nC + (1 : mC * nC), col_indices, J_size);
                dJ = reshape(dJ, [], num_q^2);
              end

              contact_data.normal = normal(:,possible_contact_indices);
              for i=1:length(d)
                contact_data.d{i} = d{i}(:,possible_contact_indices);
              end
              contact_data.xA = xA(:,possible_contact_indices);
              contact_data.xB = xB(:,possible_contact_indices);
              contact_data.idxA = idxA(possible_contact_indices);
              contact_data.idxB = idxB(possible_contact_indices);
          else
              has_contacts = false;
          end
        else
            has_contacts = false;
        end
        
        if ~has_contacts
          mC=0;
          nC=0;
          J = zeros(nL+nP,num_q);
          lb = zeros(nL+nP,1);
          ub = Big*ones(nL+nP,1);
          if (nargout>4)
            dJ = zeros(nL+nP,num_q^2);
          end
          contact_data = struct();
          phi_check = zeros(0,1);
          J_check = zeros(0,num_q);
        end
        
        obj.LCP_cache.data.contact_data = contact_data;

        if (nC+nL+nP+nV==0)  % if there are no possible contacts
          z = [];
          Mvn = [];
          wvn = v + h*(H\tau);
          obj.LCP_cache.data.z = z;
          obj.LCP_cache.data.Mqdn = Mvn;
          obj.LCP_cache.data.wqdn = wvn;
          if (nargout>4)
            Hinv = inv(H);
            dz = zeros(0,1+obj.num_x+obj.num_u);
            dwvn = [zeros(num_v,1+num_q),eye(num_v),zeros(num_v,obj.num_u)] + ...
              h*Hinv*dtau - [zeros(num_v,1),h*Hinv*matGradMult(dH(:,1:num_q),Hinv*tau),zeros(num_v,num_q),zeros(num_v,obj.num_u)];
            dMvn = zeros(0,1+obj.num_x+obj.num_u);
            obj.LCP_cache.data.dz = dz;
            obj.LCP_cache.data.dMqdn = dMvn;
            obj.LCP_cache.data.dwqdn = dwvn;
          end
          return;
        end

        % now that J is allocated (since it's size is known), apply JL
        if (nL>0)
          J(1:nL,:) = JL;
          if nargout>=5, dJL(1:nL,:) = dJL; end
        end

        %% Bilateral position constraints
        if nP > 0
          % write as
          %   phiP + h*JP*qdn >= 0 && -phiP - h*JP*qdn >= 0
          if (nargout<5)
            [phiP,JP] = obj.manip.positionConstraints(q);
          else
            [phiP,JP,dJP] = obj.manip.positionConstraints(q);
          end
          J(nL+(1:nP),:) = JP;
          lb(nL+(1:nP),1) = -Big;
        end

        %% Bilateral velocity constraints
        if nV > 0
          error('not implemented yet');  % but shouldn't be hard
        end

        M = zeros(nL+nP+(mC+2)*nC)*q(1);
        w = zeros(nL+nP+(mC+2)*nC,1)*q(1);

        Hinv = inv(H);
        wvn = v + h*Hinv*tau;
        Mvn = Hinv*vToqdot'*J';

        if (nargout>4)
          dM = zeros(size(M,1),size(M,2),1+num_q+num_v+obj.num_u);
          dw = zeros(size(w,1),1+num_q+num_v+obj.num_u);
          dwvn = [zeros(num_v,1+num_q),eye(num_v),zeros(num_v,obj.num_u)] + ...
            h*Hinv*dtau - [zeros(num_v,1),h*Hinv*matGradMult(dH(:,1:num_q),Hinv*tau),zeros(num_v,num_q),zeros(num_v,obj.num_u)];
          dJtranspose = reshape(permute(reshape(dJ,size(J,1),size(J,2),[]),[2,1,3]),numel(J),[]);
          dMvn = [zeros(numel(Mvn),1),reshape(Hinv*reshape(dJtranspose - matGradMult(dH(:,1:num_q),Hinv*J'),num_q,[]),numel(Mvn),[]),zeros(numel(Mvn),num_v+obj.num_u)];
        end

        % check gradients
        %      xdn = Mqdn;
        %      if (nargout>1)
        %        df = dMqdn;
        %        df = [zeros(prod(size(xdn)),1),reshape(dJ,prod(size(xdn)),[]),zeros(prod(size(xdn)),num_q+obj.num_u)];
        %      end
        %      return;

        %% Joint Limits:
        % phiL(qn) is distance from each limit (in joint space)
        % phiL_i(qn) >= 0, cL_i >=0, phiL_i(qn) * cL_I = 0
        % z(1:nL) = cL (nL includes min AND max; 0<=nL<=2*num_q)
        % s(1:nL) = phiL(qn) approx phiL + h*JL*qdn
        if (nL > 0)
          w(1:nL) = phiL + h*JL*vToqdot*wvn;
          M(1:nL,:) = h*JL*vToqdot*Mvn;
          if (nargout>4)
            dJL = [zeros(numel(JL),1),reshape(dJL,numel(JL),[]),zeros(numel(JL),num_q+obj.num_u)];
            if (obj.position_control)
              dw(1:nL,:) = [zeros(size(JL,1),1),JL,zeros(size(JL,1),num_q),...
                [-1*ones(length(pos_control_index),obj.num_u);1*ones(length(pos_control_index),obj.num_u)]] + h*matGradMultMat(JL,wvn,dJL,dwvn);
            else
              dw(1:nL,:) = [zeros(size(JL,1),1),JL,zeros(size(JL,1),num_q+obj.num_u)] + h*matGradMultMat(JL,wvn,dJL,dwvn);
            end
            dM(1:nL,1:size(Mvn,2),:) = reshape(h*matGradMultMat(JL,Mvn,dJL,dMvn),nL,size(Mvn,2),[]);
          end
        end

        %% Bilateral Position Constraints:
        % enforcing eq7, line 2
        if (nP > 0)
          w(nL+(1:nP)) = phiP + h*JP*vToqdot*wvn;
          M(nL+(1:nP),:) = h*JP*vToqdot*Mvn;
          if (nargout>4)
            dJP = [zeros(numel(JP),1),reshape(dJP,numel(JP),[]),zeros(numel(JP),num_q+obj.num_u)];
            dw(nL+(1:nP),:) = [zeros(size(JP,1),1),JP,zeros(size(JP,1),num_q+obj.num_u)] + h*matGradMultMat(JP,wvn,dJP,dwvn);
            dM(nL+(1:nP),1:size(Mvn,2),:) = reshape(h*matGradMultMat(JP,Mvn,dJP,qMqdn),nP,size(Mvn,2),[]);
          end
        end

        %% Contact Forces:
        % s(nL+nP+(1:nC)) = phiC+h*n*qdn  (modified (fixed?) from eq7, line 3)
        % z(nL+nP+(1:nC)) = cN
        % s(nL+nP+nC+(1:mC*nC)) = repmat(lambda,mC,1) + D*qdn  (eq7, line 4)
        % z(nL+nP+nC+(1:mC*nC)) = [beta_1;...;beta_mC]
        % s(nL+nP+(mC+1)*nC+(1:nC)) = mu*cn - sum_mC beta_mC (eq7, line 5)
        % z(nL+nP+(mC+1)*nC+(1:nC)) = lambda
        %
        % The second set of conditions gives:
        %   lambda_i >= the largest projection of the velocity vector
        %   onto the d vectors (since lambda_i >= -(D*qdn)_i for all i,
        % and by construction of d always having the mirror vectors,
        %   lambda_i >= (D_qdn)_i
        %
        % The last eqs give
        %  lambda_i > 0 iff (sum beta)_i = mu_i*cn_i
        % where i is for the ith contact.
        % Assume for a moment that mu_i*cn_i = 1, then (sum_beta)_i = 1
        % is a constraint ensuring that sum_beta_j D_j is a convex
        % combination of the D vectors (since beta_j is also > 0)
        % So lambda_i >0 if forces for the ith contact are on the boundary of
        % the friction cone (lambda_i could also be > 0 if some of the beta_j
        % D_j's are internally canceling each other out)
        %
        % So one solution is
        %  v_i = 0,
        %  beta_i >= 0
        %  lambda_i = 0,
        %  sum_d beta_i < mu*cn_i
        % and another solution is
        %  v_i > 0  (sliding)
        %  lambda_i = max_d (v_i)
        %  beta_i = mu*cn_i only in the direction of the largest negative velocity
        %  beta_i = 0 otherwise
        % By virtue of the eqs of motion connecting v_i and beta_i, only one
        % of these two can exist. (the first is actually many solutions, with
        % beta_i in opposite directions canceling each other out).
        if (nC > 0)
          w(nL+nP+(1:nC)) = phiC+h*n*vToqdot*wvn;
          M(nL+nP+(1:nC),:) = h*n*vToqdot*Mvn;

          w(nL+nP+nC+(1:mC*nC)) = D*vToqdot*wvn;
          M(nL+nP+nC+(1:mC*nC),:) = D*vToqdot*Mvn;
          M(nL+nP+nC+(1:mC*nC),nL+nP+(1+mC)*nC+(1:nC)) = repmat(eye(nC),mC,1);

          M(nL+nP+(mC+1)*nC+(1:nC),nL+nP+(1:(mC+1)*nC)) = [diag(mu), repmat(-eye(nC),1,mC)];

          if (nargout>4)
            % n, dn, and dD were only w/ respect to q.  filled them out for [t,x,u]
            dn = [zeros(size(dn,1),1),dn,zeros(size(dn,1),num_q+obj.num_u)];
            dD = [zeros(numel(D),1),reshape(dD,numel(D),[]),zeros(numel(D),num_q+obj.num_u)];

            dw(nL+nP+(1:nC),:) = [zeros(size(n,1),1),n,zeros(size(n,1),num_q+obj.num_u)]+h*matGradMultMat(n,wvn,dn,dwvn);
            dM(nL+nP+(1:nC),1:size(Mvn,2),:) = reshape(h*matGradMultMat(n,Mvn,dn,dMvn),nC,size(Mvn,2),[]);

            dw(nL+nP+nC+(1:mC*nC),:) = matGradMultMat(D,wvn,dD,dwvn);
            dM(nL+nP+nC+(1:mC*nC),1:size(Mvn,2),:) = reshape(matGradMultMat(D,Mvn,dD,dMvn),mC*nC,size(Mvn,2),[]);
          end
        end

        QP_FAILED = true;
        if ~possible_indices_changed && obj.enable_fastqp
          z = zeros(nL+nP+(mC+2)*nC,1);
          if isempty(obj.LCP_cache.data.z) || numel(obj.LCP_cache.data.z) ~= numel(lb)
            z_inactive = true(nL+nP+(mC+2)*nC,1);
            obj.LCP_cache.data.z = z;
          else
            z_inactive = obj.LCP_cache.data.z>lb+1e-8;
            % use conservative guess of M_active to avoid occasional numerical issues
            % when M*z_inactive + w > 1e-8 by a small amount
          end
          n_z_inactive = sum(z_inactive);
          if n_z_inactive > 0
            Aeq = M(z_inactive,z_inactive);
            beq = -w(z_inactive);
            Ain_fqp = -eye(n_z_inactive);
            bin_fqp = -lb(z_inactive);
            QblkDiag = {eye(n_z_inactive)};
            fqp = zeros(n_z_inactive, 1);
            [z_,info_fqp] = fastQPmex(QblkDiag,fqp,Ain_fqp,bin_fqp,Aeq,beq,[]);
            QP_FAILED = info_fqp<0;
            if ~QP_FAILED
              z(z_inactive) = z_;
              obj.LCP_cache.data.fastqp_active_set = find(abs(Ain_fqp*z_ - bin_fqp)<1e-6);
              % we know:
              % z(z_inactive) >= lb
              % M(M_active, z_inactive)*z(z_inactive) + w(M_active) = 0
              % M(M_inactive, z_inactive)*z(z_inactive) + w(M_inactive) >= 0
              %
              % check:
              % z(z_inactive)'*(M(z_inactive, z_inactive)*z(z_inactive) + w(z_inactive)) == 0  % since it is not checked by QP
              % M(z_active,z_inactive)*z(z_inactive)+w(z_active) >= 0  % since we're assuming that z(z_active) == 0
              z_active = ~z_inactive(1:(nL+nP+nC));  % only look at joint limit, position, and normal variables since if cn_i = 0,
              % then that's a solution and we don't care about the relative velocity \lambda_i

              QP_FAILED = (~isempty(w(z_active)) && any(M(z_active,z_inactive)*z(z_inactive)+w(z_active) < 0)) || ...
                  any(((z(z_inactive)>lb(z_inactive)+1e-8) & (M(z_inactive, z_inactive)*z(z_inactive)+w(z_inactive)>1e-8))) || ...
                  any(abs(z_'*(Aeq*z_ - beq)) > 1e-6);
            end
          end
        end

        if QP_FAILED
            % then the active set has changed, call pathlcp
            z = pathlcp(M,w,lb,ub);
            obj.LCP_cache.data.fastqp_active_set = [];
        end
        % for debugging
        %cN = z(nL+nP+(1:nC))
        %beta1 = z(nL+nP+nC+(1:nC))
        %beta2 = z(nL+nP+2*nC+(1:nC))
        %lambda = z(nL+nP+3*nC+(1:nC))
        % end debugging
        % more debugging
%        path_convergence_tolerance = 1e-6; % default according to http://pages.cs.wisc.edu/~ferris/path/options.pdf
%        assert(all(z>=0));
%        assert(all(M*z+w>=-path_convergence_tolerance));
%        valuecheck(z'*(M*z+w),0,path_convergence_tolerance);
        % end more debugging

        if obj.lcmgl_contact_forces_scale>0
          cN = z(nL+nP+(1:nC));
          cartesian_force = repmat(cN',3,1).*contact_data.normal;
          for i=1:mC/2  % d is mirrored in contactConstraints
            beta = z(nL+nP+i*nC+(1:nC));
            cartesian_force = cartesian_force + repmat(beta',3,1).*contact_data.d{i};
            beta = z(nL+nP+(mC/2+i)*nC+(1:nC));
            cartesian_force = cartesian_force - repmat(beta',3,1).*contact_data.d{i};
          end

          lcmgl = drake.matlab.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton,'LCP contact forces');
          for j=1:nC
            point = forwardKin(obj.manip,kinsol,contact_data.idxA(j),contact_data.xA(:,j));
            lcmgl.glColor3f(.4,.2,.4);
            lcmgl.drawVector3d(point,-obj.lcmgl_contact_forces_scale*cartesian_force(:,j));
            lcmgl.glColor3f(.2,.4,.2);
            lcmgl.drawVector3d(point,obj.lcmgl_contact_forces_scale*cartesian_force(:,j));
          end
          lcmgl.switchBuffers();
        end

        obj.LCP_cache.data.z = z;
        obj.LCP_cache.data.Mqdn = Mvn;
        obj.LCP_cache.data.wqdn = wvn;

        if (nargout>4)
          % Quick derivation:
          % The LCP solves for z given that:
          % M(a)*z + q(a) >= 0
          % z >= 0
          % z'*(M(a)*z + q(a)) = 0
          % where the vector inequalities are element-wise, and 'a' is a vector of  parameters (here the state x and control input u).
          %
          % Our goal is to solve for the gradients dz/da.
          %
          % First we solve the LCP to obtain z.
          %
          % Then, for all i where z_i = 0, then dz_i / da = 0.
          % Call the remaining active constraints (where z_i >0)  Mbar(a), zbar, and  qbar(a).  then we have
          % Mbar(a) * zbar + qbar(a) = 0
          %
          % and the remaining gradients are given by
          % for all j, dMbar/da_j * zbar + Mbar * dzbar / da_j + dqbar / da_j = 0
          % or
          %
          % dzbar / da_j =  -pinv(Mbar)*(dMbar/da_j * zbar + dqbar / da_j)
          %
          % Note that there may be multiple solutions to the above equation
          %    so we use the pseudoinverse to select the min norm solution

          dz = zeros(size(z,1),1+obj.num_x+obj.num_u);
          zposind = find(z>0);
          if ~isempty(zposind)
            Mbar = M(zposind,zposind);
            dMbar = reshape(dM(zposind,zposind,:),numel(Mbar),[]);
            zbar = z(zposind);
            dwbar = dw(zposind,:);
            dz(zposind,:) = -pinv(Mbar)*(matGradMult(dMbar,zbar) + dwbar);
          end
          obj.LCP_cache.data.dz = dz;
          obj.LCP_cache.data.dMqdn = dMvn;
          obj.LCP_cache.data.dwqdn = dwvn;
        else
          obj.LCP_cache.data.dz = [];
          obj.LCP_cache.data.dMqdn = [];
          obj.LCP_cache.data.dwqdn = [];
        end

        penetration = phi_check + h*J_check*vToqdot*(Mvn*z + wvn) < 0;
        if any(penetration)
          % then add the constraint and run the entire loop again!
          limits = sum(~possible_limit_indices);
          possible_limit_indices(~possible_limit_indices) = penetration(1:limits);
          possible_contact_indices(~possible_contact_indices) = penetration(limits+1:end);
          obj.warning_manager.warnOnce('Drake:TimeSteppingRigidBodyManipulator:ResolvingLCP','This timestep violated our assumptions about which contacts could possibly become active in one timestep.  Consider reducing your dt.  If it seems to happen a lot, then please let us know about it.');
        else
          break;
        end

        end
      end
    end

    function obj = addSensor(obj,sensor)
      if isa(sensor,'RigidBodySensor')
        obj.manip = obj.manip.addSensor(sensor);
      else
        typecheck(sensor,'TimeSteppingRigidBodySensor');
        obj.sensor{end+1} = sensor;
      end
    end

    function [obj,frame_id] = addFrame(obj,frame)
      [obj.manip,frame_id] = obj.manip.addFrame(frame);
    end



    function varargout = pdcontrol(sys,Kp,Kd,index)
      if nargin<4, index=[]; end
      [pdff,pdfb] = pdcontrol(sys.manip,Kp,Kd,index);
      pdfb = setInputFrame(pdfb,sys.manip.getStateFrame());
      pdfb = setOutputFrame(pdfb,sys.getInputFrame());
      pdff = setOutputFrame(pdff,sys.getInputFrame());
      if nargout>1
        varargout{1} = pdff;
        varargout{2} = pdfb;
      else
        % note: design the PD controller with the (non time-stepping
        % manipulator), but build the closed loop system with the
        % time-stepping manipulator:
        varargout{1} = cascade(pdff,feedback(sys,pdfb));
      end
    end

  end

  methods  % pass through methods (to the manipulator)
    function B = getB(obj)
      B = getB(obj.manip);
    end

    function g = getGravity(obj)
      g = getGravity(obj.manip);
    end

    function num_q = getNumPositions(obj)
      num_q = obj.manip.num_positions;
    end

    function num_v = getNumVelocities(obj)
      num_v = obj.manip.getNumVelocities();
    end

    function obj = setStateFrame(obj,fr)
      obj = setStateFrame@DrakeSystem(obj,fr);

      % make sure there is a transform defined to and from the
      % manipulator state frame.  (the trivial transform is the correct
      % one)
      if ~isempty(obj.manip) % this also gets called on the initial constructor
        mfr = getStateFrame(obj.manip);
        if isempty(findTransform(fr,mfr))
          addTransform(fr,AffineTransform(fr,mfr,eye(obj.manip.num_x,obj.num_x),zeros(obj.manip.num_x,1)));
        end
        if isempty(findTransform(mfr,fr))
          addTransform(mfr,AffineTransform(mfr,fr,eye(obj.num_x,obj.manip.num_x),zeros(obj.num_x,1)));
        end
      end
    end

    function obj = setTerrain(obj,varargin)
      obj.manip = setTerrain(obj.manip,varargin{:});
    end

    function terrain = getTerrain(obj)
      terrain = obj.manip.terrain;
    end

    function varargout = getTerrainHeight(obj,varargin)
      varargout = cell(1,nargout);
      [varargout{:}] = getTerrainHeight(obj.manip,varargin{:});
    end

    function obj = setJointLimits(obj,varargin)
      obj.manip = setJointLimits(obj.manip,varargin{:});
    end

    function obj=addRobotFromURDF(obj,varargin)
      if obj.twoD
        w = warning('off','Drake:PlanarRigidBodyManipulator:UnsupportedContactPoints');
        warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
      else
        w = warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
      end
      obj.manip=addRobotFromURDF(obj.manip,varargin{:});
      obj=compile(obj);  % note: compiles the manip twice, but it's ok.
      warning(w);
    end

    function obj=addRobotFromSDF(obj,varargin)
      obj.manip=addRobotFromSDF(obj.manip,varargin{:});
      obj=compile(obj);  % note: compiles the manip twice, but it's ok.
    end

    function varargout = doKinematics(obj,varargin)
      varargout = cell(1,nargout);
      [varargout{:}]=doKinematics(obj.manip,varargin{:});
    end

    function varargout = forwardKin(obj,varargin)
      varargout = cell(1,nargout);
      [varargout{:}]=forwardKin(obj.manip,varargin{:});
    end

    function varargout = bodyKin(obj,varargin)
      varargout = cell(1,nargout);
      [varargout{:}]=bodyKin(obj.manip,varargin{:});
    end

    function varargout = approximateIK(obj,varargin)
      varargout = cell(1,nargout);
      [varargout{:}]=approximateIK(obj.manip,varargin{:});
    end

    function varargout = inverseKin(obj,varargin)
      varargout = cell(1,nargout);
      [varargout{:}]=inverseKin(obj.manip,varargin{:});
    end

    function varargout = inverseKinPointwise(obj,varargin)
      varargout = cell(1,nargout);
      [varargout{:}] = inverseKinPointwise(obj.manip,varargin{:});
    end

    function varargout = inverseKinTraj(obj,varargin)
        varargout = cell(1,nargout);
        [varargout{:}] = inverseKinTraj(obj.manip,varargin{:});
    end

    function varargout = inverseKinWrapup(obj,varargin)
        varargout = cell(1,nargout);
        [varargout{:}] = inverseKinWrapup(obj.manip,varargin{:});
    end

    function varargout = findFixedPoint(obj,x0,varargin)
      varargout=cell(1,nargout);
      if isnumeric(x0)
        x0 = Point(obj.getStateFrame(),x0);
      end
      [varargout{:}]=findFixedPoint(obj.manip,x0,varargin{:});
      varargout{1} = varargout{1}.inFrame(obj.getStateFrame());
    end

    function varargout = collisionDetect(obj,varargin)
      varargout = cell(1,nargout);
      [varargout{:}]=collisionDetect(obj.manip,varargin{:});
    end

    function varargout = collisionDetectTerrain(obj,varargin)
      varargout = cell(1,nargout);
      [varargout{:}]=collisionDetectTerrain(obj.manip,varargin{:});
    end

    function [obj,id] = addStateConstraint(obj,con)
      % keep two copies of the constraints around ... :(
      % todo: re-evaluate whether that is really necessary
      [obj,id] = addStateConstraint@DrakeSystem(obj,con);
      [obj.manip,manip_id] = obj.manip.addStateConstraint(obj,con);
      assert(id==manip_id);
    end

    function obj = updateStateConstraint(obj,id,con)
      obj = updateStateConstraint@DrakeSystem(obj,id,con);
      obj.manip = updateStateConstraint(obj.manip,id,con);
    end

    function obj = removeAllStateConstraints(obj)
      obj = removeAllStateConstraints@DrakeSystem(obj);
      obj.manip = removeAllStateConstraints(obj.manip);
    end

    function varargout = positionConstraints(obj,varargin)
      varargout = cell(1,nargout);
      [varargout{:}] = positionConstraints(obj.manip,varargin{:});
    end

    function varargout = velocityConstraints(obj,varargin)
      varargout = cell(1,nargout);
      [varargout{:}] = velocityConstraints(obj.manip,varargin{:});
    end

    function varargout = manipulatorDynamics(obj,varargin)
      varargout = cell(1,nargout);
      [varargout{:}] = manipulatorDynamics(obj.manip,varargin{:});
    end

    function varargout = contactConstraints(obj,varargin)
      varargout=cell(1,nargout);
      [varargout{:}] = contactConstraints(obj.manip,varargin{:});
    end

    function varargout = contactConstraintsBV(obj,varargin)
      varargout=cell(1,nargout);
      [varargout{:}] = contactConstraintsBV(obj.manip,varargin{:});
    end

    function varargout = pairwiseContactConstraints(obj,varargin)
      varargout=cell(1,nargout);
      [varargout{:}] = pairwiseContactConstraints(obj.manip,varargin{:});
    end

    function varargout = pairwiseContactConstraintsBV(obj,varargin)
      varargout=cell(1,nargout);
      [varargout{:}] = pairwiseContactConstraintsBV(obj.manip,varargin{:});
    end

    function varargout = resolveConstraints(obj,x0,varargin)
      varargout=cell(1,nargout);
      [varargout{:}] = resolveConstraints(obj.manip,x0,varargin{:});
      varargout{1} = varargout{1}.inFrame(obj.getStateFrame());
    end

    function varargout = getMass(obj,varargin)
      varargout=cell(1,nargout);
      [varargout{:}] = getMass(obj.manip,varargin{:});
    end

    function varargout = getCOM(obj,varargin)
      varargout=cell(1,nargout);
      [varargout{:}] = getCOM(obj.manip,varargin{:});
    end

    function varargout = centerOfMassJacobianDotTimesV(obj,varargin)
      varargout = cell(1,nargout);
      [varargout{:}] = centerOfMassJacobianDotTimesV(obj.manip,varargin{:});
    end

    function varargout = centroidalMomentumMatrixDotTimesV(obj,varargin)
      varargout=cell(1,nargout);
      [varargout{:}] = centroidalMomentumMatrixDotTimesV(obj.manip,varargin{:});
    end

    function varargout = centroidalMomentumMatrix(obj,varargin)
      varargout=cell(1,nargout);
      [varargout{:}] = centroidalMomentumMatrix(obj.manip,varargin{:});
    end

    function varargout = parseBodyOrFrameID(obj,varargin)
      varargout=cell(1,nargout);
      [varargout{:}] = parseBodyOrFrameID(obj.manip,varargin{:});
    end

    function joint_ind = findJointId(model,varargin)
      joint_ind = findJointId(model.manip,varargin{:});
    end

    function body_ind = findLinkId(model,varargin)
      body_ind = findLinkId(model.manip,varargin{:});
    end

    function indices = findPositionIndices(model, varargin)
      indices = findPositionIndices(model.manip,varargin{:});
    end

    function body = findLink(model,varargin)
      body = findLink(model.manip,varargin{:});
    end

    function frame_id = findFrameId(model,varargin)
      frame_id = findFrameId(model.manip,varargin{:});
    end

    function ancestor_bodies = findAncestorBodies(obj, body_index)
      ancestor_bodies = obj.manip.findAncestorBodies(body_index);
    end

    function [body_path, joint_path, signs] = findKinematicPath(obj, start_body, end_body)
      [body_path, joint_path, signs] = obj.manip.findKinematicPath(start_body, end_body);
    end

    function obj = weldJoint(obj,body_ind_or_joint_name,robot)
      if nargin>2
        obj.manip = weldJoint(obj.manip,body_ind_or_joint_name,robot);
      else
        obj.manip = weldJoint(obj.manip,body_ind_or_joint_name);
      end
      obj.dirty = true;
    end

    function body = getBody(model,varargin)
      body = getBody(model.manip,varargin{:});
    end

    function frame = getFrame(model,varargin)
      frame = getFrame(model.manip,varargin{:});
    end

    function str = getBodyOrFrameName(obj,varargin)
      str = obj.manip.getBodyOrFrameName(varargin{:});
    end

    function model = setBody(model,varargin)
      model.manip = setBody(model.manip,varargin{:});
      model.dirty = true;
    end

    function v = constructVisualizer(obj,varargin)
      v = constructVisualizer(obj.manip,varargin{:});
    end

    function getNumContacts(~)
      error('getNumContacts is no longer supported, in anticipation of alowing multiple contacts per body pair. Use getNumContactPairs for cases where the number of contacts is fixed');
    end

    function n=getNumContactPairs(obj)
      n = obj.manip.getNumContactPairs;
    end

    function c = getBodyContacts(obj,body_idx)
      c = obj.manip.body(body_idx).collision_geometry;
    end

    function addContactShapeToBody(varargin)
      errorDeprecatedFunction('addCollisionGeometryToBody');
    end

    function obj = addCollisionGeometryToBody(obj,varargin)
      obj.manip = addCollisionGeometryToBody(obj.manip,varargin{:});
    end

    function addVisualShapeToBody(varargin)
      errorDeprecatedFunction('addVisualGeometryToBody');
    end

    function obj = addVisualGeometryToBody(obj,varargin)
      obj.manip = addVisualGeometryToBody(obj.manip,varargin{:});
    end

    function addShapeToBody(varargin)
      errorDeprecatedFunction('addGeometryToBody');
    end

    function obj = addGeometryToBody(obj,varargin)
      obj.manip = addGeometryToBody(obj.manip,varargin{:});
    end

    function replaceContactShapesWithCHull(varargin)
      errorDeprecatedFunction('replaceCollisionGeometryWithConvexHull');
    end

    function obj = replaceCollisionGeometryWithConvexHull(obj,body_indices,varargin)
      obj.manip = replaceCollisionGeometryWithConvexHull(obj.manip,body_indices,varargin{:});
    end

    function getContactShapeGroupNames(varargin)
      errorDeprecatedFunction('getCollisionGeometryGroupNames');
    end

    function groups = getCollisionGeometryGroupNames(obj)
      groups = getCollisionGeometryGroupNames(obj.manip);
    end

    function f_friction = computeFrictionForce(obj,qd)
      f_friction = computeFrictionForce(obj.manip,qd);
    end

    function obj = removeCollisionGroups(obj,contact_groups)
      obj.manip = removeCollisionGroups(obj.manip,contact_groups);
    end

    function obj = removeCollisionGroupsExcept(obj,varargin)
      obj.manip = removeCollisionGroupsExcept(obj.manip,varargin{:});
    end

    function str = getLinkName(obj,body_ind)
      str = obj.manip.getLinkName(body_ind);
    end

    function link_names = getLinkNames(obj)
      link_names =  {obj.manip.body.linkname}';
    end

    function joint_names = getJointNames(obj)
      joint_names =  {obj.manip.body.jointname}';
    end

    function num_bodies = getNumBodies(obj)
      num_bodies = length(obj.manip.body);
    end

    function [jl_min, jl_max] = getJointLimits(obj)
      jl_min = obj.manip.joint_limit_min;
      jl_max = obj.manip.joint_limit_max;
    end

    function varargout = jointLimitConstraints(obj,varargin)
      varargout=cell(1,nargout);
      [varargout{:}] = jointLimitConstraints(obj.manip,varargin{:});
    end

    function index = getActuatedJoints(obj)
      index = getActuatedJoints(obj.manip);
    end

    function ptr = getMexModelPtr(obj)
      ptr = getMexModelPtr(obj.manip);
    end

    function [phi,Jphi] = closestDistance(obj,varargin)
      [phi,Jphi] = closestDistance(obj.manip,varargin{:});
    end

    function obj = addLinksToCollisionFilterGroup(obj,linknames,collision_fg_name,robotnums)
      obj.manip = addLinksToCollisionFilterGroup(obj.manip,linknames,collision_fg_name,robotnums);
    end

    function out = name(obj)
      out = obj.manip.name;
    end

    function fr = getParamFrame(obj)
      fr = getParamFrame(obj.manip);
    end

    function p = getParams(obj)
      p = obj.manip.getParams;
    end
    
    function [pmin,pmax] = getParamLimits(obj)
      % Returns the current lower and upper bounds on the system parameters
      [pmin,pmax] = getParamLimits(obj.manip);
    end
    
    function obj = setParams(obj,p)
      obj.manip = setParams(obj.manip,p);
    end
    
    function [phat,simulation_error,estimated_delay,exitflag] = ...
        parameterEstimation(obj,data,varargin)
      [phat,simulation_error,estimated_delay,exitflag] = ...
        obj.manip.parameterEstimation(data,varargin);
    end
    
    function terrain_contact_point_struct = getTerrainContactPoints(obj,varargin)
      terrain_contact_point_struct = getTerrainContactPoints(obj.manip,varargin{:});
    end

    function varargout = terrainContactPositions(obj,varargin)
      varargout = cell(1,nargout);
      [varargout{:}] = terrainContactPositions(obj.manip,varargin{:});
    end

    function varargout = terrainContactJacobianDotTimesV(obj,varargin)
      varargout = cell(1,nargout);
      [varargout{:}] = terrainContactJacobianDotTimesV(obj.manip,varargin{:});
    end

    function distance = collisionRaycast(obj, kinsol, origin, point_on_ray, use_margins)
      if nargin < 5
        use_margins = true;
      end
      distance = collisionRaycast(obj.manip, kinsol, origin, point_on_ray, use_margins);
    end


  end


end
