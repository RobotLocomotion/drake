classdef SimpleDynamicsFullKinematicsPlanner < DirectTrajectoryOptimization
  properties(SetAccess = protected)
    robot % A RigidBodyManipulator or a TimeSteppingRigidBodyManipulator
    nq % number of positions in state
    nv % number of velocities in state
    q_inds % An nq x obj.N matrix. x(q_inds(:,i)) is the posture at i'th knot
    v_inds % An nv x obj.N matrix. x(v_inds(:,i)) is the velocity at i'th knot
    qsc_weight_inds = {}
    fix_initial_state = false
    g
    % N-element vector of indices into the shared_data, where
    % shared_data{kinsol_dataind(i)} is the kinsol for knot point i
    kinsol_dataind
    
    % kinematics cache pointers, one for each knot point
    kinematics_cache_ptrs

    contact_wrench % A cell of RigidBodyContactWrench objects
    contact_wrench_active_knot  % A cell with same length as contact_wrench. contact_wrench_active_knot{i} contains all the indices of the knots that the constraint is active
    unique_contact_bodies; % An integer array. The indices of the unique contact bodies. It is in the order of obj.contact_wrench. Namely if obj.contact_wrench has body indices [3 2 4 3 1 2], theun unique_contact_bodies = [3 2 4 1]
    unique_body_contact_pts; % A length(unique_contact_bodies x 1) cell. unique_body_contact_pts{i} is a 3 x num_pts array, which contains all the contact points for unique_contact_bodies(i)
    lambda_inds % A length(unique_contact_bodies) x 1 cell. lambda_idx{i} is an N x size(unique_body_contact_pts{i},2) x N array. where N is the number of force parameters for one contact point
    lambda2contact_wrench % A length(unique_contact_bodies) x 1 cell....
                 % lambda2contact_wrench{i} is a size(unique_body_contact_pts{i},2) x obj.N matrix.
                 % lambda{i}(:,j,k) are the contact forces whose information are encoded
                 % in the RigidBodyContactWrench
                 % obj.contact_wrench(obj.lambda2contact_wrench{i}(j,k))
                 % If there is no active RigidBodyContactWrench, then the value of
                 % lambda2contact_wrench is set to be 0.
                 % Furthermore, I guarantee that in obj.lambda2contact_wrench{i},
                 % the same RigidBodyContactWrench indices are grouped together. Namely
                 % obj.lambda2contact_wrench{2}(:,1) would be [1;1;1;2;2] rather than
                 % [1;2;1;1;2];
     num_lambda_knot % An integer. The number of external force parameters at any one knot point
     robot_mass  % A double scalar.

     Q_contact_force % A 3 x 3 PSD matrix. minimize the weighted L2 norm of the contact force.
  end

  methods
    function obj = SimpleDynamicsFullKinematicsPlanner(plant,robot,N,tf_range,Q_contact_force,contact_wrench_struct,options)
      % @param plant   This is the plant going to be used in
      % DirectTRajectoryOptimization, it can be different from robot since
      % we want to use simple dynamics here. Refer to
      % DirectTrajectoryOptimization for more details.
      % @param robot   A RigidBodyManipulator or a TimeSteppingRigidBodyManipulator
      % @param N   The number of knot points
      % @param tf_range  A double. The bounds on the total time
      % @param Q_contact_force. A 3 x 3 PSD matrix. minimize the weighted L2 norm of the
      % contact force
      % @param contact_wrench_struct  A cell of of structs, with fields
      % 'active_knot' and 'cw', where 'cw' fields contain the
      % RigidBodyContactWrench objects
      if(nargin<7)
        options = struct();
      end
      if(nargin<6)
        contact_wrench_struct = [];
      end
      if(~isfield(options,'time_option'))
        options.time_option = 2;
      end
      obj = obj@DirectTrajectoryOptimization(plant,N,tf_range,struct('time_option',options.time_option));
      if(~isa(robot,'RigidBodyManipulator') && ~isa(robot,'TimeSteppingRigidBodyManipulator'))
        error('Drake:SimpleDynamicsFullKinematicsPlanner:expect a RigidBodyManipulator or a TimeSteppingRigidBodyManipulator');
      end
      if(~isnumeric(Q_contact_force))
        error('Drake:SimpleDynamicsFullKinematicsPlanner:Argument Q_contact_force should be numeric');
      end
      sizecheck(Q_contact_force,[3,3]);
      if(any(eig(Q_contact_force)<0))
        error('Drake:SimpleDynamicsFullKinematicsPlanner:Argument Q_contact_force should be a PSD matrix');
      end
      obj.Q_contact_force = Q_contact_force;
      obj.robot = robot;
      obj.nq = obj.robot.getNumPositions();
      obj.nv = obj.robot.getNumVelocities();
      obj.q_inds = obj.x_inds(1:obj.nq,:);
      obj.v_inds = obj.x_inds(obj.nq+(1:obj.nv),:);
      obj.qsc_weight_inds = cell(1,obj.N);
      obj.g = 9.81;
      % create shared data functions to calculate kinematics at the knot
      % points
      [joint_lb,joint_ub] = obj.robot.getJointLimits();
      obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(reshape(bsxfun(@times,joint_lb,ones(1,obj.N)),[],1),...
        reshape(bsxfun(@times,joint_ub,ones(1,obj.N)),[],1)),obj.q_inds(:));
      obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(zeros(obj.N-1,1),inf(obj.N-1,1)),obj.h_inds);
      kinsol_dataind = zeros(obj.N,1);
      obj.kinematics_cache_ptrs = cell(obj.N, 1);
      for i=1:obj.N,
        obj.kinematics_cache_ptrs{i} = createKinematicsCachemex(robot.mex_model_ptr, true);
        [obj,kinsol_dataind(i)] = obj.addSharedDataFunction(@(q) obj.kinematicsData(q, obj.kinematics_cache_ptrs{i}),{obj.q_inds(:,i)});
      end
      obj.kinsol_dataind = kinsol_dataind;

      num_cw = numel(contact_wrench_struct);
      obj.contact_wrench = cell(1,num_cw);
      obj.contact_wrench_active_knot = cell(1,num_cw);
      for i = 1:num_cw
        if(~isstruct(contact_wrench_struct) ||...
            ~isfield(contact_wrench_struct(i),'active_knot') || ~isfield(contact_wrench_struct(i),'cw') || ...
            ~isa(contact_wrench_struct(i).cw,'RigidBodyContactWrench'))
          error('Drake:SimpleDynamicsFullKinematicsPlanner: expect a struct containing active_knot and a RigidBodyContactWrench');
        end
        % I am supposing that a body only has one type of contact.
        obj.contact_wrench{i} = contact_wrench_struct(i).cw;
        obj.contact_wrench_active_knot{i} = contact_wrench_struct(i).active_knot;
      end
      obj.robot_mass = obj.robot.getMass();
      obj = obj.parseRigidBodyContactWrench();
      obj = obj.addDynamicConstraints();

      obj = obj.setSolverOptions('snopt','majoroptimalitytolerance',1e-5);
      obj = obj.setSolverOptions('snopt','superbasicslimit',2000);
      obj = obj.setSolverOptions('snopt','majorfeasibilitytolerance',1e-6);
      obj = obj.setSolverOptions('snopt','iterationslimit',1e5);
      obj = obj.setSolverOptions('snopt','majoriterationslimit',200);
    end

    function data = kinematicsData(obj,q,kinematics_cache_ptr)
      options.compute_gradients = true;
      options.use_mex = true;
      options.kinematics_cache_ptr_to_use = kinematics_cache_ptr;
      data = doKinematics(obj.robot,q,[],options);
    end

    function obj = setFixInitialState(obj,flag,x0)
      % set obj.fix_initial_state = flag. If flag = true, then fix the initial state to x0
      % @param x0   A 2*obj.robot.getNumPositions() x 1 double vector. x0 = [q0;qdot0]. The initial state
      sizecheck(flag,[1,1]);
      flag = logical(flag);
      if(isempty(obj.bbcon))
        obj.fix_initial_state = flag;
        if(obj.fix_initial_state)
          obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(x0,x0),obj.x_inds(:,1));
        else
          obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(-inf(obj.robot.getNumStates(),1),inf(obj.robot.getNumStates(),1)),obj.x_inds(:,1));
        end
      elseif(obj.fix_initial_state ~= flag)
        obj.fix_initial_state = flag;
        if(obj.fix_initial_state)
          obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(x0,x0),obj.x_inds(:,1));
        else
          obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(-inf(obj.robot.getNumStates(),1),inf(obj.robot.getNumStates(),1)),obj.x_inds(:,1));
        end
      end
    end


    function obj = addKinematicConstraint(obj,constraint,time_index)
      % Add a kinematic constraint that is a function of the state at the
      % specified time or times.
      % @param constraint  a CompositeConstraint
      % @param time_index   a cell array of time indices
      %   ex1., time_index = {1, 2, 3} means the constraint is applied
      %   individually to knot points 1, 2, and 3
      %   ex2,. time_index = {[1 2], [3 4]} means the constraint is applied to knot
      %   points 1 and 2 together (taking the combined state as an argument)
      %   and 3 and 4 together.
      if ~iscell(time_index)
        time_index = num2cell(reshape(time_index,1,[]));
      end
      for j=1:length(time_index),
        kinsol_inds = obj.kinsol_dataind(time_index{j});
        cnstr_inds = mat2cell(obj.q_inds(:,time_index{j}),size(obj.q_inds,1),ones(1,length(time_index{j})));

        % record constraint for posterity
        obj.constraints{end+1}.constraint = constraint;
        obj.constraints{end}.var_inds = cnstr_inds;
        obj.constraints{end}.kinsol_inds = kinsol_inds;
        obj.constraints{end}.time_index = time_index;

        obj = obj.addConstraint(constraint,cnstr_inds,kinsol_inds);
      end
    end

    function obj = addRigidBodyConstraint(obj,constraint,time_index)
      % Add a kinematic constraint that is a function of the state at the
      % specified time or times.
      % @param constraint  a RigidBodyConstraint object
      % @param time_index   a cell array of time indices
      %   ex1., time_index = {1, 2, 3} means the constraint is applied
      %   individually to knot points 1, 2, and 3
      %   ex2,. time_index = {[1 2], [3 4]} means the constraint is applied to knot
      %   points 1 and 2 together (taking the combined state as an argument)
      %   and 3 and 4 together.
      typecheck(constraint,'RigidBodyConstraint');
      if ~iscell(time_index)
        if isa(constraint,'MultipleTimeKinematicConstraint')
          % then use { time_index(1), time_index(2), ... } ,
          % aka independent constraints for each time
          time_index = {reshape(time_index,1,[])};
        else
          % then use { time_index(1), time_index(2), ... } ,
          % aka independent constraints for each time
          time_index = num2cell(reshape(time_index,1,[]));
        end
      end
      for j = 1:numel(time_index)
        if isa(constraint,'SingleTimeKinematicConstraint')
          cnstr = constraint.generateConstraint();
          obj = obj.addKinematicConstraint(cnstr{1},time_index(j));
        elseif isa(constraint, 'PostureConstraint')
          cnstr = constraint.generateConstraint();
          obj = obj.addBoundingBoxConstraint(cnstr{1}, ...
            obj.q_inds(:,time_index{j}));
        elseif isa(constraint,'QuasiStaticConstraint')
          cnstr = constraint.generateConstraint();
          if(constraint.active)
            if(~isempty(obj.qsc_weight_inds{time_index{j}}))
              error('Drake:SimpleDynamicsFullKinematicsPlanner', ...
                ['We currently support at most one ' ...
                'QuasiStaticConstraint at an individual time']);
            end
            qsc_weight_names = cell(constraint.num_pts,1);
            for k = 1:constraint.num_pts
              qsc_weight_names{k} = sprintf('qsc_weight%d',k);
            end
            obj.qsc_weight_inds{time_index{j}} = obj.num_vars+(1:constraint.num_pts)';
            obj = obj.addDecisionVariable(constraint.num_pts,qsc_weight_names);
            obj = obj.addConstraint(cnstr{1},{obj.q_inds(:,time_index{j});obj.qsc_weight_inds{time_index{j}}},obj.kinsol_dataind(time_index{j}));
            obj = obj.addConstraint(cnstr{2},obj.qsc_weight_inds{time_index{j}});
            obj = obj.addConstraint(cnstr{3},obj.qsc_weight_inds{time_index{j}});
          end
        elseif isa(constraint,'SingleTimeLinearPostureConstraint')
          cnstr = constraint.generateConstraint();
          obj = obj.addLinearConstraint(cnstr{1},obj.q_inds(:,time_index{j}));
        elseif isa(constraint,'MultipleTimeKinematicConstraint')
          cnstr = constraint.generateConstraint([],numel(time_index{j}));
          if ~isempty(cnstr)
            obj = obj.addKinematicConstraint(cnstr{1},time_index(j));
          end
        else
          id = ['Drake:SimpleDynamicsFullKinematicsPlanner:' ...
            'unknownRBConstraint'];
          msg = ['Constraints of class %s are not currently ' ...
            'supported by %s'];
          error(id,msg,class(constraint),class(obj));
        end
      end
    end

    function wrench_sol = contactWrench(obj,x)
      % Given x as the decision variables, find out the contact wrench
      % @param x   An obj.num_vars x 1 vector
      % @retval wrench_sol   An length(obj.unique_contact_bodies) x obj.N array of struct, each struct contains fiels of body,
      % body_pts, pts_pos, force and torque
      wrench_sol = struct('body',[],'body_pts',[],'pts_pos',[],'force',[],'torque',[]);
      for j = 1:obj.N
        q = x(obj.q_inds(:,j));
        kinsol = obj.robot.doKinematics(q);
        for i = 1:length(obj.unique_contact_bodies)
          wrench_sol(i,j).body = obj.unique_contact_bodies(i);
          wrench_sol(i,j).body_pts = obj.unique_body_contact_pts{i};
          wrench_sol(i,j).pts_pos = forwardKin(obj.robot,kinsol,wrench_sol(i,j).body,wrench_sol(i,j).body_pts,0);
          num_pts_i = size(obj.unique_body_contact_pts{i},2);
          lambda_ij = reshape(x(obj.lambda_inds{i}(:,:,j)),size(obj.lambda_inds{i}(:,:,j),1),num_pts_i);
          force_ij = zeros(3,num_pts_i);
          torque_ij = zeros(3,num_pts_i);
          unique_contact_wrench_ij = unique(obj.lambda2contact_wrench{i}(obj.lambda2contact_wrench{i}(:,j)~= 0,j))';
          for k = unique_contact_wrench_ij
            body_pts_ijk_idx = obj.lambda2contact_wrench{i}(:,j) == k;
            A_force = obj.contact_wrench{k}.force;
            A_torque = obj.contact_wrench{k}.torque;
            force_ij(:,body_pts_ijk_idx) = reshape(A_force*reshape(lambda_ij(:,body_pts_ijk_idx),[],1),3,[]);
            torque_ij(:,body_pts_ijk_idx) = reshape(A_torque*reshape(lambda_ij(:,body_pts_ijk_idx),[],1),3,[]);
          end
          wrench_sol(i,j).force = force_ij;
          wrench_sol(i,j).torque = torque_ij;
        end
      end
    end

  end

  methods(Access = protected)
    function obj = parseRigidBodyContactWrench(obj)
      num_contact_wrench = length(obj.contact_wrench);
      obj.unique_contact_bodies = [];
      num_pt_F_contact_bodies = []; % num_pt_F_contact_bodies(i) is the field num_pt_F of the RigidBodyContactWrench for body unique_contact_bodies(i)
      obj.unique_body_contact_pts = {};
      body_contact_type = []; % body_contact_type(i) is the type of RigidBodyContactWrench for body unique_contact_bodies(i)

      unique_body2contact_wrench = {}; % unique_body2contact_wrench{i} contains all the indices of the RigidBodyContactWrench that impose constraint on unique_contact_bodies(i)
      for i = 1:num_contact_wrench
        repeat_body = obj.contact_wrench{i}.body == obj.unique_contact_bodies;
        if(~any(repeat_body))
          obj.unique_contact_bodies(end+1) =  obj.contact_wrench{i}.body;
          num_pt_F_contact_bodies(end+1) = obj.contact_wrench{i}.num_pt_F;
          obj.unique_body_contact_pts{end+1} = obj.contact_wrench{i}.body_pts;
          body_contact_type(end+1) = obj.contact_wrench{i}.contact_force_type;
          unique_body2contact_wrench{end+1} = i;
        else
          if(sum(repeat_body)>1)
            error('Drake:SimpleDynamicsFullKinematics:parseRigidBodyContactWrench: there should be at most one existing contact body with the same body index');
          end
          if(obj.contact_wrench{i}.contact_force_type ~= body_contact_type(repeat_body))
            error('Drake:SimpleDynamicsFullKinematics:parseRigidBodyContactWrench: Currently we only support one type of RigidBodyContactWrench for one contact body');
          end
          if(obj.contact_wrench{i}.num_pt_F ~= num_pt_F_contact_bodies(repeat_body))
            error('Drake:SimpleDynamicsFullKinematics:parseRigidBodyContactWrench: The number of force parameters on the same body should be the same');
          end
          obj.unique_body_contact_pts{repeat_body} = unique([obj.unique_body_contact_pts{repeat_body} obj.contact_wrench{i}.body_pts]','rows','stable')';
          unique_body2contact_wrench{repeat_body}(end+1) = i;
        end
      end
      % set up lambda2contact_wrench
      obj.lambda2contact_wrench = cell(length(obj.unique_contact_bodies),1);
      for i = 1:length(obj.unique_contact_bodies)
        obj.lambda2contact_wrench{i} = zeros(size(obj.unique_body_contact_pts{i},2),obj.N);
        for cwc_idx = unique_body2contact_wrench{i}
          for t_idx = 1:obj.N
            if(any(obj.contact_wrench_active_knot{cwc_idx} == t_idx))
              [~,pt_idx] = intersect(obj.unique_body_contact_pts{i}',obj.contact_wrench{cwc_idx}.body_pts','rows','stable');
              if(any(obj.lambda2contact_wrench{i}(pt_idx,t_idx) ~= 0))
                pt_coord = obj.unique_body_contact_pts{i}(:,pt_idx(1));
                error('Drake:SimpleDynamicsFullKinematics:parseRigidBodyContactWrench: at knot %d, %s pt [%5.2f %5.2f %5.2f] is active for more than 1 RigidBodyContactWrench, one of which is obj.contact_wrench{%d}',...
                  t_idx,obj.contact_wrench{cwc_idx}.body_name,pt_coord(1),pt_coord(2),pt_coord(3),cwc_idx);
              end
              obj.lambda2contact_wrench{i}(pt_idx,t_idx) = cwc_idx;
            end
          end
        end
      end
      obj.num_lambda_knot = 0;
      for i = 1:length(obj.unique_contact_bodies)
        num_lambda_i = num_pt_F_contact_bodies(i)*size(obj.unique_body_contact_pts{i},2)*obj.N;
        obj.num_lambda_knot = obj.num_lambda_knot+num_lambda_i;
        obj.lambda_inds{i} = obj.num_vars+reshape(1:num_lambda_i,num_pt_F_contact_bodies(i),size(obj.unique_body_contact_pts{i},2),obj.N);
        x_name = cell(num_lambda_i,1);
        for j = 1:obj.N
          for k = 1:size(obj.unique_body_contact_pts{i},2)
            contact_wrench_idx = obj.lambda2contact_wrench{i}(k,j);
            if(contact_wrench_idx>0)
              body_name = obj.contact_wrench{contact_wrench_idx}.body_name;
              for l = 1:num_pt_F_contact_bodies(i)
                x_name{(j-1)*(size(obj.unique_body_contact_pts{i},2)*num_pt_F_contact_bodies(i))+(k-1)*num_pt_F_contact_bodies(i)+l}...
                  = sprintf('%s_pt%d_F[%d]',body_name,k,j);
              end
            else
              x_name((j-1)*(size(obj.unique_body_contact_pts{i},2)*num_pt_F_contact_bodies(i))+(k-1)*num_pt_F_contact_bodies(i)+(1:num_pt_F_contact_bodies(i)))...
                = repmat({'null contact force'},num_pt_F_contact_bodies(i),1);
            end
          end
        end
        obj = obj.addDecisionVariable(num_lambda_i,x_name);
        % Now add the constraint from RigidBodyContactWrench
        for k = 1:obj.N
          contact_wrench_idx = obj.lambda2contact_wrench{i}(:,k);
          unique_contact_wrench_idx = unique(contact_wrench_idx,'stable');
          for j = reshape(unique_contact_wrench_idx,1,[])
            lambda_idx_ijk = obj.lambda_inds{i}(:,contact_wrench_idx==j,k); % lambda_idx_jk is the indices of lambda used for i'th body, with j'th RigidBodyContactWrench, at k'th knot point
            if(j > 0)
              [lincon_wrench,nlcon_wrench,bcon_wrench,num_slack,slack_name] = obj.contact_wrench{j}.generateWrenchConstraint();
              new_slack_inds = obj.num_vars+(1:num_slack)';
              if(num_slack>0)
                obj = obj.addDecisionVariable(num_slack,slack_name);
              end
              nlcon_name = cell(nlcon_wrench.num_cnstr,1);
              for l = 1:nlcon_wrench.num_cnstr
                nlcon_name{l} = sprintf('%s[%d]',nlcon_wrench.name{l},k);
              end
              nlcon_wrench = nlcon_wrench.setName(nlcon_name);

              obj = obj.addConstraint(nlcon_wrench,[{obj.q_inds(:,k)};{reshape(lambda_idx_ijk,[],1)};{new_slack_inds}],obj.kinsol_dataind(k));
              lincon_name = cell(lincon_wrench.num_cnstr,1);
              for l = 1:lincon_wrench.num_cnstr
                lincon_name{l} = sprintf('%s[%d]',lincon_wrench.name{l},k);
              end
              lincon_wrench = lincon_wrench.setName(lincon_name);
              obj = obj.addLinearConstraint(lincon_wrench,[reshape(lambda_idx_ijk,[],1);new_slack_inds]);
              obj = obj.addBoundingBoxConstraint(bcon_wrench,[reshape(lambda_idx_ijk,[],1);new_slack_inds]);
            else
              obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(zeros(numel(lambda_idx_ijk),1),zeros(numel(lambda_idx_ijk),1)),lambda_idx_ijk);
            end
          end
        end
      end
      obj = obj.addForceNormCost();
    end
    
    function obj = addForceNormCost(obj)
      % add a quadratic cost on sum_i,j force_j[i]'*obj.Q_contact_force*force_j[i]
      A_force = cell(length(obj.contact_wrench),1);
      for i = 1:length(obj.contact_wrench)
        A_force{i} = obj.contact_wrench{i}.force;
      end
      total_num_contact_pts = 0;
      for i = 1:length(obj.unique_body_contact_pts)
        total_num_contact_pts = total_num_contact_pts+size(obj.unique_body_contact_pts{i},2);
      end
      iAfun = [];
      jAvar = [];
      Aval = [];
      active_lambda_inds = [];
      for j = 1:obj.N
        count_num_pts_j = 0;
        for i = 1:length(obj.unique_contact_bodies)
          num_pts_i = size(obj.unique_body_contact_pts{i},2);
          unique_contact_wrench_ij = unique(obj.lambda2contact_wrench{i}(obj.lambda2contact_wrench{i}(:,j)~=0,j))';
          for k = unique_contact_wrench_ij
            num_pt_F_ijk = obj.contact_wrench{k}.num_pt_F;
            body_pts_ijk_idx = find(obj.lambda2contact_wrench{i}(:,j) == k)';
            iAfun = [iAfun;reshape(bsxfun(@times,(j-1)*3*total_num_contact_pts+3*count_num_pts_j+reshape(bsxfun(@plus,[1;2;3],(body_pts_ijk_idx-1)*3),[],1),ones(1,num_pt_F_ijk*length(body_pts_ijk_idx))),[],1)];
            jAvar = [jAvar;reshape(bsxfun(@times,reshape(obj.lambda_inds{i}(:,body_pts_ijk_idx,j),1,[]),ones(3*length(body_pts_ijk_idx),1)),[],1)];
            Aval = [Aval;A_force{k}(:)];
            active_lambda_inds = [active_lambda_inds;reshape(obj.lambda_inds{i}(:,body_pts_ijk_idx,j),[],1)];
          end
          count_num_pts_j = count_num_pts_j+num_pts_i;
        end
      end
      A_force_total = sparse(iAfun,jAvar,Aval,3*total_num_contact_pts*obj.N,obj.num_vars);
      Q_diag = kron(speye(total_num_contact_pts*obj.N),obj.Q_contact_force);
      force_norm_cost = QuadraticConstraint(-inf,inf,2*A_force_total'*Q_diag*A_force_total,zeros(obj.num_vars,1));
      force_norm_cost = force_norm_cost.setSparseStructure(ones(length(active_lambda_inds),1),active_lambda_inds);
      obj = obj.addCost(force_norm_cost);
    end
    
    function obj = addSimpleDynamicConstraints(obj)
      % First I find out the order of the contact_wrench such that it is in the same
      % order of lambda
      tLeft_contact_wrench_idx = [];
      tLeft_lambda_idx = zeros(obj.num_lambda_knot,1);
      tLeft_lambda_count = 0;
      for j = 1:length(obj.unique_contact_bodies)
        tLeft_valid_pt_idx = obj.lambda2contact_wrench{j}(:,1) ~= 0;
        tLeft_lambda_idx_j_count = sum(tLeft_valid_pt_idx)*size(obj.lambda_inds{j},1);
        tLeft_lambda_idx(tLeft_lambda_count+(1:tLeft_lambda_idx_j_count))...
          = reshape(obj.lambda_inds{j}(:,tLeft_valid_pt_idx,1),[],1);
        tLeft_lambda_count = tLeft_lambda_count+tLeft_lambda_idx_j_count;
        tLeft_contact_wrench_idx = [tLeft_contact_wrench_idx unique(obj.lambda2contact_wrench{j}(tLeft_valid_pt_idx,1)','stable')];
      end
      tLeft_lambda_idx = tLeft_lambda_idx(1:tLeft_lambda_count);
      obj = obj.addContactDynamicConstraints(1,{tLeft_contact_wrench_idx},{tLeft_lambda_idx});
      for i = 2:obj.N
        tRight_contact_wrench_idx = [];
        tRight_lambda_idx = zeros(obj.num_lambda_knot,1);
        tRight_lambda_count = 0;
        for j = 1:length(obj.unique_contact_bodies)
          tRight_valid_pt_idx = obj.lambda2contact_wrench{j}(:,i) ~= 0;
          tRight_lambda_idx_j_count = sum(tRight_valid_pt_idx)*size(obj.lambda_inds{j},1);
          tRight_lambda_idx(tRight_lambda_count+(1:tRight_lambda_idx_j_count))...
            =reshape(obj.lambda_inds{j}(:,tRight_valid_pt_idx,i),[],1);
          tRight_lambda_count = tRight_lambda_count+tRight_lambda_idx_j_count;
          tRight_contact_wrench_idx = [tRight_contact_wrench_idx unique(obj.lambda2contact_wrench{j}(tRight_valid_pt_idx,i)','stable')];
        end
        tRight_lambda_idx = tRight_lambda_idx(1:tRight_lambda_count);
        obj = obj.addContactDynamicConstraints(i,{tRight_contact_wrench_idx},{tRight_lambda_idx});
        obj = obj.addContactDynamicConstraints([i-1,i],[{tLeft_contact_wrench_idx},{tRight_contact_wrench_idx}],[{tLeft_lambda_idx},{tRight_lambda_idx}]);
        tLeft_contact_wrench_idx = tRight_contact_wrench_idx;
        tLeft_lambda_idx = tRight_lambda_idx;
        tLeft_lambda_count = tRight_lambda_count;
      end
    end
  end
  
  methods(Abstract,Access = protected)
    obj = addContactDynamicConstraints(obj,num_knot,contact_wrench_idx, knot_lambda_idx)
  end
end
