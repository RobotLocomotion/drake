

classdef KinematicTrajectoryOptimization
  % Abstract class for kinematic planning
  properties (SetAccess = protected)
    nq
    q_inds
    kinsol_dataind
    qsc_weight_inds = {}
    robot
  end
  
  methods (Abstract)
    obj = addConstraint(cnstr,xind)
    obj = addSharedDataFunction(fun,xind)
    obj = addDecisionVariable(num_var,names)
    N = getN(obj)
    x_inds = getXinds(obj)
    h_inds = getHinds(obj)
  end
  
  methods
    function obj = KinematicTrajectoryOptimization(robot,initialize_flag)
      
      if nargin < 2
        initialize_flag = 1;
      end
      
      typecheck(robot,{'KinematicDummyPlant','RigidBodyManipulator','TimeSteppingRigidBodyManipulator'})
      obj.robot = robot;
      
      if initialize_flag
        obj = obj.initialize();
      end
    end
    
    function obj = initialize(obj)
      obj.N =obj.getN;
      obj.nq = obj.robot.getNumPositions();
      x_inds = obj.getXinds();
      obj.q_inds = x_inds(1:obj.nq,:);
      obj.qsc_weight_inds = cell(1,obj.N);
      
      % Add joint limit constraints
      [joint_lb,joint_ub] = obj.robot.getJointLimits();
      obj = obj.addConstraint(BoundingBoxConstraint(reshape(bsxfun(@times,joint_lb,ones(1,obj.getN())),[],1),...
        reshape(bsxfun(@times,joint_ub,ones(1,obj.getN())),[],1)),obj.q_inds(:));
      
      % skip this if getHinds() = 0
      if obj.getHinds() > 0
        obj = obj.addConstraint(BoundingBoxConstraint(zeros(obj.getN()-1,1),inf(obj.getN()-1,1)),obj.getHinds());
      end
      
      % create shared data functions to calculate kinematics at the knot
      % points
      kinsol_dataind = zeros(obj.getN(),1);
      for i=1:obj.getN(),
        [obj,kinsol_dataind(i)] = obj.addSharedDataFunction(@obj.kinematicsData,{obj.q_inds(:,i)});
      end
      obj.kinsol_dataind = kinsol_dataind;
    end
    
    
    function data = kinematicsData(obj,q)
      data = doKinematics(obj.robot,q,false,false);
    end
    
    function obj = addKinematicConstraint(obj,constraint,time_index)
      % Add a kinematic constraint that is a function of the joint positions at the
      % specified time or times.
      % @param constraint  a CompositeConstraint
      % @param time_index   a cell array of time indices
      %   ex1., time_index = {1, 2, 3} means the constraint is applied
      %   individually to knot points 1, 2, and 3
      %   ex2,. time_index = {[1 2], [3 4]} means the constraint is applied to knot
      %   points 1 and 2 together (taking the combined state as an argument)
      %   and 3 and 4 together.
      if ~iscell(time_index)
        time_index = {time_index};
      end
      for j=1:length(time_index),
        kinsol_inds = obj.kinsol_dataind(time_index{j});
        cnstr_inds = mat2cell(obj.q_inds(:,time_index{j}),size(obj.q_inds,1),ones(1,length(time_index{j})));
        
        % record constraint for posterity
        %obj.constraints{end+1}.constraint = constraint;
        %obj.constraints{end}.var_inds = cnstr_inds;
        %obj.constraints{end}.kinsol_inds = kinsol_inds;
        %obj.constraints{end}.time_index = time_index;
        
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
          obj = obj.addConstraint(cnstr{1}, ...
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
          obj = obj.addConstraint(cnstr{1},obj.q_inds(:,time_index{j}));
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
  end
end
