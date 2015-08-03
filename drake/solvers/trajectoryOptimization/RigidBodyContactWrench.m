classdef RigidBodyContactWrench
  % constrain the contact forces
  properties(SetAccess = protected)
    robot % A RigidBodyManipulator or a TimeSteppingRigidBodyManipulator
    body % The index of contact body on the robot
    body_name % The name of the body.
    body_pts % A 3 x num_pts double matrix, each column represents the coordinate
             % of the contact point on the body frame.
    num_pts % The number of contact points
    num_wrench_constraint; % A scalar. The number of nonlinear constraints on the wrench paramters. 
    num_pt_F; % An integer. The number of force parameters for one contact point
    F_lb; % A double matrix of size F_size. The lower bound on the
          % force parameters
    F_ub; % A double matrix of size F_size. The upper bound on the
          % force parameters
    wrench_iCfun % A column vector. The row indices of the non-zero entries in the gradient of wrench constraints
    wrench_jCvar % A column vector. The column indices of the non-zero entries in the gradient of wrench constraints
    wrench_cnstr_ub % A num_wrench_constraint x 1 vector. The upper bound of the wrench constraint
    wrench_cnstr_lb % A num_wrench_constraint x 1 vector. The lower bound of the wrench constraint
    wrench_cnstr_name % A num_wrench_constraint x 1 cell of strings.
    contact_force_type
    num_slack  % An integer. The number of slack variables introduced to the wrench constraint
    slack_name % A cell of strings containing the name of slack variables
    slack_lb  % A num_slack x 1 vector. The lower bound of the slack variable;
    slack_ub  % A num_salck x 1 vector. The upper bound of the slack variable;
    kinematics_chain_idx % A vector of integers. It contains the joint configuration vector (q) indices corresponding to the joints that are on the kinematics chain to compute the body posture
    complementarity_flag % A boolean flag. True if we implement the complementarity contact constraint. @default is false
  end
  
  properties(Constant)
    FrictionConeType = 1;
    LinearFrictionConeType = 2;
    GraspType = 3;
    GraspFrictionConeType = 4;
  end
  
  methods
    function obj = RigidBodyContactWrench(robot,body,body_pts)
      % @param robot    A RigidBodyManipulator or a TimeSteppingRigidBodyManipulator
      % object
      % @param body       -- The index of contact body on the robot
      % @param body_pts   -- A 3 x num_pts double matrix, each column represents the coordinate
      % of the contact point on the body frame.
      if(~isa(robot,'RigidBodyManipulator') && ~isa(robot,'TimeSteppingRigidBodyManipulator'))
        error('Drake:RigidBodyContactWrench: expect a RigidBodyManipulator or a TimeSteppingRigidBodyManipulator');
      end
      obj.robot = robot;
      body_size = size(body);
      if(~isnumeric(body) || length(body_size) ~= 2 || body_size(1) ~= 1 || body_size(2) ~= 1)
        error('Drake:ContactWrenchConstraint: body should be a numeric scalar');
      end
      obj.body = body;
      obj.body_name = obj.robot.getBodyOrFrameName(obj.body);
      body_pts_size = size(body_pts);
      if(~isnumeric(body_pts) || length(body_pts_size) ~= 2 || body_pts_size(1) ~= 3)
        error('Drake:ContactWrenchConstraint: body_pts should be 3 x num_pts double matrix');
      end
      obj.body_pts = body_pts;
      obj.num_pts = body_pts_size(2);
      obj.num_slack = 0;
      obj.slack_name = {};
      obj.slack_lb = [];
      obj.slack_ub = [];
      [~,joint_path] = obj.robot.findKinematicPath(1,obj.body);
      if isa(obj.robot,'TimeSteppingRigidBodyManipulator')
        obj.kinematics_chain_idx = vertcat(obj.robot.getManipulator().body(joint_path).position_num)';
      else
        obj.kinematics_chain_idx = vertcat(obj.robot.body(joint_path).position_num)';
      end
      obj.complementarity_flag = false;
    end
    
    function [lincon,nlcon,bcon,num_slack,slack_name] = generateWrenchConstraint(obj)
      % @retval lincon   A LinearConstraint object on parameter F and slack variables
      % @retval nlcon    A FunctionHandleConstraint object on q, F and slack variables
      % @retval bcon     A BoundingBoxConstraint on F and slack variables
      nlcon = obj.generateWrenchNlcon();
      bcon = obj.generateWrenchBcon();
      lincon = obj.generateWrenchLincon();
      num_slack = obj.num_slack;
      if(length(obj.slack_name) ~= obj.num_slack)
        error('Drake:RigidBodyContactWrench:The slack_name has incorrect dimension');
      end
      slack_name = obj.slack_name;
    end
  end
  
  methods(Access = protected)
    function flag = checkForceSize(obj,F)
       F_size_tmp = size(F);
       flag = isnumeric(F) && length(F_size_tmp) == 2 && F_size_tmp(1) == obj.num_pt_F && F_size_tmp(2) == obj.num_pts;
    end
  end
  
  methods(Abstract)
    [c,dc] = evalWrenchConstraint(obj,kinsol,F,slack);
      % This function evaluates the constraint and its gradient
      % @param kinsol  - kinematics tree returned from doKinematics function
      % @param F       - A double matrix of obj.num_pt_F*obj.num_pts. The contact forces parameter
      % @param slack   - A obj.num_slack x 1 vector. The slack variables
      % @retval c      - A double column vector, the constraint value. The size of the
      % vector is obj.getNumConstraint(t) x 1
      % @retval dc - A double matrix. The gradient of c w.r.t q and F.
    A = torque(obj)
      % Compute the torque at each contact point from the force parameter.
      % @retval A   -- A (3*num_pts) x (obj.num_pt_F*obj.num_pts) double matrix.
      % reshape(A*F(:),3,num_pts) are the torque at each contact point
    A = force(obj)
      % Compute the indivisual force from the force parameters F. The individual forces
      % are reshape(A*F,3,obj.num_pts)
      % @retval A   -- A (3*num_pts) x (obj.num_pt_F*obj.num_pts) double matrix
    [pos,J] = contactPosition(obj,kinsol)
      % Compute the contact position and its gradient w.r.t q
      % @param kinsol   -- The kinematics tree
      % @retval pos     -- A matrix with 3 rows. pos(:,i) is the i'th contact position
      % @retval J       -- A matrix of size prod(size(pos)) x nq. The gradient of pos
      % w.r.t q
  end
  
  methods(Access=protected,Abstract)
    lincon = generateWrenchLincon(obj);
  end
  
  methods(Access = private)
    function nlcon = generateWrenchNlcon(obj)
      nlcon = FunctionHandleConstraint(obj.wrench_cnstr_lb,obj.wrench_cnstr_ub,obj.robot.getNumPositions+obj.num_pt_F*obj.num_pts+obj.num_slack,@(~,F,slack,kinsol) obj.evalWrenchConstraint(kinsol,reshape(F,obj.num_pt_F,obj.num_pts),slack));
      nlcon = nlcon.setSparseStructure(obj.wrench_iCfun,obj.wrench_jCvar);  
      nlcon = nlcon.setName(obj.wrench_cnstr_name);
    end
    
    function bcon = generateWrenchBcon(obj)
      bcon = BoundingBoxConstraint([obj.F_lb(:);obj.slack_lb],[obj.F_ub(:);obj.slack_ub]);
    end
  end
end
