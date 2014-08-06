classdef GraspWrench < RigidBodyContactWrench
  % A force and a torque can be applied at a single contact point. The force is
  % constrained to be within a sphere, namely it has a maximum magnitude, but arbitrary
  % direction. The torque is constrained within a polytope
  % F = [force_x;force_y;force_z;torque_x;torque_y;torque_z];
  properties(SetAccess = protected)
    force_max  % A positive scalar. The maximum magnitude of the force
    num_torque_cnstr % An integer. The number of halfspace constraint on the torque
    A_torque % An num_torque_cnstr x 3 matrix. b_torque_lb <= A_torque*torque<=b_torque_ub
    b_torque_ub % An num_torque_cnstr x 1 vector. 
    b_torque_lb % An num_torque_cnstr x 1 vector.
    
  end
  
  methods
    function obj = GraspWrench(robot,body,grasp_pt,force_max,A_torque,b_torque_lb,b_torque_ub)
      % @param robot  A RigidBodyManipulator or a TimeSteppingRigidBodyManipulator
      % @param body   An integer. The index of the contact body
      % @param grasp_pt  A 3 x 1 vector. The coordinate of the grasping point on the body
      % frame
      % @param force_max  A positive double. The magnitude of the maximum contact force
      % @param A_torque  An n x 3 matrix. b_torque_lb <= A_torque*torque<=b_torque_ub
      % @param b_torque_lb  An n x 1 vector.
      % @param b_torque_ub  An n x 1 vector.
      sizecheck(grasp_pt,[3,1]);
      obj = obj@RigidBodyContactWrench(robot,body,grasp_pt);
      sizecheck(force_max,[1,1]);
      if(~isnumeric(force_max) || force_max<0)
        error('Drake:GraspContactWrench:force_max should be non-negative scalaer');
      end
      obj.force_max = force_max;
      if(~isnumeric(A_torque) || ~isnumeric(b_torque_lb) || ~isnumeric(b_torque_ub))
        error('Drake:GraspContactWrench:expect A_torque, b_torque_lb and b_torque_ub being numeric');
      end
      sizecheck(A_torque,[nan,3]);
      obj.num_torque_cnstr = size(A_torque,1);
      sizecheck(b_torque_lb,[obj.num_torque_cnstr,1]);
      sizecheck(b_torque_ub,[obj.num_torque_cnstr,1]);
      if(any(b_torque_lb>b_torque_ub))
        error('Drake:GraspContactWrench:b_torque_lb should be no larger than b_torque_ub');
      end
      obj.A_torque = A_torque;
      obj.b_torque_lb = b_torque_lb;
      obj.b_torque_ub = b_torque_ub;
      obj.contact_force_type = RigidBodyContactWrench.GraspType;
      obj.F_lb = -inf(6,1);
      obj.F_ub = inf(6,1);
      obj.num_pt_F = 6;
      obj.num_wrench_constraint = 1;
      obj.wrench_cnstr_ub = obj.force_max^2/(obj.robot.getMass*9.81)^2;
      obj.wrench_cnstr_lb = 0;
      obj.wrench_iCfun = [1;1;1];
      obj.wrench_jCvar = obj.robot.getNumPositions+[1;2;3];
      obj.wrench_cnstr_name = {sprintf('%s_grasp_force_magnitude',obj.body_name)};
    end
    
    function [c,dc] = evalWrenchConstraint(obj,kinsol,F,slack)
      c = sum(F(1:3).^2);
      dc_val = 2*F(1:3);
      nq = obj.robot.getNumPositions;
      dc = sparse([1;1;1],nq+[1;2;3],dc_val,1,nq+6);
    end
    
    function A = force(obj)
      A = obj.robot.getMass*9.81*speye(3,6);
    end
    
    function A = torque(obj)
      A = obj.robot.getMass*9.81/100*[sparse(3,3) speye(3)];
    end
    
    function [pos,J] = contactPosition(obj,kinsol)
      [pos,J] = obj.robot.forwardKin(kinsol,obj.body,obj.body_pts,0);
    end
  end
  
  methods(Access=protected)
    function lincon = generateWrenchLincon(obj)
      A_torque = obj.torque();
      lincon = LinearConstraint(obj.b_torque_lb,obj.b_torque_ub,[zeros(obj.num_torque_cnstr,3) obj.A_torque*A_torque(:,4:6)]);
      lincon = lincon.setName(repmat({sprintf('%s_grasp_torque_constraint',obj.body_name)},obj.num_torque_cnstr,1));
    end
  end
end
