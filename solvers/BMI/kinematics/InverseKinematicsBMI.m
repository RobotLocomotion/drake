classdef InverseKinematicsBMI < BMIspotless
  properties(SetAccess = protected)
    robot
    robot_visualizer
    num_bodies % number of bodies
    body_quat % body_quat(:,i) is the quarterion of body i
    body_pos  % body_pos(:,i) is the position of body i
    cosine % cosine(i) is the cosine of the joint that connects body(i) and the parent of body(i)
    sine   % sine(i) is the sine of the joint that connects body(i) and the parent of body(i)
    body_Quat % body_Quat{i} is the 4 x 4 bilinear matrix to replace body_quat(:,i)*body_quat(:,i)';
    body_QuatTheta % body_QuatTheta{i} is a 4 x 2 bilinear matrix to replace body_quat(:,parent)*[cosine(i-1) sine(i-1)];
    body_Theta % body_Theta{i} is a 2 x 2 bilinear matrix to replace [cosine(i-1);sine(i-1)]*[cosine(i-1) sine(i-1)];
    
  end
  
  methods
    function obj = InverseKinematicsBMI(robot)
      obj = obj@BMIspotless();
      if(nargin>0)
        obj = obj.addRobot(robot);
      end
    end
    
    function obj = addRobot(obj,robot)
      if(~isa(robot,'RigidBodyManipulator') && ~isa(robot,'TimeSteppingRigidBodyManipulator'))
        error('The input should be a RigidBodyManipulator or a TimeSteppingRigidBodyManipulator');
      end
      obj.robot = robot;
      % Add the maximal coordinate for each body, using quaternions and
      % position
      obj.num_bodies = obj.robot.getNumBodies();
      [obj,obj.body_quat] = obj.newFree(4,obj.num_bodies);
      [obj,obj.body_pos] = obj.newFree(3,obj.num_bodies);
      [obj,obj.cosine] = obj.newFree(1,obj.num_bodies-1);
      [obj,obj.sine] = obj.newFree(1,obj.num_bodies-1);
      obj.body_Quat = cell(obj.num_bodies,1);
      obj.body_QuatTheta = cell(obj.num_bodies-1,1);
      obj.body_Theta = cell(obj.num_bodies-1,1);
      
      is_end_effector = true(1,obj.num_bodies);
      for i = 1:obj.num_bodies
        % add the kinematic constraints between adjacent bodies
        bodyi = obj.robot.getBody(i);
        parent_idx = bodyi.parent;
        if(parent_idx ~= 0) % The body is not 'world'
          [obj,obj.body_Quat{i}] = obj.newSym(4);
          is_end_effector(parent_idx) = false;
          if(~bodyi.floating) % non-floating base
            if(bodyi.pitch == 0) % revolute joint around z axis
              % revolute joint around z axis
              [obj,obj.body_QuatTheta{i-1}] = obj.newFree(4,2);
              [obj,obj.body_Theta{i-1}] = obj.newSym(2);
              % compute the position and orientation constraint between
              % adjacent bodies
              if(any(bodyi.T_body_to_joint(1:3,4)~= 0))
                error('Not implemented yet');
              end
              r = bodyi.Ttree(1:3,4);
              pos_expr = obj.body_pos(:,parent_idx) + rotmatFromQuatBilinear(obj.body_Quat{parent_idx})*r;
              tree_quat = rotmat2quat(bodyi.Ttree(1:3,1:3));
              body2joint_quat = rotmat2quat(bodyi.T_body_to_joint(1:3,1:3));
              quat_expr = clean(singleBodyQuatPropagation(obj.body_quat(:,parent_idx),tree_quat,body2joint_quat,obj.cosine(i-1),obj.sine(i-1)),1e-7);
%               forwardKin_expr = [pos_expr;quat_expr];
              Wi = [obj.body_Quat{parent_idx} obj.body_QuatTheta{i-1};obj.body_QuatTheta{i-1}' obj.body_Theta{i-1}];
              wi = [obj.body_quat(:,parent_idx);obj.cosine(i-1);obj.sine(i-1)];
              forwardKin_exprW = [pos_expr;replaceBilinearProduct(quat_expr,wi,Wi)];
              obj = obj.withEqs(forwardKin_exprW - [obj.body_pos(:,i);obj.body_quat(:,i)]);
              % unit quaternion
              obj = obj.withEqs(obj.body_Quat{i}(1,1)+obj.body_Quat{i}(2,2)+obj.body_Quat{i}(3,3)+obj.body_Quat{i}(4,4)-1);
              % sin^2+cos^2 = 1
              obj = obj.withEqs(obj.body_Theta{i-1}(1,1)+obj.body_Theta{i-1}(2,2)-1);
              % rank relaxation
              obj = obj.addBilinearVariable([obj.body_quat(:,parent_idx);obj.cosine(i-1);obj.sine(i-1)],Wi);
              % range on cos and sin
              theta_min = bodyi.joint_limit_min/2;
              theta_max = bodyi.joint_limit_max/2;
              [sin_min,sin_max,cos_min,cos_max] = rangeOfSinCos(theta_min,theta_max);
              obj = obj.withPos(obj.cosine(i-1)-cos_min);
              obj = obj.withPos(cos_max-obj.cosine(i-1));
              obj = obj.withPos(obj.sine(i-1)-sin_min);
              obj = obj.withPos(sin_max-obj.sine(i-1)); 
            else
              error('Not implemented yet');
            end
          else
            % floating base
            obj = obj.addBilinearVariable(obj.body_quat(:,i),obj.body_Quat{i});
            obj = obj.withEqs(obj.body_Quat{i}(1,1)+obj.body_Quat{i}(2,2)+obj.body_Quat{i}(3,3)+obj.body_Quat{i}(4,4)-1);
          end
        else
          is_end_effector(i) = false;
        end
      end
      % Now add the bilinear matrix Quat for end effectors
      for i = 1:obj.num_bodies
        if(is_end_effector(i))
          obj = obj.addBilinearVariable(obj.body_quat(:,i),obj.body_Quat{i});
        end
      end
      
      % Now handle the RigidBodyLoop
      for i = 1:length(obj.robot.loop)
        % Add the constraint that the pt1 on body1 coincides with pt2 on
        % body2
        body1 = obj.robot.loop(i).body1;
        body2 = obj.robot.loop(i).body2;
        pt1 = obj.robot.loop(i).pt1;
        pt2 = obj.robot.loop(i).pt2;
        pt1_pos = obj.body_pos(:,body1) + rotmatFromQuatBilinear(obj.body_Quat{body1})*pt1;
        pt2_pos = obj.body_pos(:,body2) + rotmatFromQuatBilinear(obj.body_Quat{body2})*pt2;
        obj = obj.withEqs(pt1_pos-pt2_pos);
      end
      
      obj.robot_visualizer = obj.robot.constructVisualizer();
    end
      
    function obj = addPositionConstraint(obj,link_idx,pt,Aeq,beq,Ain,bin)
      % add the constraint that the position c of point pt on link link_idx
      % satisfies the linear constraint
      % Ain*c<= bin
      % Aeq*c = beq
      if(numel(link_idx) ~= 1)
        error('Add one link each time');
      end
      obj = obj.addBilinearVariable(obj.body_quat(:,link_idx),obj.body_Quat{link_idx});
      pos_exprQ = obj.body_pos(:,link_idx) + rotmatFromQuatBilinear(obj.body_Quat{link_idx})*pt;
      if(~isempty(Aeq))
        if(size(Aeq,2) ~= 3 || size(Aeq,1) ~= size(beq,1) || size(beq,2) ~= 1)
          error('Incorrect size for linear equality Aeq*x=beq');
        end
        obj = obj.withEqs(Aeq*pos_exprQ-beq);
      end
      if(~isempty(Ain))
        if(size(Ain,2) ~= 3 || size(Ain,1) ~= size(bin,1) || size(bin,2) ~= 1)
          error('Incorrect size for linear inequality Ain*x<=bin');
        end
        obj = obj.withPos(bin-Ain*(obj.body_pos(:,link_idx)+pos_exprQ));
      end
    end
    
    function [q_sol,pos_sol,quat_sol,cosine_sol,sine_sol,Quat_sol,quat_theta_sol,theta_sol] = retrieveIKSolution(obj,sol)
      pos_sol = double(sol.eval(obj.body_pos));
      quat_sol = double(sol.eval(obj.body_quat));
      quat_sol(:,1) = [1;0;0;0];
      cosine_sol = double(sol.eval(obj.cosine));
      sine_sol = double(sol.eval(obj.sine));
      Quat_sol = cell(numel(obj.body_Quat),1);
      quat_theta_sol = cell(numel(obj.body_QuatTheta),1);
      theta_sol = cell(numel(obj.body_Theta),1);
      for i = 1:numel(Quat_sol)
        Quat_sol{i} = double(sol.eval(obj.body_Quat{i}));
      end
      for i = 1:numel(quat_theta_sol)
        quat_theta_sol{i} = double(sol.eval(obj.body_QuatTheta{i}));
      end
      for i = 1:numel(theta_sol)
        theta_sol{i} = double(sol.eval(obj.body_Theta{i}));
      end
      q_sol = zeros(obj.robot.getNumPositions,1);
      [joint_lb,joint_ub] = obj.robot.getJointLimits();
      for i = 1:obj.num_bodies
        if(obj.robot.getBody(i).parent ~= 0) % not world
          if(obj.robot.getBody(i).floating) %floating base
            T_base = [rotmatFromQuatBilinear(Quat_sol{i}) pos_sol(:,i);0 0 0 1];
            TJ = (obj.robot.getBody(i).T_body_to_joint)/obj.robot.getBody(i).Ttree*T_base/obj.robot.getBody(i).T_body_to_joint;
            if(obj.robot.getBody(i).floating == 1) % eular angles
              q_sol(obj.robot.getBody(i).position_num) = [TJ(1:3,4);rotmat2rpy(TJ(1:3,1:3))];
            else
              q_sol(obj.robot.getBody(i).position_num) = [TJ(1:3,4);rotmat2quat(TJ(1:3,1:3))];
            end
          else
            body_theta = asin(sine_sol(i-1)/sqrt(sine_sol(i-1)^2 + cosine_sol(i-1)^2))*2;
            if(cosine_sol(i-1)<0)
              body_theta = 2*pi-body_theta;
            end
            joint_ub_i = joint_ub(obj.robot.getBody(i).position_num);
            if(body_theta > joint_ub_i)
              delta_theta = ceil((body_theta - joint_ub_i)/(4*pi))*4*pi;
              body_theta = body_theta - delta_theta;
            end
            q_sol(obj.robot.getBody(i).position_num) = body_theta;
          end
        end
      end
    end
    
    function [sol,sol_bilinear] = retrieveSolution(obj,solver_sol)
      [sol,sol_bilinear] = retrieveSolution@BMIspotless(obj,solver_sol);
      if(~isempty(obj.robot))
        if(nargout<1)
          [sol.q,sol.body_pos,sol.body_quat,sol.cosine,sol.sine] = obj.retrieveIKSolution(solver_sol);
        else
          [sol.q,sol.body_pos,sol.body_quat,sol.cosine,sol.sine,sol_bilinear.body_Quat,sol_bilinear.body_QuatTheta,sol_bilinear.body_Theta] = obj.retrieveIKSolution(solver_sol);
        end
      end
    end
    
    function plotSolution(obj,sol,sol_bilinear)
      if(obj.use_lcmgl && ~isempty(obj.robot))
        if(isa(obj.robot_visualizer,'BotVisualizer'))
          for i = 1:obj.robot.getNumBodies
            obj.robot_visualizer.draw_msg.position(i,:) = sol.body_pos(:,i);
            obj.robot_visualizer.draw_msg.quaternion(i,:) = sol.body_quat(:,i);
          end
          lc = lcm.lcm.LCM.getSingleton();
          lc.publish('DRAKE_VIEWER_DRAW',obj.robot_visualizer.draw_msg);
        end
%         obj.robot_visualizer.draw(0,sol.q);
      else
        plotSolution@BMIspotless(obj,sol,sol_bilinear);
      end
    end
    
    function obj = fixLinkPosture(obj,link_idx,link_pos,link_quat)
      % fix the link position and orientation of the robot
      % @param link_idx  The index of the link to be fixed
      % @param link_pos  A 3 x 1 vector. The position of the link
      % @param link_quat A 4 x 1 unit vector. The quaternion of the link
      if(numel(link_idx)~=1)
        error('link_idx should be a scalar');
      end
      if(any(size(link_pos)~= [3,1]))
        error('link_pos should be a 3 x 1 vector');
      end
      if(any(size(link_quat) ~= [4,1]))
        error('link_quat should be a 4 x 1 vector');
      end
      norm_link_quat = norm(link_quat);
      if(abs(norm_link_quat-1)>1e-3)
        error('link_quat should be a unit vector');
      end
      link_quat = link_quat/norm_link_quat;
      obj = obj.withEqs(obj.body_pos(:,link_idx)-link_pos);
      obj = obj.withEqs(obj.body_quat(:,link_idx)-link_quat);
      obj = obj.withEqs(obj.body_Quat{link_idx}-link_quat*link_quat');
      for i = 1:obj.num_bodies
        if obj.robot.getBody(i).parent == link_idx
          obj = obj.withEqs(obj.body_QuatTheta{link_idx} -link_quat*[obj.cosine(i-1) obj.sine(i-1)]);
        end
      end
    end
  end
end