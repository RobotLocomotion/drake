classdef InverseKinematicsBMI < BMIspotless
  properties(SetAccess = protected)
    robot
    robot_visualizer
    num_bodies % number of bodies
    body_quat % body_quat(:,i) is the quarterion of body i
    body_pos  % body_pos(:,i) is the position of body i
    joint_cos % joint_cos(i) is the cosine of the joint that connects body(i) and the parent of body(i)
    joint_sin   % joint_sin(i) is the sine of the joint that connects body(i) and the parent of body(i)
    body_Quat % body_Quat{i} is the 4 x 4 bilinear matrix to replace body_quat(:,i)*body_quat(:,i)';
    body_QuatTheta % body_QuatTheta{i} is a 4 x 2 bilinear matrix to replace body_quat(:,parent)*[joint_cos(i-1) joint_sin(i-1)];
    body_Theta % body_Theta{i} is a 2 x 2 bilinear matrix to replace [joint_cos(i-1);joint_sin(i-1)]*[joint_cos(i-1) joint_sin(i-1)];
    
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
      [obj,obj.joint_cos] = obj.newFree(1,obj.num_bodies-1);
      [obj,obj.joint_sin] = obj.newFree(1,obj.num_bodies-1);
      obj.body_Quat = cell(obj.num_bodies,1);
      obj.body_QuatTheta = cell(obj.num_bodies-1,1);
      obj.body_Theta = cell(obj.num_bodies-1,1);
      
      is_end_effector = true(1,obj.num_bodies);
      for i = 2:obj.num_bodies
        % add the kinematic constraints between adjacent bodies
        bodyi = obj.robot.getBody(i);
        parent_idx = bodyi.parent;
        [obj,obj.body_Quat{i}] = obj.newSym(4);
        
        is_end_effector(parent_idx) = false;
        if(~bodyi.floating) % non-floating base
          if(bodyi.pitch == 0) % revolute joint around z axis
            % revolute joint around z axis
            
            [obj,obj.body_Theta{i-1}] = obj.newSym(2);
            % compute the position and orientation constraint between
            % adjacent bodies
            if(any(bodyi.T_body_to_joint(1:3,4)~= 0))
              error('Not implemented yet');
            end

            if(parent_idx ~= 1) % parent link is not 'world'
              [obj,obj.body_QuatTheta{i-1}] = obj.newFree(4,2);
              parent_pos = obj.body_pos(:,parent_idx);
              parent_quat = obj.body_quat(:,parent_idx);
              parent_Quat = obj.body_Quat{parent_idx};
              Wi = [obj.body_Quat{parent_idx} obj.body_QuatTheta{i-1};obj.body_QuatTheta{i-1}' obj.body_Theta{i-1}];
              % rank relaxation
              obj = obj.addBilinearVariable([obj.body_quat(:,parent_idx);obj.joint_cos(i-1);obj.joint_sin(i-1)],Wi);
            else
              parent_pos = zeros(3,1);
              parent_quat = [1;0;0;0];
              parent_Quat = zeros(4);
              parent_Quat(1,1) = 1;
              obj = obj.addBilinearVariable([obj.joint_cos(i-1);obj.joint_sin(i-1)],obj.body_Theta{i-1});
            end
            r = bodyi.Ttree(1:3,4);
            pos_expr = parent_pos + rotmatFromQuatBilinear(parent_Quat)*r;
            tree_quat = rotmat2quat(bodyi.Ttree(1:3,1:3));
            body2joint_quat = rotmat2quat(bodyi.T_body_to_joint(1:3,1:3));
            quat_expr = clean(singleBodyQuatPropagation(parent_quat,tree_quat,body2joint_quat,obj.joint_cos(i-1),obj.joint_sin(i-1)),1e-7);

            if(parent_idx ~= 1)
              wi = [obj.body_quat(:,parent_idx);obj.joint_cos(i-1);obj.joint_sin(i-1)];
              forwardKin_exprW = [pos_expr;replaceBilinearProduct(quat_expr,wi,Wi)];
            else
              forwardKin_exprW = [pos_expr;quat_expr];
            end
            obj = obj.withEqs(forwardKin_exprW - [obj.body_pos(:,i);obj.body_quat(:,i)]);
            % unit quaternion
            obj = obj.withEqs(obj.body_Quat{i}(1,1)+obj.body_Quat{i}(2,2)+obj.body_Quat{i}(3,3)+obj.body_Quat{i}(4,4)-1);
            % sin^2+cos^2 = 1
            obj = obj.withEqs(obj.body_Theta{i-1}(1,1)+obj.body_Theta{i-1}(2,2)-1);
            
            % range on cos and sin
            theta_min = bodyi.joint_limit_min/2;
            theta_max = bodyi.joint_limit_max/2;
            [sin_min,sin_max,cos_min,cos_max] = rangeOfSinCos(theta_min,theta_max);
            obj = obj.withPos(obj.joint_cos(i-1)-cos_min);
            obj = obj.withPos(cos_max-obj.joint_cos(i-1));
            obj = obj.withPos(obj.joint_sin(i-1)-sin_min);
            obj = obj.withPos(sin_max-obj.joint_sin(i-1)); 
          else
            error('Non-revolute joint is not supported yet');
          end
        else
          % floating base
          obj = obj.addBilinearVariable(obj.body_quat(:,i),obj.body_Quat{i});
          obj = obj.withEqs(obj.body_Quat{i}(1,1)+obj.body_Quat{i}(2,2)+obj.body_Quat{i}(3,3)+obj.body_Quat{i}(4,4)-1);
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
        frameA = obj.robot.loop(i).frameA;
        frameB = obj.robot.loop(i).frameB;
        [bodyA,T_frameA] = extractFrameInfo(obj.robot,frameA);
        [bodyB,T_frameB] = extractFrameInfo(obj.robot,frameB);
        bodyA_rotmat = rotmatFromQuatBilinear(obj.body_Quat{bodyA});
        bodyB_rotmat = rotmatFromQuatBilinear(obj.body_Quat{bodyB});
        % origins coincide.
        frameA_origin = bodyA_rotmat*T_frameA(1:3,4) + obj.body_pos(:,bodyA);
        frameB_origin = bodyB_rotmat*T_frameB(1:3,4) + obj.body_pos(:,bodyB);
        obj = obj.withEqs(frameA_origin - frameB_origin);
        % axis coincide
        frameA_axis = bodyA_rotmat*T_frameA(1:3,1:3)*obj.robot.loop(i).axis + frameA_origin;
        frameB_axis = bodyB_rotmat*T_frameB(1:3,1:3)*obj.robot.loop(i).axis + frameB_origin;      
        obj = obj.withEqs(frameA_axis - frameB_axis);
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
    
    function [sol,sol_bilinear] = retrieveSolution(obj,solver_sol)
      if(nargout > 1)
        [sol,sol_bilinear] = retrieveSolution@BMIspotless(obj,solver_sol);
      else
        sol = retrieveSolution@BMIspotless(obj,solver_sol);
      end
      if(~isempty(obj.robot))
        sol.body_pos = double(solver_sol.eval(obj.body_pos));
        sol.body_quat = double(solver_sol.eval(obj.body_quat));
        sol.body_quat(:,1) = [1;0;0;0];
        sol.joint_cos = double(solver_sol.eval(obj.joint_cos));
        sol.joint_sin = double(solver_sol.eval(obj.joint_sin));
        if(nargout > 1)
          sol_bilinear.body_Quat = cell(numel(obj.body_Quat),1);
          sol_bilinear.body_QuatTheta = cell(numel(obj.body_QuatTheta),1);
          sol_bilinear.body_Theta = cell(numel(obj.body_Theta),1);
          for i = 1:numel(sol_bilinear.body_Quat)
            sol_bilinear.body_Quat{i} = double(solver_sol.eval(obj.body_Quat{i}));
          end
          for i = 1:numel(sol_bilinear.body_QuatTheta)
            sol_bilinear.body_QuatTheta{i} = double(solver_sol.eval(obj.body_QuatTheta{i}));
          end
          for i = 1:numel(sol_bilinear.body_Theta)
            sol_bilinear.body_Theta{i} = double(solver_sol.eval(obj.body_Theta{i}));
          end
          sol_bilinear.body_Quat{1} = zeros(4,4);
          sol_bilinear.body_Quat{1}(1,1) = 1;
        end
        sol.q = zeros(obj.robot.getNumPositions,1);
        [joint_lb,joint_ub] = obj.robot.getJointLimits();
        for i = 1:obj.num_bodies
          if(obj.robot.getBody(i).parent ~= 0) % not world
            if(obj.robot.getBody(i).floating) %floating base
              T_base = [rotmatFromQuatBilinear(sol.body_quat(:,i)*sol.body_quat(:,i)') sol.body_pos(:,i);0 0 0 1];
              TJ = (obj.robot.getBody(i).T_body_to_joint)/obj.robot.getBody(i).Ttree*T_base/obj.robot.getBody(i).T_body_to_joint;
              if(obj.robot.getBody(i).floating == 1) % eular angles
                sol.q(obj.robot.getBody(i).position_num) = [TJ(1:3,4);rotmat2rpy(TJ(1:3,1:3))];
              else
                sol.q(obj.robot.getBody(i).position_num) = [TJ(1:3,4);rotmat2quat(TJ(1:3,1:3))];
              end
            else
              body_theta = asin(sol.joint_sin(i-1)/sqrt(sol.joint_sin(i-1)^2 + sol.joint_cos(i-1)^2))*2;
              if(sol.joint_cos(i-1)<0)
                body_theta = 2*pi-body_theta;
              end
              joint_ub_i = joint_ub(obj.robot.getBody(i).position_num);
              if(body_theta > joint_ub_i)
                delta_theta = ceil((body_theta - joint_ub_i)/(4*pi))*4*pi;
                body_theta = body_theta - delta_theta;
              end
              sol.q(obj.robot.getBody(i).position_num) = body_theta;
            end
          end
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
          obj = obj.withEqs(obj.body_QuatTheta{link_idx} -link_quat*[obj.joint_cos(i-1) obj.joint_sin(i-1)]);
        end
      end
    end
  end
end