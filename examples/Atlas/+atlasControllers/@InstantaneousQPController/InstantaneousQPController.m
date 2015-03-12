classdef InstantaneousQPController 
% A QP-based balancing and walking controller that exploits TV-LQR solutions
% for (time-varing) linear COM/ZMP dynamics. Includes logic specific to
% atlas/bipeds for raising the heel while walking. This differs from
% AtlasQPController in that it contains no stateful information about the
% currently executed plan. Instead, it is designed to be fully general for a
% variety of plan types (standing, walking, manipulating, etc.). The
% AtlasPlanEval class now handles the state of the current plan.
  properties(SetAccess=private, GetAccess=public);
    debug;
    debug_pub;
    robot;
    controller_data
    q_integrator_data
    vref_integrator_data
    robot_property_cache
    use_mex;
    data_mex_ptr;
    support_detect_mex_ptr;
    use_bullet = false;
    default_terrain_height = 0;
    param_sets
    gurobi_options = struct();
    solver = 0;
  end

  properties
    quiet = true;
  end

  methods
    function obj = InstantaneousQPController(r, param_sets, options)
      if nargin < 3
        options = struct();
      end
      if nargin < 2 || isempty(param_sets)
        param_sets = atlasParams.getDefaults(r);
      end
      options = applyDefaults(options,...
        struct('debug', false,...
               'use_mex', 1, ...
               'solver', 0),...
        struct('debug', @(x) typecheck(x, 'logical') && sizecheck(x, 1),...
               'use_mex', @isscalar, ...
               'solver', @(x) x == 0 || x == 1));
      for f = fieldnames(options)'
        obj.(f{1}) = options.(f{1});
      end

      if r.getNumPositions() ~= r.getNumVelocities()
        error('Drake:NonQuaternionFloatingBaseAssumption', 'this code assumes a 6-dof XYZRPY floating base, and will need to be updated for quaternions');
      end
      obj.robot = r;
      obj.param_sets = param_sets;
      obj.robot_property_cache = atlasUtil.propertyCache(r);
      import atlasControllers.*;
      import atlasFrames.*;

      if obj.debug
        obj.debug_pub = ControllerDebugPublisher('CONTROLLER_DEBUG');
      end

      obj.controller_data = InstantaneousQPControllerData(struct('infocount', 0,...
                                                     'qp_active_set', []));
      obj.q_integrator_data = IntegratorData(r);
      obj.vref_integrator_data = VRefIntegratorData(r);

      obj.gurobi_options.outputflag = 0; % not verbose
      if obj.solver==0
        obj.gurobi_options.method = 2; % -1=automatic, 0=primal simplex, 1=dual simplex, 2=barrier
      else
        obj.gurobi_options.method = 0; % -1=automatic, 0=primal simplex, 1=dual simplex, 2=barrier
      end
      obj.gurobi_options.presolve = 0;
      % obj.gurobi_options.prepasses = 1;

      if obj.gurobi_options.method == 2
        obj.gurobi_options.bariterlimit = 20; % iteration limit
        obj.gurobi_options.barhomogeneous = 0; % 0 off, 1 on
        obj.gurobi_options.barconvtol = 5e-4;
      end

      terrain = getTerrain(r);
      obj.default_terrain_height = r.getTerrainHeight([0;0]);
      if isa(terrain,'DRCTerrainMap')
        terrain_map_ptr = terrain.map_handle.getPointerForMex();
      else
        terrain_map_ptr = 0;
      end

      obj.data_mex_ptr = ...
             constructQPDataPointerMex(obj.robot.getMexModelPtr(),...
                                       obj.param_sets,...
                                       obj.robot_property_cache,...
                                       obj.robot.getB(),...
                                       obj.robot.umin,...
                                       obj.robot.umax,...
                                       terrain_map_ptr,...
                                       obj.default_terrain_height,...
                                       obj.solver==0,...
                                       obj.gurobi_options);
    end

    function [y, v_ref] = updateAndOutput(obj, t, x, qp_input, foot_contact_sensor)
      % Parse inputs from the robot and the planEval, set up the QP, solve it,
      % and return the torques and feed-forward velocity.
      % @param t time (s)
      % @param x robot state vector
      % @param qp_input a QPInputConstantHeight object
      % @param foot_contact_sensor a 2x1 vector indicating whether contact force was
      %                            detected by the [left; right] foot

      % t0 = tic();

      % Unpack variable names (just to make our code a bit easier to read)
      r = obj.robot;
      nq = obj.robot_property_cache.nq;
      nv = obj.robot_property_cache.nv;

      % lcmgl = LCMGLClient('desired zmp');
      % lcmgl.glColor3f(0, 0.7, 1.0);
      % lcmgl.sphere([y0; 0], 0.02, 20, 20);
      % lcmgl.switchBuffers();
      
      % mu = qp_input.support_data(1).mu;

      contact_sensor = zeros(obj.robot_property_cache.num_bodies, 1);
      if foot_contact_sensor(1) > 0.5
        contact_sensor(obj.foot_body_id.left) = 1;
      end
      if foot_contact_sensor(2) > 0.5
        contact_sensor(obj.foot_body_id.right) = 1;
      end
      ctrl_data = obj.controller_data;


      if (obj.use_mex==0 || obj.use_mex==2)
        % if t >= 0.1
        %   keyboard();
        % end
        q = x(1:nq);
        qd = x(nq+(1:nv));
      
        [y, qdd, info_fqp, supp,...
          alpha, Hqp, fqp, Aeq, beq, Ain, bin,lb,ub] = obj.setupAndSolveQP(t, q, qd, qp_input, contact_sensor);
        active_supports = [supp.body_id];
        if nargout >= 2
          params = obj.param_sets.(qp_input.param_set_name);
          v_ref = obj.velocityReference(t, q, qd, qdd, foot_contact_sensor, params.vref_integrator);
        end
      end

      if (obj.use_mex==1 || obj.use_mex==2)

        if (obj.use_mex==1)
          if ~obj.quiet
            t0 = tic();
          end
          [y,qdd,v_ref,info_fqp] = ...
                      instantaneousQPControllermex(obj.data_mex_ptr,...
                      t,...
                      x,...
                      qp_input,...
                      contact_sensor);
          if ~obj.quiet
            fprintf(1, 'mex: %f, ', toc(t0));
          end

          if info_fqp < 0
            ctrl_data.infocount = ctrl_data.infocount+1;
          else
            ctrl_data.infocount = 0;
          end
          if ctrl_data.infocount > 10 && exist('AtlasBehaviorModePublisher','class')
            % kill atlas
            disp('freezing atlas!');
            behavior_pub = AtlasBehaviorModePublisher('ATLAS_BEHAVIOR_COMMAND');
            d.utime = 0;
            d.command = 'freeze';
            behavior_pub.publish(d);
          end

        else
          if ~obj.quiet
            t0 = tic();
          end
          [y_mex,mex_qdd,vref_mex,info_mex,active_supports_mex,~,Hqp_mex,fqp_mex,...
            Aeq_mex,beq_mex,Ain_mex,bin_mex,Qf,Qeps] = ...
               instantaneousQPControllermex(obj.data_mex_ptr,...
                      t,...
                      x,...
                      qp_input,...
                      contact_sensor);
          if ~obj.quiet
            fprintf(1, 'mex: %f, ', toc(t0));
          end

          num_active_contacts = zeros(1, length(supp));
          for j = 1:length(supp)
            num_active_contacts(j) = size(supp(j).contact_pts, 2);
          end
          nc = sum(num_active_contacts);
          if (nc>0)
            valuecheck(active_supports_mex,active_supports);
          end

          if size(Hqp_mex,2)==1
            Hqp_mex=diag(Hqp_mex);
          end
          if info_mex < 0
            Hqp_mex=Hqp_mex*2;
            Qf=Qf*2;
            Qeps=Qeps*2;
          end
          if ~valuecheck(Hqp,blkdiag(Hqp_mex,diag(Qf),diag(Qeps)),1e-6);
            keyboard();
          end
          valuecheck(Hqp,blkdiag(Hqp_mex,diag(Qf),diag(Qeps)),1e-6);
          valuecheck(fqp',fqp_mex,1e-6);
          if ~obj.use_bullet
            % contact jacobian rows can be permuted between matlab/mex when
            % using bullet
            valuecheck(Aeq,Aeq_mex(1:length(beq),:),1e-6);
            valuecheck(beq,beq_mex(1:length(beq)),1e-6);
            valuecheck(Ain,Ain_mex(1:length(bin),:),1e-6);
            valuecheck(bin,bin_mex(1:length(bin)),1e-6);
          end
          valuecheck([-lb;ub],bin_mex(length(bin)+1:end),1e-6);
          if info_mex >= 0 && info_fqp >= 0 && ~obj.use_bullet
            % matlab/mex are using different gurobi fallback options, so
            % solutions can be slightly different
            %valuecheck(y,y_mex,1e-3);
            %valuecheck(qdd,mex_qdd,1e-3);
          end
        end
      end

      if obj.debug
        % publish debug
        debug_data.utime = t*1e6;
        debug_data.alpha = alpha;
        debug_data.u = y;
        debug_data.active_supports = active_supports;
        debug_data.info = info_fqp;
        debug_data.qddot_des = qddot_des;
        if obj.use_mex==0 % TODO: update this
          debug_data.active_constraints = qp_active_set;
        else
          debug_data.active_constraints = [];
        end
        debug_data.r_foot_contact = any(r.foot_body_id.right==active_supports);
        debug_data.l_foot_contact = any(r.foot_body_id.left==active_supports);
        if ~isempty(all_bodies_vdot)
          acc_mat = [[all_bodies_vdot.body_id]; all_bodies_vdot.body_vdot];
          debug_data.body_acc_des = reshape(acc_mat,numel(acc_mat),1);
        else
          debug_data.body_acc_des = [];
        end
        debug_data.zmp_err = [0;0];

        debug_data.individual_cops = zeros(3 * length(active_supports), 1);
        if obj.use_mex==0 % TODO: update this
          beta = alpha(nq + (1 : nc * nd));
          if ~isempty(active_supports)
            for j=1:length(active_supports)
              [~,Bj,~,~,normalj] = contactConstraintsBV(r,kinsol,false,struct('terrain_only',~obj.use_bullet,'body_idx',[1,active_supports(j)]));
              normals_identical = ~any(any(bsxfun(@minus, normalj, normalj(:,1))));
              if normals_identical % otherwise computing a COP doesn't make sense
                normalj = normalj(:,1);
                betaj = beta((j - 1) * nd + (1 : nd * supp(j).num_contact_pts));
                contact_positionsj = r.getBody(active_supports(j)).getTerrainContactPoints();
                forcej = zeros(3, 1);
                torquej = zeros(3, 1);
                min_contact_position_z = inf;
                for k = 1 : supp(j).num_contact_pts
                  Bjk = Bj{k};
                  betajk = betaj((k - 1) * nd + (1:nd));
                  contact_positionjk = contact_positionsj(:, k);
                  forcejk = Bjk * betajk;
                  forcej = forcej + forcejk;
                  torquejk = cross(contact_positionjk, forcejk);
                  torquej = torquej + torquejk;
                  if normalj' * contact_positionjk < min_contact_position_z
                    min_contact_position_z = normalj' * contact_positionjk;
                  end
                end
                fzj = normalj' * forcej; % in body frame
                if abs(fzj) > 1e-7
                  normal_torquej = normalj' * torquej; % in body frame
                  tangential_torquej = torquej - normalj * normal_torquej; % in body frame
                  cop_bodyj = cross(normalj, tangential_torquej) / fzj; % in body frame
                  cop_bodyj = cop_bodyj + min_contact_position_z * normalj;
                  cop_worldj = r.forwardKin(kinsol, active_supports(j), cop_bodyj,0);
                else
                  cop_worldj = nan(3, 1);
                end
                debug_data.individual_cops((j - 1) * 3 + (1 : 3)) = cop_worldj;
              end
            end
          end
        end

        obj.debug_pub.publish(debug_data);
      end

      if (0)     % simple timekeeping for performance optimization
        % note: also need to uncomment tic at very top of this method
        out_toc=toc(out_tic);
        persistent average_tictoc average_tictoc_n;
        if isempty(average_tictoc)
          average_tictoc = out_toc;
          average_tictoc_n = 1;
        else
          average_tictoc = (average_tictoc_n*average_tictoc + out_toc)/(average_tictoc_n+1);
          average_tictoc_n = average_tictoc_n+1;
        end
        if mod(average_tictoc_n,50)==0
          fprintf('Average control output duration: %2.4f\n',average_tictoc);
        end
      end
    end
  end
  
  methods(Static)
    function tf = applyContactLogic(support_logic_map, body_force_sensor, body_kin_sensor)
      tf = support_logic_map(2*logical(body_force_sensor) + logical(body_kin_sensor)+1);
    end
  end
  

end
