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
               'solver', 0),...
        struct('debug', @(x) typecheck(x, 'logical') && sizecheck(x, 1),...
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
                                                     'qp_active_set', [],...
                                                     'num_active_contact_pts', 0));
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
        terrain_map_ptr = nullPointer();
      end

      coordinate_names = struct(...
        'state', {obj.robot.getStateFrame().coordinates(1:obj.robot.getNumPositions())},...
        'input', struct('robot', {atlasUtil.getHardwareJointNames(obj.robot)}, ...
                      'drake', {obj.robot.getInputFrame().coordinates}));


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
                                       obj.gurobi_options,...
                                       coordinate_names);
    end

    function [y, v_ref] = updateAndOutput(obj, t, x, qp_input, foot_contact_sensor)
      % Parse inputs from the robot and the planEval, set up the QP, solve it,
      % and return the torques and feed-forward velocity.
      % @param t time (s)
      % @param x robot state vector
      % @param qp_input a QPInputConstantHeight object
      % @param foot_contact_sensor a 2x1 vector indicating whether contact force was
      %                            detected by the [left; right] foot


      r = obj.robot;

      contact_sensor = zeros(obj.robot_property_cache.num_bodies, 1);
      if foot_contact_sensor(1) > 0.5
        contact_sensor(obj.foot_body_id.left) = 1;
      end
      if foot_contact_sensor(2) > 0.5
        contact_sensor(obj.foot_body_id.right) = 1;
      end
      ctrl_data = obj.controller_data;


      if ~obj.quiet
        t0 = tic();
      end
      [y,qdd,qd_ref,info_fqp] = ...
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
      
      v_ref = qd_ref(obj.robot_property_cache.actuated_indices);
    end
  end
end
