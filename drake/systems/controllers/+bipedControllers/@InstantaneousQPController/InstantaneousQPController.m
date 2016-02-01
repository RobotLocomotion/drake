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
    robot_property_cache
    data_mex_ptr;
    support_detect_mex_ptr;
    use_bullet = false;
    param_sets
    gurobi_options = struct();
    solver = 0;
  end

  properties
    quiet = true;
  end

  methods
    function obj = InstantaneousQPController(r, options)
      if nargin < 2
        options = struct();
      end
      options = applyDefaults(options,...
        struct('debug', false),...
        struct('debug', @(x) typecheck(x, 'logical') && sizecheck(x, 1)));
      for f = fieldnames(options)'
        obj.(f{1}) = options.(f{1});
      end

      if r.getNumPositions() ~= r.getNumVelocities()
        error('Drake:NonQuaternionFloatingBaseAssumption', 'this code assumes a 6-dof XYZRPY floating base, and will need to be updated for quaternions');
      end
      obj.robot = r;
      obj.robot_property_cache = r.getRobotPropertyCache();

      if obj.debug
        obj.debug_pub = ControllerDebugPublisher('CONTROLLER_DEBUG');
      end

      obj.data_mex_ptr = ...
             constructQPDataPointerMex(obj.robot.getManipulator.urdf{1},...
                                       obj.robot.control_config_file,...
                                       fullfile(getDrakePath(), 'examples', 'Atlas', 'config', 'urdf_modifications_robotiq_weight.yaml'));
    end

    function [y, v_ref] = updateAndOutput(obj, t, x, qp_input_msg, foot_contact_sensor)
      % Parse inputs from the robot and the planEval, set up the QP, solve it,
      % and return the torques and feed-forward velocity.
      % @param t time (s)
      % @param x robot state vector
      % @param qp_input_msg a drake.lcmt_qp_controller_input object
      % @param foot_contact_sensor a 2x1 vector indicating whether contact force was
      %                            detected by the [left; right] foot


      bodies_in_contact = {};
      if foot_contact_sensor(1) > 0.5
        bodies_in_contact{end+1} = obj.robot.l_foot_name;
      end
      if foot_contact_sensor(2) > 0.5
        bodies_in_contact{end+1} = obj.robot.r_foot_name;
      end
      for j = 1:length(qp_input_msg.support_data)
        if all(qp_input_msg.support_data(j).support_logic_map)
          bodies_in_contact{end+1} = char(qp_input_msg.support_data(j).body_name);
        end
      end

      if ~obj.quiet
        t0 = tic();
      end
      stream = java.io.ByteArrayOutputStream();
      data_output = java.io.DataOutputStream(stream);
      qp_input_msg.encode(data_output);
      byte_array = stream.toByteArray();
      [y,qdd,qd_ref,info_fqp] = ...
                  instantaneousQPControllermex(obj.data_mex_ptr,...
                  t,...
                  x,...
                  byte_array,...
                  bodies_in_contact);
      if ~obj.quiet
        fprintf(1, 'mex: %f, ', toc(t0));
      end
      
      v_ref = qd_ref(obj.robot_property_cache.actuated_indices);
    end
  end
end
