classdef InstantaneousQPController 
% A QP-based balancing and walking controller that exploits TV-LQR solutions
% for (time-varing) linear COM/ZMP dynamics. Includes logic specific to
% atlas/bipeds for raising the heel while walking. This differs from
% AtlasQPController in that it contains no stateful information about the
% currently executed plan. Instead, it is designed to be fully general for a
% variety of plan types (standing, walking, manipulating, etc.). The
% AtlasPlanEval class now handles the state of the current plan.
  properties(SetAccess=private, GetAccess=public);
    data_mex_ptr;
  end

  properties
    quiet = true;
  end

  methods
    function obj = InstantaneousQPController(urdf_filename, control_config_filename, urdf_modifications_filename)
      if nargin < 3
        urdf_modifications_filename = '';
      end
      obj.data_mex_ptr = ...
             constructQPDataPointerMex(urdf_filename,...
                                       control_config_filename,...
                                       urdf_modifications_filename);
    end

    function y = updateAndOutput(obj, t, x, qp_input_msg, foot_contact_sensor)
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

      stream = java.io.ByteArrayOutputStream();
      data_output = java.io.DataOutputStream(stream);
      qp_input_msg.encode(data_output);
      byte_array = stream.toByteArray();
      y = ...
        instantaneousQPControllermex(obj.data_mex_ptr,...
        t,...
        x,...
        byte_array,...
        bodies_in_contact);
    end
  end
end
