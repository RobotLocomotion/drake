classdef LibBotVisualizer %< Visualizer

  methods
%    constructor (with optional urdf arg and call to addRobotFromURDF) 
%        opens a new viewer app with unique ID (like the figure num)
%        specified at the command line
%        viewer_id should be a shared data handle, which sends a shutdown
%        command on delete
%    draw
%    playbackMovie
%    lcmglwrappers
    function obj = LibBotVisualizer()
      obj.state_msg = drake.systems.plants.viewer.lcmt_robot_state();
      obj.state_msg.robot_name = 'Acrobot';
      obj.state_msg.num_joints = 2;
      obj.state_msg.joint_name = {'theta1','theta2'};
      obj.state_msg.joint_position = single(zeros(2,1));
      obj.state_msg.joint_velocity = single(zeros(2,1));
    end
    
    function draw(obj,t,y)
      obj.state_msg.timestamp = int64(t*1000000);
      obj.state_msg.joint_position = single(y);
      lc = lcm.lcm.LCM.getSingleton();
      lc.publish('DRAKE_VIEWER_STATE',obj.state_msg);
    end

    function obj = addRobotFromURDF(obj,urdf_filename)
      vc = drake.systems.plants.viewer.lcmt_viewer_command();
      vc.command_type = vc.LOAD_URDF;
      vc.command_data = urdf_filename;
      lc = lcm.lcm.LCM.getSingleton();
      lc.publish('DRAKE_VIEWER_COMMAND',vc);
    end
    
    function obj = loadRenderer(obj,renderer_dynobj_path)
      % dynamically load a libbot renderer
      vc = drake.systems.plants.viewer.lcmt_viewer_command();
      vc.command_type = vc.LOAD_RENDERER;
      vc.command_data = renderer_dynobj_path;
      lc = lcm.lcm.LCM.getSingleton();
      lc.publish('DRAKE_VIEWER_COMMAND',vc);
    end

  end
  
  properties 
    viewer_id;
    state_msg;
  end
end