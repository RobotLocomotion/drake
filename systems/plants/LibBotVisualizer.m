classdef LibBotVisualizer < Visualizer

  methods
%    constructor (with optional urdf arg and call to addRobotFromURDF) 
%        opens a new viewer app with unique ID (like the figure num)
%        specified at the command line
%        viewer_id should be a shared data handle, which sends a shutdown
%        command on delete
%    draw
%    playbackMovie
%    lcmglwrappers
    function obj = LibBotVisualizer(urdf_filename)
      typecheck(urdf_filename,'char');
      urdf_filename = GetFullPath(urdf_filename);
      manip = RigidBodyManipulator(urdf_filename,struct('visual',false,'visual_geometry',false,'collision',false));
      obj = obj@Visualizer(getStateFrame(manip));

      %      obj = addRobotFromURDF(obj,urdf_filename);
      vc = drake.systems.plants.viewer.lcmt_viewer_command();
      vc.command_type = vc.LOAD_URDF;
      vc.command_data = urdf_filename;
      lc = lcm.lcm.LCM.getSingleton();
      lc.publish('DRAKE_VIEWER_COMMAND',vc);

      obj.state_msg = drake.systems.plants.viewer.lcmt_robot_state();
      obj.state_msg.robot_name = manip.name{1};
      obj.state_msg.num_joints = getNumDOF(manip);
      obj.state_msg.joint_name = manip.getStateFrame.coordinates(1:getNumDOF(manip));
      obj.state_msg.joint_position = single(zeros(2,1));
      obj.state_msg.joint_velocity = single(zeros(2,1));
    end
    
    function drawWrapper(obj,t,y)
      draw(obj,t,y);
    end
    
    function draw(obj,t,y)
      obj.state_msg.timestamp = int64(t*1000000);
      obj.state_msg.joint_position = single(y(1:obj.state_msg.num_joints));
      lc = lcm.lcm.LCM.getSingleton();
      lc.publish('DRAKE_VIEWER_STATE',obj.state_msg);
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