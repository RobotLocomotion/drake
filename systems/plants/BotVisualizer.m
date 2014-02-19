classdef BotVisualizer < RigidBodyVisualizer
  % Wraps the visualizer functionality around the drake libbot visualizer
  % (externally compiled program).  
  % Note: unlike other visualizers, only one BotVisualizer window can be
  % open at a time (multiple BotVisualizer class instances will share the
  % same viewer)

  methods
%    constructor (with optional urdf arg and call to addRobotFromURDF) 
%        opens a new viewer app with unique ID (like the figure num)
%        specified at the command line
%        viewer_id should be a shared data handle, which sends a shutdown
%        command on delete
%    draw
%    playbackMovie
%    lcmglwrappers
    function obj = BotVisualizer(manip)
      
      global g_disable_botvis;
      if g_disable_botvis % evaluates to false if empty
        error('Drake:BotVisualizer:Disabled','botvis is disabled with g_disable_botvis');
      end
      
      if ispc
        error('Drake:BotVisualizer:Windows','botvis doesn''t support windows yet');
      end
      
      if ~exist(fullfile(pods_get_bin_path,'drake_viewer'),'file')
        error('Drake:BotVisualizer:MissingViewer','can''t find drake_viewer executable.  you might need to run make (from the shell).  note: BotVisualizer is not supported on windows yet');
      end
      typecheck(manip,'RigidBodyManipulator');
      
      if ~strcmp(class(manip.terrain),'RigidBodyTerrain')
        error('Drake:BotVisualizer:UnsupportedModel','This model has (non-zero) terrain.  Not supported (yet)');
      end
%      if numel(manip.urdf)~=1
%        error('Drake:BotVisualizer:UnsupportedModel','I don''t actually support robots with multiple urdfs yet, but it will be easy enough');
%      end

      lc = lcm.lcm.LCM.getSingleton();
      agg = lcm.lcm.MessageAggregator();
      lc.subscribe('DRAKE_VIEWER_STATUS',agg);

      % check if there is an instance of drake_viewer already running
      [~,ck] = system('ps ax 2> /dev/null | grep -i drake_viewer | grep -c -v grep');
      if (str2num(ck)<1) 
        % if not, then launch one...
        disp('launching drake_viewer...');
        retval = systemWCMakeEnv([fullfile(pods_get_bin_path,'drake_viewer'),' &> drake_viewer.out &']);
        
        % listen for ready message
        if isempty(agg.getNextMessage(5000)) % wait for viewer to come up
          error('Drake:BotVisualizer:AutostartFailed','Failed to automatically start up a viewer');
        end
      end

      obj = obj@RigidBodyVisualizer(manip);

      %      obj = addRobotFromURDF(obj,urdf_filename);
      vc = drake.lcmt_viewer_command();
      vc.command_type = vc.LOAD_URDF;
      vc.command_data = [sprintf('%s:',manip.urdf{1:end-1}),manip.urdf{end}];
      lc.publish('DRAKE_VIEWER_COMMAND',vc);
      % listen for acknowledgement
      ack = agg.getNextMessage(5000);
      if isempty(ack)
        error('Drake:BotVisualizer:LoadURDFFailed','Did not receive ack from viewer');
      else
        msg = drake.lcmt_viewer_command(ack.data);
        if ~strcmp(vc.command_data,msg.command_data)
          error('Drake:BotVisualizer:LoadURDFFailed','ack from viewer contained different data');
        end
      end

      nq = getNumDOF(manip);
      obj.state_msg = drake.lcmt_robot_state();
      obj.state_msg.num_robots = length(manip.name);
      obj.state_msg.robot_name = manip.name;
      obj.state_msg.num_joints = nq;
      for i=1:length(manip.body)
        b = manip.body(i);
        if (b.dofnum>0)
          obj.state_msg.joint_robot(b.dofnum) = b.robotnum-1;
        end
      end
      obj.state_msg.joint_name = manip.getStateFrame.coordinates(1:nq);
      obj.state_msg.joint_position = single(zeros(nq,1));
      obj.state_msg.joint_velocity = single(zeros(nq,1));
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
