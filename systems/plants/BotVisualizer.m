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

      obj = obj@RigidBodyVisualizer(manip);

      lc = lcm.lcm.LCM.getSingleton();
      obj.status_agg = lcm.lcm.MessageAggregator();
      lc.subscribe('DRAKE_VIEWER_STATUS',obj.status_agg);

      % check if there is an instance of drake_viewer already running
      [~,ck] = system('ps ax 2> /dev/null | grep -i drake_viewer | grep -c -v grep');
      if (str2num(ck)<1) 
        % if not, then launch one...
        disp('launching drake_viewer...');
        retval = systemWCMakeEnv([fullfile(pods_get_bin_path,'drake_viewer'),' &> drake_viewer.out &']);
        
        % listen for ready message
        if isempty(obj.status_agg.getNextMessage(5000)) % wait for viewer to come up
          error('Drake:BotVisualizer:AutostartFailed','Failed to automatically start up a viewer');
        end
      end

      vr = drake.lcmt_viewer_load_robot();
      vr.num_links = getNumBodies(manip);
      vr.link = javaArray('drake.lcmt_viewer_link_data',vr.num_links);
      for i=1:vr.num_links
        b = getBody(manip,i);
        link = drake.lcmt_viewer_link_data();
        link.name = b.linkname;
        link.robot_num = b.robotnum;
        link.num_geom = length(b.visual_shapes);
        if (link.num_geom>0)
          link.geom = javaArray('drake.lcmt_viewer_geometry_data',link.num_geom);
        end
        for j=1:link.num_geom
          link.geom(j) = serializeToLCM(b.visual_shapes{j});
        end
        vr.link(i) = link;
      end
      
      lc.publish('DRAKE_VIEWER_LOAD_ROBOT',vr);
      % listen for acknowledgement
      ack = obj.status_agg.getNextMessage(5000);
      if isempty(ack)
        error('Drake:BotVisualizer:LoadRobotFailed','Did not receive ack from viewer');
      else
        msg = drake.lcmt_viewer_command(ack.data);
%        if ~strcmp(vr.command_data,msg.command_data)
%          error('Drake:BotVisualizer:LoadURDFFailed','ack from viewer contained different data');
%        end
      end

      nq = getNumDOF(manip);
      obj.draw_msg = drake.lcmt_viewer_draw();
      nb = getNumBodies(manip);
      obj.draw_msg.num_links = nb;
      obj.draw_msg.link_name = {manip.body.linkname};
      obj.draw_msg.robot_num = [manip.body.robotnum];
      obj.draw_msg.position = single(zeros(nb,3));
      obj.draw_msg.quaternion = single(zeros(nb,4));
      
      draw(obj,0,zeros(getNumStates(manip),1));
    end
    
    function drawWrapper(obj,t,y)
      draw(obj,t,y);
    end
    
    function draw(obj,t,y)
      obj.draw_msg.timestamp = int64(t*1000000);
      
      kinsol = doKinematics(obj.model,y(1:getNumDOF(obj.model)));
      for i=1:getNumBodies(obj.model)
        pt = forwardKin(obj.model,kinsol,i,zeros(3,1),2);
        obj.draw_msg.position(i,:) = pt(1:3);
        obj.draw_msg.quaternion(i,:) = pt(4:7);
      end
      
      lc = lcm.lcm.LCM.getSingleton();
      lc.publish('DRAKE_VIEWER_DRAW',obj.draw_msg);
    end
    
    function obj = loadRenderer(obj,renderer_dynobj_path)
      % dynamically load a libbot renderer
      vc = drake.lcmt_viewer_command();
      vc.command_type = vc.LOAD_RENDERER;
      vc.command_data = renderer_dynobj_path;
      lc = lcm.lcm.LCM.getSingleton();
      lc.publish('DRAKE_VIEWER_COMMAND',vc);
    end
    
    function playbackMovie(obj,xtraj,filename)
      ffmpeg = getCMakeParam('ffmpeg');
      if isempty(ffmpeg)
        error('need ffmpeg.  rerun make configure from the prompt to help find it');
      end
      
      if (nargin<2)
        [filename,pathname] = uiputfile('*','Save playback to movie');
        if isequal(filename,0) || isequal(pathname,0)
          return;
        end
        filename = fullfile(pathname,filename);
      end
      
      [path,name,ext] = fileparts(filename);
      if isempty(ext), ext = '.mp4'; end  % set a default
      
      vc = drake.lcmt_viewer_command();
      vc.command_type = vc.START_RECORDING;
      vc.command_data = '';
      lc = lcm.lcm.LCM.getSingleton();
      lc.publish('DRAKE_VIEWER_COMMAND',vc);

      playback(obj,xtraj);
      
      vc.command_type = vc.STOP_RECORDING;
      lc.publish('DRAKE_VIEWER_COMMAND',vc);
      
      ppmsgz_filename='';
      while isempty(ppmsgz_filename)
        msg = obj.status_agg.getNextMessage(5000);
        if isempty(msg) % wait for ack
          error('Drake:BotVisualizer:RecordingFailed','Never received recording OK ack from viewer');
        end
        ppmsgz_filename=char(drake.lcmt_viewer_command(msg.data).command_data);
        % todo: make this more robust (so I don't get a different status
        % message)
      end
      
      system(['gzip -d ',ppmsgz_filename]);
      [p,n,e] = fileparts(ppmsgz_filename);  % remove .gz from filename
      ppms_filename = fullfile(p,n);

      vcodec = '';
      switch(ext)
        case '.mp4'
          vcodec = '-vcodec mpeg4'
        otherwise
          warning('haven''t handled this file extension yet.  you might need to add a -vcodec arg here to help ffmpeg decide');
      end
      system([ffmpeg,' -r 30 -y -vcodec ppm -f image2pipe -i ', ppms_filename,' ',vcodec,' ', filename]);
      delete(ppms_filename);
    end
    
  end
  
  properties 
    viewer_id;
    draw_msg;
    status_agg;
  end
end
