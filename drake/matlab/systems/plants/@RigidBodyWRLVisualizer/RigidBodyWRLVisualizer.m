classdef RigidBodyWRLVisualizer < RigidBodyVisualizer 
  % Loads a URDF file and implements the draw function.  
  % You could think of this as a very limited version of ROS' RViz. 
  % 
  
  methods
    function obj = RigidBodyWRLVisualizer(manip,options)
      checkDependency('vrml');
      typecheck(manip,'RigidBodyManipulator');

      global g_disable_visualizers;
      if g_disable_visualizers % evaluates to false if empty
        error('Drake:MissingDependency:WRLVisualizerDisabled','visualizer is disabled with g_disable_visualizers');
      end

      if ~usejava('awt') % usejava('awt') returns 0 if i'm running without a display
        error('Drake:MissingDependency:awt','VRML visualizer will not work without a display');
      end
      
      obj = obj@RigidBodyVisualizer(manip);
      
      if nargin<2
        options = struct();
      end
      
      wrlfile = fullfile(tempdir,[obj.model.name{1},'.wrl']);
      writeWRL(obj,wrlfile,options);
      
      % For some inexplicable reason, any interaction with the VR system causes 
      % subsequent calls to evalin('base', 'clear all classes java imports'); 
      % to become incredibly slow (on the order of 20 seconds instead of 0.01 
      % seconds). We are overwriting vrworld to set an environment variable so 
      % that later, in megaclear, we can skip a call to vrwho() if no vr worlds 
      % have been created. 

      setenv('HAS_MATLAB_VRWORLD', '1');
      obj.wrl = vrworld(wrlfile);
      if ~strcmpi(get(obj.wrl,'Open'),'on')
        open(obj.wrl);
      else
        reload(obj.wrl);
      end
      if get(obj.wrl,'Clients')<1
        view(obj.wrl);
      end
      obj.display_time=false;
    end
    
    function delete(obj)
      close(obj.wrl);
      delete(obj.wrl);
    end
    
    function viewpoint_struct = getViewpoint(obj)
      fig = get(obj.wrl,'Figures'); 
      viewpoint_struct.Viewpoint = get(fig,'Viewpoint');
      viewpoint_struct.CameraDirection = get(fig,'CameraDirection');
      viewpoint_struct.CameraPosition = get(fig,'CameraPosition');
      viewpoint_struct.CameraUpVector = get(fig,'CameraUpVector');
      viewpoint_struct.ZoomFactor = get(fig,'ZoomFactor');
    end
    
    function setViewpoint(obj,viewpoint_struct)
      fig = get(obj.wrl,'Figures'); 
      set(fig,viewpoint_struct);
    end
        
    function drawWrapper(obj,t,x)
      draw(obj,t,x);
    end
    
    function draw(obj,t,q)
      for i=1:length(obj.model.body)
        b = obj.model.body(i);
        if b.parent>0
          node=getfield(obj.wrl,b.jointname);
          if (b.floating==1)
            node.rotation = rpy2axis(q(b.position_num(4:6)))';
            node.translation = q(b.position_num(1:3))';
          elseif (b.floating==2)
            node.rotation = quat2axis(q(b.position_num(4:7)))';
            node.translation = q(b.position_num(1:3))';
          elseif (b.pitch==0)
            node.rotation=[b.joint_axis' q(b.position_num)];
          elseif isinf(b.pitch)
            node.translation=q(b.position_num)*b.joint_axis';
          else
            error('helical joints not implemented yet (but would be simple)');
          end
        end
      end
      set(obj.wrl,'Time',t);
      vrdrawnow;
      
      if (false)  % useful for graphically debugging kinematics
        figure(143); title('WRL kinematics debugger');
        kinsol = doKinematics(obj.model,x(1:obj.model.getNumPositions),false,false);
        pts = contactPositions(obj.model,kinsol);
        plot3(pts(1,:),pts(2,:),pts(3,:),'b*');
        xlabel('x'); ylabel('y'); zlabel('z');
        axis equal; xlim([-2.5,2.5]); ylim([-2.5,2.5]); zlim([-2.5;2.5]); view(0,20);
      end
    end
    
    function playbackAVI(obj,xtraj,filename)
      % Plays back a trajectory and creates an avi file.
      %   The filename argument is optional; if not specified, a gui will prompt
      %   for one.
      %
      %  @param xtraj trajectory to visulalize
      %  @param filename file to produce (optional, if not given a GUI will
      %    pop up and ask for it)
      
      typecheck(xtraj,'Trajectory');
      if (xtraj.getOutputFrame()~=obj.getInputFrame)
        xtraj = xtraj.inFrame(obj.getInputFrame);  % try to convert it
      end
      
      if (nargin<3)
        [filename,pathname] = uiputfile('*.avi','Save playback to AVI');
        filename = [pathname,'/',filename];
      end
      
      ts = getSampleTime(xtraj);
      fig = get(obj.wrl,'Figures');
      mov = VideoWriter(filename,'Motion JPEG AVI');

      if (ts(1)>0)
        tspan = getBreaks(xtraj);
        if (obj.display_dt~=0)
          interval = floor(obj.display_dt/ts(1));
          tspan = tspan(1:interval:end);
        else
          interval = 1;
        end
        mov.FrameRate = obj.playback_speed/(ts(1)*interval);
      else
        if (obj.display_dt==0)
          if (ishandle(obj)), error('i assumed it wasn''t a handle'); end
          obj.display_dt = 1/30;  % just for the remainder of this file.
        end
        
        breaks = getBreaks(xtraj);
        tspan = breaks(1):obj.display_dt:breaks(end);
        if (breaks(end)-tspan(end)>eps), tspan=[tspan,breaks(end)]; end
      
        mov.FrameRate = obj.playback_speed/obj.display_dt;
      end
      open(mov);
      
      width=[]; height=[];
      for i=1:length(tspan)
        obj.draw(tspan(i),eval(xtraj,tspan(i)));
        fr = capture(fig); pause(0.1);
        writeVideo(mov,fr);
      end
      
      close(mov);
    end
    
    function playbackSWF(varargin)
      error('SWF playback not available for VRML visualizers.  The vector graphics equivalent is playbackVRML.');
    end
    
    function playbackVRML(obj,xtraj,filename)
      set(obj.wrl,'Record3D','on');
      set(obj.wrl,'Record3DFileName',filename);
      set(obj.wrl,'RecordMode','scheduled');
      set(obj.wrl,'RecordInterval',[xtraj.tspan(1),xtraj.tspan(end)]);
%      set(obj.wrl,'Recording','on');
      
      playback(obj,xtraj);
      
      set(obj.wrl,'Recording','off');
    end
    
    function playbackMovie(obj,xtraj,filename)
      if (nargin<2)
        [filename,pathname] = uiputfile('*','Save playback to movie');
        if isequal(filename,0) || isequal(pathname,0)
          return;
        end
        filename = fullfile(pathname,filename);
      end

      [path,name,ext] = fileparts(filename);
      ext = tolower(ext);
      switch ext
        case '.avi'
          playbackAVI(obj,xtraj,filename);
        case '.wrl'
          playbackVRML(obj,xtraj,filename);
        otherwise
          disp(' Please select an output format: ');
          disp(' 1) AVI');
          disp(' 2) VRML');
          choice = input('Select a format (1-2): ');
          switch choice
            case 1
              ext = '.avi';
            case 2
              ext = '.wrl';
          end
          playbackMovie(obj,xtraj,fullfile(path,[name,ext]));
      end
    end    
  end

  properties (Access=protected)
    wrl;
  end
end
