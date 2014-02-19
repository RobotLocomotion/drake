classdef RigidBodyWRLVisualizer < RigidBodyVisualizer 
  % Loads a URDF file and implements the draw function.  
  % You could think of this as a very limited version of ROS' RViz. 
  % 
  
  methods
    function obj = RigidBodyWRLVisualizer(manip,options)
      % @option ground set options.ground = true to have ground visualized

      checkDependency('vrml');
      typecheck(manip,'RigidBodyManipulator');

      if ~usejava('awt') % usejava('awt') returns 0 if i'm running without a display
        error('Drake:MissingDependency:awt','VRML visualizer will not work without a display');
      end
      
      obj = obj@RigidBodyVisualizer(manip);
      
      if nargin<2
        options = struct();
      end
      if ~isfield(options,'ground') options.ground = manip.num_contacts>0; end
      
      wrlfile = fullfile(tempdir,[obj.model.name{1},'.wrl']);
      obj.model.writeWRL(wrlfile,options);
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
    
    function drawWrapper(obj,t,x)
      draw(obj,t,x);
    end
    
    function draw(obj,t,x)
      for i=1:length(obj.model.body)
        b = obj.model.body(i);
        if b.parent>0
          node=getfield(obj.wrl,b.jointname);
          if (b.floating==1)
            node.rotation = rpy2axis(x(b.dofnum(4:6)))';
            node.translation = x(b.dofnum(1:3))';
          elseif (b.floating==2)
            node.rotation = quat2axis(x(b.dofnum(4:7)))';
            node.translation = x(b.dofnum(1:3))';
          elseif (b.pitch==0)
            node.rotation=[b.joint_axis' x(b.dofnum)];
          elseif isinf(b.pitch)
            node.translation=x(b.dofnum)*b.joint_axis';
          else
            error('helical joints not implemented yet (but would be simple)');
          end
        end
      end
      set(obj.wrl,'Time',t);
      vrdrawnow;
      
      if (false)  % useful for graphically debugging kinematics
        figure(143); title('WRL kinematics debugger');
        kinsol = doKinematics(obj.model,x(1:obj.model.getNumDOF),false,false);
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
          if (ishandle(obj)) error('i assumed it wasn''t a handle'); end
          obj.display_dt = 1/30;  % just for the remainder of this file.
        end
        
        breaks = getBreaks(xtraj);
        tspan = breaks(1):obj.display_dt:breaks(end);
        if (breaks(end)-tspan(end)>eps) tspan=[tspan,breaks(end)]; end
      
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
    
    function playbackVRML(varargin)
      error('not implemented yet, but should be possible to record the sequence to a VRML movie');
    end
  end

  properties (Access=protected)
    wrl;
  end
end
