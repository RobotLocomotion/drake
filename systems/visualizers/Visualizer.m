classdef Visualizer < DrakeSystem
% An interface class which draws (e.g., produces a plot) the output of
% another dynamical system.  An example might be a set of plotting commands
% to draw a pendulum given the position of the pendulum.  Visualizers can
% be cascaded onto the output of a DynamicalSystem so that the
% visualization occurs at the time of simulation, or can 'playback' the
% trajectory that is the result of a simulation.

  methods (Abstract=true)
    draw(obj,t,y); % draw function interface
  end

  methods
    function obj=Visualizer(input_frame)
      if isempty(input_frame)
        dim = 0;
      else
        typecheck(input_frame,'CoordinateFrame');
        dim = input_frame.dim;
      end      
      obj=obj@DrakeSystem(0,0,dim,0,true);
      if ~isempty(input_frame)
        obj = setInputFrame(obj,input_frame);
      end
    end

    function x0 = getInitialState(obj)
      x0=[];
    end
    function xcdot = dynamics(obj,t,x,u)
      error('shouldn''t get here');
    end
    function xdn = update(obj,t,x,u)
      error('shouldn''t get here');
    end

    function y = output(obj,t,x,u)
      drawWrapper(obj,t,u);
      y=[];
    end

    function ts = getSampleTime(obj)
      if (obj.display_dt>0)
        ts = [obj.display_dt;0];
      else
        ts = [-1;1];  % inherited sample time, fixed in minor offset
      end
    end

    function drawWrapper(obj,t,y)
      sfigure(obj.fignum);
      if (obj.preserve_view)
        [az,el]=view;
      end
      clf; hold on;
      draw(obj,t,y);
      if (obj.display_time)
        title(['t = ', num2str(t,'%.2f') ' sec']);
      end
      if (obj.preserve_view)
        view(az,el);
      end
      drawnow;
    end

    function status = ode_draw(obj,t,x,flag)
      status=0;
      if (strcmp(flag,'done'))
        return;
      end
      drawWrapper(obj,t,x);
    end

    function playback(obj,xtraj,options)
      %   Animates the trajectory in quasi- correct time using a matlab timer
      %     optional controlobj will playback the corresponding control scopes
      %
      % @param xtraj trajectory to visualize
      % @option slider set to true to create playback slider to control time and speed
      % @option lcmlog plays back an lcmlog while calling the draw methods.
      %                (useful for, e.g., lcmgl debugging)

      typecheck(xtraj,'Trajectory');
      if (xtraj.getOutputFrame()~=obj.getInputFrame)
        xtraj = xtraj.inFrame(obj.getInputFrame);  % try to convert it
      end

      if nargin < 3, options = struct(); end
      defaultOptions.slider = false;
      defaultOptions.lcmlog = [];
      options = applyDefaults(options,defaultOptions);

      if ishandle(89)
        position = get(89, 'Position');
      else
        position = [560, 400];
      end
      f = sfigure(89);
      set(f, 'Visible', 'off');
      set(f, 'Position', [position(1:2), 560, 70]);

      tspan = xtraj.getBreaks();
      t0 = tspan(1);
      ts = getSampleTime(xtraj);
      time_steps = (tspan(end)-tspan(1))/max(obj.display_dt,eps);
      last_display_time = tspan(1)-eps;

      speed_format = 'Speed = %.3g';
      time_format = 'Time = %.3g';


      time_slider = uicontrol('Style', 'slider', 'Min', tspan(1), 'Max', tspan(end),...
        'Value', tspan(1), 'Position', [135, 10, 415, 20],...
        'Callback',{@update_time_display});
      speed_slider = uicontrol('Style', 'slider', 'Min', -3, 'Max', 1, ...
        'Value', log10(obj.playback_speed), 'Position', [255, 35, 295, 20], ...
        'Callback', {@update_speed});
      speed_display = uicontrol('Style', 'text', 'Position', [130, 35, 120, 20],...
        'String', sprintf(speed_format, obj.playback_speed));
      rewind_button = uicontrol('Style', 'pushbutton', 'String', 'Reset', ...
        'Position', [10, 35, 55, 20], 'Callback', {@rewind_vis});
      play_button = uicontrol('Style', 'pushbutton', 'String', 'Play', ...
        'Position', [70, 35, 55, 20], 'Callback', {@start_playback},...
        'Interruptible', 'on');
      time_display = uicontrol('Style', 'text', 'Position', [10, 10, 120, 20],...
        'String', sprintf(time_format, tspan(1)));

      % set up continuous slider feedback:
      time_slider_listener = addlistener(time_slider,'ContinuousValueChange',@update_time_display);

      function update_speed(source, eventdata)
        obj.playback_speed = 10 ^ (get(speed_slider, 'Value'));
        set(speed_display, 'String', sprintf(speed_format, obj.playback_speed));
      end
      function rewind_vis(source, eventdata)
        set(time_slider, 'Value', get(time_slider, 'Min'));
        update_time_display(time_slider, []);
      end
      function update_time_display(source, eventdata)
        t = get(time_slider, 'Value');
        if (ts(1)>0) t = round((t-ts(2))/ts(1))*ts(1) + ts(2); end  % align with sample times if necessary
        set(time_display, 'String', sprintf(time_format, t));
        obj.drawWrapper(t, xtraj.eval(t));
        if ~isempty(options.lcmlog)
          % note: could make this faster by not checking the times which I
          % know have passed
          topublish = options.lcmlog([options.lcmlog.simtime]>last_display_time & [options.lcmlog.simtime]<=t);
          if ~isempty(topublish)
            publishLCMLog(topublish);
          end
        end
        last_display_time = t;
      end
      function start_playback(source, eventdata)
        if ~ishandle(play_button)
          error('somebody deleted my uicontrols.  draw functions should not call clf without creating their own figure (it''s bad form)');
        end
        if get(play_button, 'UserData')
          set(play_button, 'UserData', 0, 'String', 'Play');
          return;
        else
          set(play_button, 'UserData', 1, 'String', 'Pause');
        end
%         set(stop_button, 'UserData', 0);
        if get(time_slider, 'Value') >= (tspan(end) - 0.02)
          set(time_slider, 'Value', get(time_slider, 'Min'));
        end
        tic;
        t0 = get(time_slider, 'Value');
        if (obj.playback_speed<=0)  % then playback as quickly as possible
          t = t0;
          while (t<tspan(end))
            t = t0 + obj.playback_speed*toc;
            if (ts(1)>0) t = round((t-ts(2))/ts(1))*ts(1) + ts(2); end  % align with sample times if necessary
            x = xtraj.eval(t);
            set(time_slider, 'Value', t)
            update_time_display(time_slider, [])
            if ~get(play_button, 'UserData')
              break;
            end
          end
        else
          ti = timer('TimerFcn',{@timer_draw},'ExecutionMode','fixedRate',...
            'Period',max(obj.display_dt/obj.playback_speed,.01),...
            'TasksToExecute',time_steps,'BusyMode','queue');
          start(ti);
          wait(ti);  % unfortunately, you can't try/catch a ctrl-c in matlab
          delete(ti);
        end
        set(play_button, 'UserData', 0, 'String', 'Play');
        function timer_draw(timerobj,event)
          t=t0+obj.playback_speed*toc;
          if (t>tspan(end))
            stop(timerobj);
            return;
          end
          if (ts(1)>0) t = round((t-ts(2))/ts(1))*ts(1) + ts(2); end  % align with sample times if necessary
          x = xtraj.eval(t);
          set(time_slider, 'Value', t)
          update_time_display(time_slider, [])
          if ~get(play_button, 'UserData')
            stop(timerobj);
            return;
          end
        end
      end
      update_time_display(time_slider, [])

      if options.slider
        set(f, 'Visible', 'on');
      else
        start_playback([], []);
      end
    end

    function inspector(obj,x0,state_dims,minrange,maxrange,visualized_system)
      % brings up a simple slider gui that displays the robot
      % in the specified state when possible.
      %
      % @param x0 the initial state to display the robot in
      % @param state_dims are the indices of the states to show on the
      % slider. Including velocity states will display the forces and torques.
      % @param minrange is the lower bound for the sliders
      % @param maxrange is the upper bound for the sliders
      % @param visualized_system is the system to be displayed
      %
      % Example, for drawing forces on a 12-state floating base system:
      %
      % <pre>
      %   v.inspector(zeros(12,1), 1:12)
      % </pre>

      fr = obj.getInputFrame();
      if (nargin<2 || isempty(x0)), x0 = zeros(fr.dim,1); end
      if (nargin<3 || isempty(state_dims)), state_dims = (1:fr.dim)'; end
      if (nargin<4), minrange = repmat(-5,size(state_dims)); end
      if (nargin<5), maxrange = -minrange; end
      if (nargin<6), visualized_system = []; end

      x0(state_dims) = max(min(x0(state_dims),maxrange),minrange);
      if ~isempty(visualized_system),
        bb_min = -inf(size(x0)); bb_min(state_dims) = minrange;
        bb_max = inf(size(x0)); bb_max(state_dims) = maxrange;
        bbcon = BoundingBoxConstraint(bb_min,bb_max);
        [x0,~,prog] = resolveConstraints(visualized_system,x0,[],bbcon);
      end

      rows = ceil(length(state_dims)/2);
      f = sfigure(99); clf;
      set(f,'ResizeFcn',@resize_gui);

      for i=1:numel(state_dims)
        label{i} = uicontrol('Style','text','String',getCoordinateName(fr,state_dims(i)), ...
          'HorizontalAlignment','right');
        slider{i} = uicontrol('Style', 'slider', 'Min', minrange(i), 'Max', maxrange(i), ...
          'Value', x0(state_dims(i)), 'Callback',{@update_display},'UserData',state_dims(i));
        value{i} = uicontrol('Style','text','String',num2str(x0(state_dims(i))), ...
          'HorizontalAlignment','left');

        % use a little undocumented matlab to get continuous slider feedback:
        slider_listener{i} = addlistener(slider{i},'ContinuousValueChange',@update_display);
      end

      set(f, 'Position', [560 400 560 20 + 30*rows]);
      resize_gui();
      if isempty(state_dims), 
        obj.drawWrapper(0,[]); 
      else 
        update_display(slider{1});
      end
      
      function resize_gui(source, eventdata)
        p = get(gcf,'Position');
        width = p(3);
        y=30*rows-10;
        for i=1:numel(state_dims)
          set(label{i},'Position',[20+width/2*(i>rows), y+30*rows*(i>rows), width/2-220, 20]);
          set(slider{i},'Position', [width/2-190+width/2*(i>rows), y+30*rows*(i>rows), 140, 20]);
          set(value{i},'Position', [width/2-45+width/2*(i>rows), y+30*rows*(i>rows), 45, 20]);
          y = y - 30;
        end
      end

      function update_display(source, eventdata)
        if nargin>1 && isempty(eventdata), return; end  % was running twice for most events

        t = 0; x = x0;
        for i=1:numel(state_dims)
          x(state_dims(i)) = get(slider{i}, 'Value');
          set(value{i},'String',num2str(x(state_dims(i)),'%4.3f'));
        end

        if (~isempty(visualized_system) && getNumStateConstraints(visualized_system)+getNumUnilateralConstraints(visualized_system)>0)
          % constrain the current slider to be exactly the specified value
          current_slider_statedim = get(source,'UserData');
          this_prog = addConstraint(prog,ConstantConstraint(get(source,'Value')),current_slider_statedim);

          % and add an objective to be as close as possible to the previous
          % solution on the other sliders.
          % objective = sum_over_remaining_state_dims .5*(x_i-x0_i)^2
          remaining_state_dims = state_dims(state_dims~=current_slider_statedim);
          if ~isempty(remaining_state_dims)
            Q = eye(numel(remaining_state_dims));
            b = -x0(remaining_state_dims);
            objective = QuadraticConstraint(0,inf,Q,b);
            this_prog = addCost(this_prog,objective,remaining_state_dims);
          end
          [x,objval,exitflag,infeasible_constraint_name] = solve(this_prog,x);
          if ~isempty(infeasible_constraint_name)
              infeasible_constraint_name
          end
%          .5*(x0(remaining_state_dims) - x(remaining_state_dims))^2
%          fval = objective.eval(x(remaining_state_dims))
          for i=1:numel(state_dims)
            set(slider{i},'Value',x(state_dims(i)));
          end
        end
        x0 = x;
        obj.drawWrapper(t,x);

        if (~isempty(visualized_system) && max(state_dims)>getNumPositions(visualized_system) && isa(obj,'RigidBodyVisualizer'))
          % was asked to show velocties on a RigidBodyManipulator, draw forces and torques
          q = x0(1:getNumPositions(visualized_system));
          qd = x0(getNumPositions(visualized_system)+1:getNumPositions(visualized_system)+getNumVelocities(visualized_system));
          visualized_system.drawLCMGLGravity(q,obj.gravity_visual_magnitude);
          visualized_system.drawLCMGLForces(q,qd,obj.gravity_visual_magnitude);
        end

      end
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
        case '.swf'
          playbackSWF(obj,xtraj,filename);
        otherwise
          disp(' Please select an output format: ');
          disp(' 1) AVI');
          disp(' 2) SWF');
          choice = input('Select a format (1-2): ');
          switch choice
            case 1
              ext = '.avi';
            case 2
              ext = '.swf';
          end
          playbackMovie(obj,xtraj,fullfile(path,[name,ext]));
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
      ts = getSampleTime(xtraj);

      if (nargin<3)
        [filename,pathname] = uiputfile('*.avi','Save playback to AVI');
        filename = [pathname,'/',filename];
      end

      if (obj.display_dt==0)
        if (ishandle(obj)) error('i assumed it wasn''t a handle'); end
        obj.display_dt = 1/30;  % just for the remainder of this file.
      end

      breaks = getBreaks(xtraj);
      tspan = breaks(1):obj.display_dt:breaks(end);
      if (breaks(end)-tspan(end)>eps) tspan=[tspan,breaks(end)]; end

      mov = VideoWriter(filename,'Motion JPEG AVI');
      mov.FrameRate = obj.playback_speed/obj.display_dt;
      open(mov);

      width=[]; height=[];
      for i=1:length(tspan)
        t = tspan(i);
        if (ts(1)>0) t = round((t-ts(2))/ts(1))*ts(1) + ts(2); end  % align with sample times if necessary
        obj.drawWrapper(t,eval(xtraj,t));
        if (obj.draw_axes)
          f=gcf;
        else
          f=gca;
        end
        if (isempty(width))
          fr=getframe(f);
          [height,width,~]=size(fr.cdata);
        else
          fr=getframe(f,[0 0 width height]);  % explicitly ask for the same size, otherwise videowriter will complain
        end
        writeVideo(mov,fr);
      end

      close(mov);
    end

    function playbackSWF(obj,xtraj,swf,options)
      % Creates a SWF (Flash) movie of the trajectory.  This is often
      % useful for presentations because the movie is all vector graphics,
      % so will scale losslessly.
      %
      % @param xtraj a Trajectory object that will be played back
      %
      % @param swf either the filename of the swf file to write, or an
      % instance of the SWFWriter class.  If swf is empty, then you will be
      % prompted for a filename (with the gui).
      %
      % @option poster set to true to export a pdf of the first frame.
      % @default false
      %
      % Note: You must have <b>pdftk</b> and <b>swftools</b> installed for
      % this to work.
      %
      % If you want to print axes, you'll want to set your visualizer to
      % draw axes: my_visualizer.draw_axes = true
      %
      % This scales the output file to look the same as the onscreen
      % figure, so if you want to change the dimensions, change the
      % onscreen figure's size and then run playbackSWF.
      %
      % Note that you can control the length/speed of your video by
      % changing your visualizer's display_dt property.

      typecheck(xtraj,'Trajectory');
      if (xtraj.getOutputFrame()~=obj.getInputFrame)
        xtraj = xtraj.inFrame(obj.getInputFrame);  % try to convert it
      end
      ts = getSampleTime(xtraj);

      bCloseAtEnd = true;
      if (nargin<3 || isempty(swf))
        swf = SWFWriter();  % this will prompt for a filename
      elseif isa(swf,'char') % then it's a filename
        swf = SWFWriter(swf);
      else
        bCloseAtEnd = false;
      end

      if (nargin<4) options=struct(); end
      if (isfield(options,'poster'))
        swf.poster = options.poster;
      end
      if (isfield(options,'loop'))
        swf.loop = options.loop;
      end

      if (obj.display_dt==0)
        if (ishandle(obj)) error('i assumed it wasn''t a handle'); end
        obj.display_dt = 1/30;  % just for the remainder of this file.
      end

      breaks = getBreaks(xtraj)/obj.playback_speed;
      tspan = obj.playback_speed*(breaks(1):obj.display_dt:breaks(end));
      if (breaks(end)-tspan(end)>eps) tspan=[tspan,breaks(end)]; end

      for i=1:length(tspan)
        t = tspan(i);
        if (ts(1)>0) t = round((t-ts(2))/ts(1))*ts(1) + ts(2); end  % align with sample times if necessary
        obj.drawWrapper(t,eval(xtraj,t));
        if (~obj.draw_axes) axis off; end
        swf.addFrame();
      end

      if (bCloseAtEnd)
        swf.close();
      end

    end
  end

  properties
    display_dt=0; %0.05;  % requested time between display frames (use 0 for drawing as fast as possible)
    playback_speed=1;  % 1=realtime
    draw_axes=false;  % when making movies true=gcf,false=gca
    display_time=true; % when true, time is written to figure title
    preserve_view=false; % when true, drawWrapper gets the view, calls draw, then restores the view
    axis;  % set this to non-empty for a fixed view (must be implemented by the draw method)
    fignum = 25;
  end

end
