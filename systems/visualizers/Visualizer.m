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
      typecheck(input_frame,'CoordinateFrame');
      obj=obj@DrakeSystem(0,0,input_frame.dim,0,true);
      obj = setInputFrame(obj,input_frame);
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
      clf; hold on;
      draw(obj,t,y);
      if (obj.display_time)
        title(['t = ', num2str(t,'%.2f') ' sec']);
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
      %   @param xtraj trajectory to visualize
      %   @param options visualizer configuration:
      %                     slider: create playback slider to control time and speed

      typecheck(xtraj,'Trajectory');
      if (xtraj.getOutputFrame()~=obj.getInputFrame)
        xtraj = xtraj.inFrame(obj.getInputFrame);  % try to convert it
      end

      if nargin < 3
        options = struct();
      end
      if ~isfield(options, 'slider')
        options.slider = false;
      end

      f = sfigure(89);
      set(f, 'Visible', 'off');
      set(f, 'Position', [560 400 560 70]);

      tspan = xtraj.getBreaks();
      t0 = tspan(1);
      ts = getSampleTime(xtraj);
      time_steps = (tspan(end)-tspan(1))/max(obj.display_dt,eps);

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

      % use a little undocumented matlab to get continuous slider feedback:
      time_slider_listener = handle.listener(time_slider,'ActionEvent',@update_time_display);

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

    function inspector(obj,x0,state_dims,minrange,maxrange)
      % set up a little gui with sliders to manually adjust each of the
      % coordinates.

      fr = obj.getInputFrame();
      if (nargin<2) x0 = zeros(fr.dim,1); end
      if (nargin<3) state_dims = (1:fr.dim)'; end
      if (nargin<4) minrange = repmat(-5,size(state_dims)); end
      if (nargin<5) maxrange = -minrange; end

      x0(state_dims) = max(min(x0(state_dims),maxrange),minrange);

      obj.drawWrapper(0,x0);

      rows = ceil(length(state_dims)/2);
      f = sfigure(99); clf;
      set(f, 'Position', [560 400 560 20 + 30*rows]);

      y=30*rows-10;
      for i=reshape(state_dims,1,[])
        label{i} = uicontrol('Style','text','String',getCoordinateName(fr,state_dims(i)), ...
          'Position',[10+280*(i>rows), y+30*rows*(i>rows), 90, 20],'BackgroundColor',[.8 .8 .8]);
        slider{i} = uicontrol('Style', 'slider', 'Min', minrange(i), 'Max', maxrange(i), ...
          'Value', x0(i), 'Position', [100+280*(i>rows), y+30*rows*(i>rows), 170, 20],...
          'Callback',{@update_display});

        % use a little undocumented matlab to get continuous slider feedback:
        slider_listener{i} = handle.listener(slider{i},'ActionEvent',@update_display);
        y = y - 30;
      end

      function update_display(source, eventdata)
        t = 0; x = x0;
        for i=state_dims(:)'
          x(state_dims(i)) = get(slider{i}, 'Value');
        end
        x
        obj.drawWrapper(t,x);
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
      %
      % @param xtraj Trajectory to make the movie around
      % @param filename name of swf file. (optional, if it isn't given a GUI will pop
      %   up and ask for it.)

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
    axis;  % set this to non-empty for a fixed view (must be implemented by the draw method)
    fignum = 25;
  end

end
