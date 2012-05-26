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
    function obj=Visualizer(num_u)
      obj=obj@DrakeSystem(0,0,num_u,0,true);
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
      draw(obj,t,u);
      y=[];
    end
    
    function ts = getSampleTime(obj)
      if (obj.display_dt>0)
        ts = [obj.display_dt;0];
      else
        ts = [-1;1];  % inherited sample time, fixed in minor offset
      end
    end
    
    function status = ode_draw(obj,t,x,flag)
      status=0;
      if (strcmp(flag,'done'))
        return;
      end
      draw(obj,t,x,[]);
    end
    
    function playback(obj,xtraj)
      %   Animates the trajectory in quasi- correct time using a matlab timer
      %     optional controlobj will playback the corresponding control scopes
      %
      %   @param xtraj trajectory to visualize
      
      tspan = xtraj.getBreaks();
      t0 = tspan(1);
      
      tic;
      
      if (obj.playback_speed<=0)  % then playback as quickly as possible
        t=tspan(1);
        while (t<tspan(end))
          t = tspan(1)+obj.playback_speed*toc;
          x = xtraj.eval(t);
          obj.draw(t,x);
          if (obj.display_time)
            title(['t = ', num2str(t,'%.2f') ' sec']);
          end
          drawnow;
        end
      else
        ti = timer('TimerFcn',{@timer_draw},'ExecutionMode','fixedRate','Period',max(obj.display_dt/obj.playback_speed,.01),'TasksToExecute',(tspan(end)-tspan(1))/max(obj.display_dt,eps),'BusyMode','drop');
        start(ti);
        wait(ti);  % unfortunately, you can't try/catch a ctrl-c in matlab
        delete(ti);
      end
      
      obj.draw(tspan(end),xtraj.eval(tspan(end)));
      
      function timer_draw(timerobj,event)
        t=tspan(1)+obj.playback_speed*toc;
        if (t>tspan(end))
          stop(timerobj);
          return;
        end
        x = xtraj.eval(t);
        obj.draw(t,x);
        if (obj.display_time)
          title(['t = ', num2str(t,'%.2f') ' sec']);
        end
        drawnow;
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
        obj.draw(tspan(i),eval(xtraj,tspan(i)));
        if (obj.display_time)
          title(['t = ', num2str(t,'%.2f') ' sec']);
        end
        if (obj.draw_axes)
          f=gcf;
        else
          f=gca;
        end
        if (isempty(width))
          fr=getframe(f);
          [width,height,~]=size(fr.cdata);
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
        obj.draw(tspan(i),eval(xtraj,tspan(i)));
        if (obj.display_time)
          title(['t = ', num2str(tspan(i),'%.2f') ' sec']);
        end
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
  end
  
end
