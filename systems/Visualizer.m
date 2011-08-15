classdef Visualizer < RobotLibSystem
% an interface for a draw method. listens on LCM for robot_xhat and displays  
%  note:  could make this a dynamicalsystem, but it's not right now

  methods (Abstract=true)
    draw(obj,t,x); % draw function interface
  end

  methods 
    function obj=Visualizer(num_u)
      obj=obj@RobotLibSystem(0,0,num_u,0,true);
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
      ts = [-1;1];  % inherited sample time, fixed in minor offset
    end
%    function x0 = getInitialState(obj)
%      x0 = [];
%    end
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
      
      tspan = xtraj.getBreaks();
      t0 = tspan(1);
      
      tic;
      
      if (obj.playback_speed<=0)  % then playback as quickly as possible
        t=tspan(1);
        while (t<tspan(end))
          t = tspan(1)+obj.playback_speed*toc;
          x = xtraj.eval(t);
          obj.draw(t,x);
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
      end
      
    end
    
    function playbackAVI(obj,xtraj,filename)
      % Plays back a trajectory and creates an avi file.
      %   The filename argument is optional; if not specified, a gui will prompt
      %   for one.
      %
      
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
  end
  
  properties 
    display_dt=0.05;  % requested time between display frames (use 0 for drawing as fast as possible)
    playback_speed=1;  % 1=realtime
    draw_axes=false;  % when making movies true=gcf,false=gca
  end
  
end
