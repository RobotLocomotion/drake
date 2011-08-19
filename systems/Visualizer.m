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
    
    function playbackSWF(obj,xtraj,filename)
      if (nargin<3)
        [filename,pathname] = uiputfile('*.swf','Save playback to SWF');
        filename = [pathname,'/',filename];
      end
      
      dirname = [filename,'_frames'];
      mkdir(dirname);
      
      if (obj.display_dt==0)
        if (ishandle(obj)) error('i assumed it wasn''t a handle'); end
        obj.display_dt = 1/30;  % just for the remainder of this file.
      end
      
      breaks = getBreaks(xtraj);
      tspan = breaks(1):obj.display_dt:breaks(end);
      if (breaks(end)-tspan(end)>eps) tspan=[tspan,breaks(end)]; end
            
      width=[]; height=[];
      num_chars=length(num2str(length(tspan)));
      for i=1:length(tspan)
        obj.draw(tspan(i),eval(xtraj,tspan(i)));
        if (~obj.draw_axes) axis off; end
        frame_fname=[dirname,'/',repmat('0',1,num_chars-length(num2str(i))),num2str(i)];
        saveas(gcf,[frame_fname '.eps'],'epsc');
      end

      % convert to pdfs
      cmd{1}=['ls ',dirname,'/*.eps | xargs -n 1 -P 8 epstopdf'];

      % merge pdfs
      cmd{2}=['pdftk `ls ',dirname,'/*.pdf` cat output ',dirname,'/merge.pdf'];

      % convert pdf to swf
      cmd{3}=['pdf2swf -s framerate=30 ',dirname,'/merge.pdf ',filename];

      % set the framerate to 30 (the framerate command above doesn't seem
      % to work in pdf2swf 0.8.1)
      cmd{4}=['swfcombine -r 30 -d ',filename,' -o ',filename];
      
      for i=1:length(cmd)
        try
          if (system(cmd{i}) ~= 0) error('command failed'); end
        catch
          fprintf(1,'\n\n\ntry running this on the command line:\n  ');
          disp(cmd{i});
          disp('... go head.  i''ll wait.');
          input('just hit RETURN when you''re done');
        end
      end      

      disp([filename,' generated successfully']);
      
      rmdir(dirname,'s');

      end
  end
  
  properties 
    display_dt=0.05;  % requested time between display frames (use 0 for drawing as fast as possible)
    playback_speed=1;  % 1=realtime
    draw_axes=false;  % when making movies true=gcf,false=gca
  end
  
end
