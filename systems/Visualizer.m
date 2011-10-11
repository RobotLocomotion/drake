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
      % Creates a SWF (Flash) movie of the trajectory.  This is often
      % useful for presentations because the movie is all vector graphics,
      % so will scale losslessly.
      %
      % You must have <b>pdftk</b> and <b>swftools</b> installed for this
      % to work.
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
        
        % Backup previous settings
        prePaperType = get(gcf,'PaperType');
        prePaperUnits = get(gcf,'PaperUnits');
        preUnits = get(gcf,'Units');
        prePaperPosition = get(gcf,'PaperPosition');
        prePaperSize = get(gcf,'PaperSize');

        % Make changing paper type possible
        set(gcf,'PaperType','<custom>');

        % Set units to all be the same
        set(gcf,'PaperUnits','inches');
        set(gcf,'Units','inches');

        % Set the page size and position to match the figure's onscreen
        % dimensions
        position = get(gcf,'Position');
        set(gcf,'PaperPosition',[0,0,position(3:4)]);
        set(gcf,'PaperSize',position(3:4));

        
        % Save the pdf
        % we do this instead of going to eps so that we'll print onto a
        % page, which keeps our size the same between frames that have
        % different sizes.
        print(gcf,'-dpdf',[frame_fname '.pdf']);
        
        % Restore the previous settings
        set(gcf,'PaperType',prePaperType);
        set(gcf,'PaperUnits',prePaperUnits);
        set(gcf,'Units',preUnits);
        set(gcf,'PaperPosition',prePaperPosition);
        set(gcf,'PaperSize',prePaperSize);
        
      end
      
      % merge pdfs
      cmd{1}=['pdftk `ls ',dirname,'/*.pdf` cat output ',dirname,'/merge.pdf'];

      % convert pdf to swf
      cmd{2}=['pdf2swf -s framerate=30 ',dirname,'/merge.pdf ',filename];

      % set the framerate to 30 (the framerate command above doesn't seem
      % to work in pdf2swf 0.8.1)
      % also combine with swfstop.swf which is a flash file that is in
      % robotlib/util that sends the stop command to prevent the movie from
      % looping
      cmd{3}=['swfcombine -r 30 --cat ',filename,' -o ',filename, ' ', getRobotlibPath(), '/util/swfstop.swf'];
      
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
