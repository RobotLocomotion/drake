function playback(obj,xtraj)
%   Animates the trajectory in quasi- correct time using a matlab timer

tspan = xtraj.getBreaks();
t0 = tspan(1);

tic;
ti = timer('TimerFcn',{@timer_draw,obj,tspan,xtraj},'ExecutionMode','fixedRate','Period',max(obj.display_dt/obj.playback_speed,.01),'TasksToExecute',(tspan(end)-tspan(1))/obj.display_dt,'BusyMode','drop');
start(ti); 
wait(ti);  % unfortunately, you can't try/catch a ctrl-c in matlab
delete(ti);
obj.draw(tspan(end),xtraj.eval(tspan(end)),[]);


  function timer_draw(timerobj,event,vizobj,tspan,xtraj)
    t=tspan(1)+toc;
    if (t>tspan(end)) 
      stop(timerobj); 
      return;
    end
    x = xtraj.eval(t);
    if (vizobj.draw(t,x,[])) stop(timerobj); end
  end

end