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
    if (obj.draw(t,x,[]))
      break;
    end
  end
else
  ti = timer('TimerFcn',{@timer_draw},'ExecutionMode','fixedRate','Period',max(obj.display_dt/obj.playback_speed,.01),'TasksToExecute',(tspan(end)-tspan(1))/obj.display_dt,'BusyMode','drop');
  start(ti);
  wait(ti);  % unfortunately, you can't try/catch a ctrl-c in matlab
  delete(ti);
end

obj.draw(tspan(end),xtraj.eval(tspan(end)),[]);

  function timer_draw(timerobj,event)
    t=tspan(1)+obj.playback_speed*toc;
    if (t>tspan(end)) 
      stop(timerobj); 
      return;
    end
    x = xtraj.eval(t);
    if (obj.draw(t,x,[])) 
      stop(timerobj); 
    end
  end

end