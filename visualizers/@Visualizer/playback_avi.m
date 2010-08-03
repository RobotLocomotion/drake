function playbackAVI(obj,xtraj,filename)
% Plays back a trajectory and creates an avi file.
%   The filename argument is optional; if not specified, a gui will prompt
%   for one.
%

if (nargin<
if (~isfield(options,'filename'))
  options.filename = uiputfile('*.avi','Save playback to AVI');
end

breaks = getBreaks(xtraj);
tspan = breaks(1):obj.display_dt:breaks(end);
if (breaks(end)-tspan(end)>eps) tspan=[tspan;breaks(end)]; end

if (ispc)
  mov = avifile(options.filename,'fps',obj.playback_speed/obj.display_dt,'compression','Cinepak');
else
  mov = avifile(options.filename,'fps',obj.playback_speed/obj.display_dt);
end

for i=1:length(tspan)
  if (obj.draw(tspan(i),eval(xtraj,tspan(i)),[])), stop(obj), end;
  if (obj.draw_axes)
    mov = addframe(mov,getframe(gcf));
  else
    mov = addframe(mov,getframe(gca));
  end
end

mov = close(mov);
