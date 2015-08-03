function data = loadLCMDrakeLog(logfile,lcm_coder)
% Loads an LCM log file into a MATLAB structure using an LCMCoder.
%
% @param logfile Filename of the lcm log.  If empty, then a file
%   selection gui is opened.
% @param lcm_coder an LCMCoder object which defines the x,u, and y messages
%
% @retval a structure containing any of the elements .x, .xhat, .u, and .y
%   that are found in the log.    
%
% Note: Unlike loadLCMLog, this log data is known to be vector doubles, so
% can be returned in a more immediately useful structure.
%
% See also loadLCMLog

if (isempty(logfile))
  [filename,pathname]=uigetfile('*');
  logfile=[pathname,filename];
end
typecheck(lcm_coder,'LCMCoder');

try 
  log = lcm.logging.Log(logfile,'r');
catch
  disp(['Can''t load log file: ',logfile]);
end
robotname=lower(getRobotName(lcm_coder)); l=length(robotname)+1;

data=[];
while(1)
  try 
    ev = log.readNext();
  catch  % end of file exception
    break;
  end
  
  % todo: figure out utime field (typecast(ev.utime,'uint64') gets close,
  % but not quite logical data) and add it to trace
  
  channel=char(ev.channel);
  if (strncmp(channel,[robotname,'_'],l))
    channel=channel(l+1:end);
    switch (channel)
      case {'x','xhat'}
        [d,t]=decodeX(lcm_coder,ev);
      case 'u'
        [d,t]=decodeU(lcm_coder,ev);
      case 'y'
        [d,t]=decodeY(lcm_coder,ev);
      otherwise
            continue;
    end
    tfield = ['t_',channel];
    if (isfield(data,channel))
      data = setfield(data,channel,[getfield(data,channel),reshape(d,numel(d),1)]);
      data = setfield(data,tfield,[getfield(data,tfield),t]);
    else
      data = setfield(data,channel,reshape(d,numel(d),1));
      data = setfield(data,tfield,t);
    end
  end
end

end
