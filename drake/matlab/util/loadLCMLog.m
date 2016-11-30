function data = loadLCMLog(logfile,classpathtosearch)
% Loads an LCM log file into a MATLAB structure.  
%
% @param logfile Filename of the lcm log.  If empty, then a file
%   selection gui is opened.
% @param classpathtosearch a colon separated list of jar files (which must
%   include lcm.jar) to search for your lcmtype in. 
%   @default uses your dynamic java classpath (e.g. javaclasspath('-dynamic'); 
%
% @retval a structure containing all of the recognized channels found in 
%   the log file.  The field utime (the time in microseconds from the start
%   of the log) is appended to each data point.
%
% Note: data.channel is a list of structures (one for each message of 
% received).  This is appropriate, since not all messages are only simple
% numeric values.  If you'd like to get a simple time history of a
% variable, then you could do something like this:
%   utime = [data.channel(:).utime];  
% where channel is replaced by a true channel name.
%
% See also loadLCMDrakeLog 

if (nargin<1 || isempty(logfile))
  [filename,pathname]=uigetfile('*');
  logfile=[pathname,filename];
end
if (nargin<2)
  %classpathtosearch=getenv('CLASSPATH');
  
  % munge javaclasspath return into a colon separated list
  classpathtosearch=javaclasspath('-dynamic')'; 
  classpathtosearch=[classpathtosearch; repmat({':'},1,length(classpathtosearch))];
  classpathtosearch=[classpathtosearch{:}]; 
  classpathtosearch=classpathtosearch(1:end-1);  % remove extra ':'
  
  disp(['Searching for lcmtypes in the following classpath:\n',classpathtosearch]);
end
try 
  log = lcm.logging.Log(logfile,'r');
catch
  disp(['Can''t load log file: ',logfile]);
end
handlers = drake.matlab.util.MyLCMTypeDatabase(classpathtosearch);

data=struct();

while(1)
  try 
    ev = log.readNext();
  catch  % end of file exception
    break;
  end

  fingerprint = typecast(ev.data(8:-1:1),'UINT64');
  cls=handlers.getClassByFingerprint(fingerprint);
  if (~isempty(cls))
    % I'd like to do this:
    %   lcmtype = cls.getConstructor(byte[]).newInstance(ev.data);
    % but instead I have to do this hack:
    c = cls.getConstructors(); lcmtype=c(2).newInstance(ev.data);
    % end hack
    d=struct(lcmtype); d=rmfield(d,{'LCM_FINGERPRINT','LCM_FINGERPRINT_BASE'});
    d.utime=ev.utime;
    channel=char(ev.channel);
    if (isfield(data,channel))
      data=setfield(data,channel,[getfield(data,channel),d]);
    else
      data=setfield(data,channel,d);
    end
  end
end

end
