function addtoenv(env,str,delim,insert_at_front)
% 
% Adds str to the end of the current environment variable
%
% @param env string name of the environment variable to set
% @param str string value that should be appended to the variable
% @param delim optional string value that should be inserted between 
%              entries @default pathsep

if (nargin<3 || isempty(delim)), delim=pathsep; end
if (nargin<4), insert_at_front=false; end

val = getenv(env);

if (insert_at_front)
  val = [str,delim,val];
else
  % search current env and only add when it's not already present
  [token,remain] = strtok(val,delim);
  while ~isempty(token)
    if strcmp(token,str)
      disp([str, ' is already included in environment variable ',env]);
      return;
    end
    remain = remain(length(delim)+1:end);  % zap the delimeter
    [token,remain] = strtok(remain,delim);
  end
  
  val = [val,delim,str];
end

setenv(env,val);
disp([env,'=',val]);
