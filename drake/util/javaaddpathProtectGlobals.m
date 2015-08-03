function javaaddpathProtectGlobals(varargin)

% by default, calling javaaddpath class clear java which also clears global
% variables.  this wrapper simply saves the globals and restores them after
% the call
% This will also throw an error if there are user-created Java objects
% present which will prevent the java path from being updated.

% todo: could probably make these faster by avoiding the evals, but this
% was the quick version

% make a backup
current_globals = struct();
var = who('global');
for i=1:numel(var)
  eval(['global ',var{i}]);
  eval(['current_globals.',var{i},' = ', var{i},';']);
end

% Convert the 'not clearning java' warning into an error so we can catch it
w = warning('error', 'MATLAB:Java:DuplicateClass');
try
  javaaddpath(varargin{:});
catch err
  warning(w);
  if strcmp(err.identifier, 'MATLAB:Java:DuplicateClass')
    error('Drake:CannotClearJava', 'Could not modify the Java path because there are already Java objects present. You should either run ''clear java'' yourself or ensure that this function is called before any Java objects are created. If possible, move any calls to checkDependency to the beginning of their function or script.');
  else
    rethrow(err);
  end
end
warning(w);

% restore globals
var = fieldnames(current_globals);
for i=1:numel(var)
  eval(['global ',var{i}]);
  eval([var{i},' = current_globals.',var{i},';']);
end
