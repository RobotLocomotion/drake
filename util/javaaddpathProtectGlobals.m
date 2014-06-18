function javaaddpathProtectGlobals(varargin)

% by default, calling javaaddpath class clear java which also clears global
% variables.  this wrapper simply saves the globals and restores them after
% the call

% todo: could probably make these faster by avoiding the evals, but this
% was the quick version

% make a backup
current_globals = struct();
var = who('global');
for i=1:numel(var)
  eval(['global ',var{i}]);
  eval(['current_globals.',var{i},' = ', var{i},';']);
end

javaaddpath(varargin{:});

% restore globals
var = fieldnames(current_globals);
for i=1:numel(var)
  eval(['global ',var{i}]);
  eval([var{i},' = current_globals.',var{i},';']);
end
