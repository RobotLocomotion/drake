function varargout = debugMexEval(fun,varargin)
% syntax is equavalent to feval, but this routine writes the inputs to a
% .mat file, which can be loaded and run from a standalone application to
% help debug mex files. 

persistent funmap;
if isempty(funmap), 
  funmap = containers.Map; 
end

typecheck(fun,'char');
if isKey(funmap,fun) 
  count = funmap(fun)+1; 
  appendstr = '-append';
else, 
  count=1; 
  appendstr = '';
end
countstr = num2str(count);  countstr=[repmat('0',1,5-length(countstr)),countstr];
funmap(fun)=count;

% note: it writes out consequitive calls with the notation nrhs_0001, etc.

eval(sprintf('nrhs_%s = length(varargin);',countstr));
eval(sprintf('nlhs_%s = nargout;',countstr));
eval(sprintf('varargin_%s = varargin;',countstr));
eval(sprintf('save([fun,''_mexdebug.mat''],''nrhs_%s'',''nlhs_%s'',''varargin_%s'',''%s'')',countstr,countstr,countstr,appendstr));

%if (nargout>0)
  varargout=cell(1:nargout);
  [varargout{:}] = feval(fun,varargin{:});
%else
%  feval(fun,varargin{:});
%end
