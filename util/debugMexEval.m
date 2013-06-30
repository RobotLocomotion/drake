function varargout = debugMexEval(fun,varargin)
% syntax is equavalent to feval, but this routine writes the inputs to a
% .mat file, which can be loaded and run from a standalone application to
% help debug mex files. 
%
% Note: on mac, I needed to do the following to run debugMex from the
% command line:
%  export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:/Applications/MATLAB_R2012a.app/bin/maci64


%
% TODO:
%
% support multiple mex files from a single program
%   write data to a single location instead of one named after a particular
%   mex file
%   write out path to the mex file so that the debuMex script doesn't have
%   to be in any single directory.
%
% *first* arg in is structure implies options
%   input_pointer_mask
%   output_pointer_mask
% debugMex executable then keeps a global table and fills in the right
% pointer.  will need to handle the case that this pointer does not exists,
% and give a useful warning so that people put a debugMexEval around the
% function that creates that pointer, as well. 
%
% Alternatively, if I replace my use of SharedDataHandle with matlab's
% lib.pointer class:
% http://www.mathworks.com/help/matlab/matlab_external/working-with-pointers.html
% then I could do everything automagically (without requiring the use of
% additional arguments)
%
% 
% if one of the arguments is a class, then convert it to a structure before
% saving.  debugMex executable then overloads the mxGetProperty method 
% using this technique: 
%   http://www.ibm.com/developerworks/library/l-glibc/index.html
%   https://blogs.oracle.com/DatabaseEmporium/entry/where_is_ld_preload_under


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

% convert classes to structures, so that they can be loaded without the
% matlab engine attached.  (note: i don't feel like I should have to do
% this!)



% note: it writes out consequitive calls with the notation nrhs_0001, etc.

eval(sprintf('nrhs_%s = length(varargin);',countstr));
eval(sprintf('nlhs_%s = nargout;',countstr));
eval(sprintf('varargin_%s = varargin;',countstr));
eval(sprintf('save([fun,''_mexdebug.mat''],''nrhs_%s'',''nlhs_%s'',''varargin_%s'',''%s'')',countstr,countstr,countstr,appendstr));

varargout=cell(1:nargout);
[varargout{:}] = feval(fun,varargin{:});
