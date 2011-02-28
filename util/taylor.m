function varargout=taylor(fun,order,varargin)
% Takes basically any matlab function and returns the function + gradients:
% for a function
% [f1,f2,...,fn]=fun(a1,a2,..,an)
% you have
% [f1,f2,...,fn,df1,df2,...,dfn]=gradients(@fun,order,a1,a2,...,an)
% The implementation uses TaylorVar

% create TaylorVars out of the nominal inputs (no matter what the size)
avec=[];
for i=1:length(varargin), avec=[avec; varargin{i}(:)]; end
ta=TaylorVar(avec,order);
ind=0;
for i=1:length(varargin)
  n=prod(size(varargin{i}));
  a{i}=reshape(ta(ind+(1:n)),size(varargin{i}));
  ind=ind+n;
end
  
n=nargout(fun);
if (n>1) error('not implemented yet... but I just have to find the right syntax'); end

[varargout{1},varargout{2}]=eval(fun(a{:}));

if (order==1)  % just output the double matrix (not a cell) if we're just asking for the 1st order gradients
  varargout{2}=varargout{2}{1};
end

end
