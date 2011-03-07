function varargout = geval(fun,varargin)
% GRADIENT FUNCTION EVAL - wraps any matlab function 'fun' to produce gradient outputs.
%   It will use user-supplied gradients when possible, and taylor
%   expansion gradients when necessary.
%
% For a single-output (+ user gradients) function
%      [f,df,d2f,...,dnf] = fun(a1,a2,a3,...)
% the call 
%      [f,df,d2f,...,dmf] = geval(fun,a1,a2,a3,...,options)
% If (m<=n), then it uses the user supplied gradients naturally output by
% the function. 
% If (m>n), then it performs a taylor expansion to compute *all* of the
% gradients. (so it doesn't currently pay to provide 2nd order gradients if
% you ask for 3rd order).
%
% For a function with p>1 different outputs (+ user gradients):
%      [f1,f2,...fp,df1,df2,...,dfp,d2f1,...,dmf1,...,dmfn] = fun(a1,a2,a3,...)
% use the notation 
%      geval(p,fun,varargin) 
% to tell geval that there are p different outputs.
% 

% todo: implement options? (without sacrificing too much performance?)
%% Options:
%%   grad_method:  {'user','numerical','taylorvar','symbolic'}

p=1;
if (isnumeric(fun)) 
  p=fun;
  fun=varargin{1};
  varargin={varargin{2:end}};
end

if (isstruct(varargin{end}))
  options=varargin{end};
  varargin={varargin{1:end-1}};
  if (isfield(options,'grad_method'))
    warning('grad_method option not implemented yet... (but soon!)'); 
  end
else
  options=struct();
end

varargout=cell(1,nargout);
try 
  n=nargout(fun);
catch
  % nargout isn't going to work for class methods
  n=-1;
end

if (n<0) % variable number of outputs
  % then just call it with the requested and catch if it fails:
  try
    [varargout{:}]=feval(fun,varargin{:});
  catch
    n=1;  % it failed, assume it only has one argument (so force the taylor var version)
  end
  if (n<1) % then it passed, and we're done
    return
  end
end

if (n>=nargout)
  [varargout{:}]=feval(fun,varargin{:});
else
  order=ceil(nargout/p)-1;

  avec=[];
  for i=1:length(varargin), 
      if (~isobject(varargin{i}) || isa(varargin{i},'sym'))
          avec=[avec; varargin{i}(:)];
      end
  end
  ta=TaylorVar.init(avec,order);
  ind=0;
  for i=1:length(varargin)
      if (~isobject(varargin{i}) || isa(varargin{i},'sym'))
          n=prod(size(varargin{i}));
          a{i}=reshape(ta(ind+(1:n)),size(varargin{i}));
          ind=ind+n;
      else
          a{i}=varargin{i};
      end
  end

  f=cell(1,p);
  [f{:}]=feval(fun,a{:});
  for i=1:p
      [varargout{1:p:nargout}]=eval(f{i});      
  end
end
