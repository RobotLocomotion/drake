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
% If the output is a matrix (or ND-array), then the gradients are reshaped
% so that size(df) = [prod(size(f)), prod(size(a))]
%
% Higher order gradients are output as a q x r^o sparse matrix , where 
%    q is the dimension of the output, r is the dimension of the input, and o is the order.
%    e.g. 
%        d^2 f(i,j)/da1_k da1_l =
%             d2f(sub2ind(size(f),i,j),sub2ind([p p],k,l))
%        where p = prod(size(a1))
%
%</pre>
%
% @option grad_method {'user_then_taylorvar','user','taylorvar','numerical','symbolic'}
%         @default user_then_taylorvar
%    grad_method can also be a cell array of the strings above, in which
%    case geval will run both methods and compare the output.
%      e.g. option.grad_method = {'user','taylorvar'};

% todo: implement options? (without sacrificing too much performance?)
%% Options:
%%   grad_method:  {'user_then_taylorvar','user','taylorvar','numerical','symbolic'}
%         default is user_then_taylorvar

p=1;
if (isnumeric(fun)) 
  p=fun;
  fun=varargin{1};
  varargin={varargin{2:end}};
end

varargout=cell(1,nargout);

method = 12; % user if possible, otherwise taylorvar
if (isstruct(varargin{end}))
  options=varargin{end};
  varargin={varargin{1:end-1}};
  if (isfield(options,'grad_method'))
    if (iscell(options.grad_method))  % if the user supplies two grad methods, then run them both and compare the outputs
      gm=options.grad_method;
      options.grad_method=gm{1};
      [varargout{:}]=geval(p,fun,varargin{:},options);
      for i=2:length(gm)
        fdf = cell(1,nargout);
        options.grad_method=gm{i};
        if(~isfield(options,'tol'))
            options.tol = 1e-8;
        end
        [fdf{:}]=geval(p,fun,varargin{:},options);
        for j=1:nargout
          if ~valuecheck(fdf{j},varargout{j},options.tol)
            disp(['gradients generated by ',gm{1},' and ',gm{i},' don''t match']);
            % todo: put in some smart plotting here?
            fdf{j}-varargout{j}
            error(['gradients generated by ',gm{1},' and ',gm{i},' don''t match']);
          end
        end
      end
      return;
    end
      
    switch (lower(options.grad_method))
      case 'user'
        method=1;
      case 'taylorvar'
        method=2;
      case 'numerical'
        method=3;
      case 'symbolic'
        method=4;
      case 'user_then_taylorvar'
        method=12;
      case 'user_then_numerical'
        method=13;
      otherwise
        error('i don''t know what you''re talking about'); 
    end
  end
else
  options=struct();
end

if (method==12 || method == 13) % user then taylorvar or user then numerical
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
      return;  % if i get here, then it passed and I'm done
    catch exception
      if (strcmp(exception.identifier,'MATLAB:maxlhs') || strcmp(exception.identifier,'MATLAB:TooManyOutputs'))
        n=p;  % it failed, assume it only has the nominal outputs
      else  % it was some different error
        rethrow(exception);
      end
    end
  end
  if (n<nargout)
    method=method-10;
  else
    method=1;
  end
end

order=ceil(nargout/p)-1;

switch (method)
  case 1  % user
    [varargout{:}]=feval(fun,varargin{:});
    
  case 2  % taylorvar
    
    avec=[];
    for i=1:length(varargin),
      if (isa(varargin{i},'TaylorVar'))
        error('nested gevals not supported (at least not yet)');
      elseif (~isobject(varargin{i}) || isa(varargin{i},'sym'))
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
      if (isa(f{i},'TaylorVar'))
        [varargout{i:p:nargout}]=eval(f{i});
      elseif (isa(f{i},'double'))
        varargout{i}=f{i};
        for o=1:order  % return zeros for the gradients
          varargout{o*p+i}=zeros(prod(size(f{i})),ind^o);
        end
      else
        error('non-double outputs not supported (yet?)');
      end
    end
    
  case 3  % numerical

    if (order>1) error('higher order numerical gradients not implemented yet'); end
    
    [varargout{1:p}]=feval(fun,varargin{:});
    df_p = cell(1,p);
    df_m = cell(1,p);
    o = nargout-p;
    ind=1;
    
    if isfield(options,'da') 
      da = options.da;
    else
      da = 1e-7;
    end
    for i=1:length(varargin)
      if (isa(varargin{i},'double'))  %only handle doubles so far
        n=prod(size(varargin{i}));
        for j=1:n
          varargin{i}(j)=varargin{i}(j)+da;
          [df_p{:}]=feval(fun,varargin{:});
          varargin{i}(j)=varargin{i}(j)-2*da;
          [df_m{:}]=feval(fun,varargin{:});
          for k=1:o
            %varargout{p+k}(:,ind) = (df_p{k}-df_m{k})/(2*da);
            varargout{p+k}(:,ind) = (df_p{k}-varargout{k})/da;
          end
          varargin{i}(j)=varargin{i}(j)+da;
          ind=ind+1;
        end
      elseif ~isobject(varargin{i})
        warning(['input ',num2str(i), 'is not a double nor an object... I will not take gradients w/respect to this input']);
      end
    end
    
  case 4  % symbolic
    % note that this is inefficient (making a new symbolic thing each
    % time), but is very useful for debugging.
    
    s=[]; s0=[];
    for i=1:length(varargin)
      if (isobject(varargin{i}))
        a{i}=varargin{i};
      elseif (isnumeric(varargin{i}))
        if (exist('ai')), clear ai; end
        for j=1:prod(size(varargin{i}))
          ai(j)=sym(['a',num2str(i),'_',num2str(j)],'real');
        end
        a{i}=reshape(ai,size(varargin{i}));
        s=[s;ai(:)];
        s0=[s0;varargin{i}(:)];
      else
        error(['input ',num2str(i),' must be an object or numeric']);
      end
    end

    f = feval(fun,a{:});
    m = prod(size(f));
    n = length(s);
    
    varargout{1}=double(subs(f,s,s0));

    df{1} = jacobian(reshape(f,m,1),s);
    varargout{2}=double(subs(df{1},s,s0));
    for o=2:order
      df{o} = reshape(jacobian(reshape(df{o-1},m*n^(o-1),1),s),m,n^o);
      varargout{o+1}=double(subs(df{o},s,s0));
    end
    
  otherwise
    error('shouldn''t get here');
end    

