classdef TrigPoly 
% support class for parsing functions into their polynomial and
% trigonemetric components.  mostly passes all functionality through to
% msspoly, but adds handling of sin() and cos().
%
% example use:
%  t=msspoly('t',1);
%  x=TrigPoly('x','s','c',2);
%  u=msspoly('u',1);
%  p = PendulumPlant();
%  p.dynamics(t,x,u);

  properties 
    q   % simple msspoly with definition of q (polynomial variable)
    s   % simple msspoly with definition of s
    c   % simple msspoly with definition of c
    p   % msspoly with current poly value (in x,c,s);
  end
  
  methods
    function obj = TrigPoly(q,s,c,n)
      % @param q simple msspoly (vector) or char to represent the basic polynomial variable
      % @param s simple msspoly or char to represent sin(q)
      % @param c simple msspoly or char to represent cos(q)
      % @param n the length of q  (optional if q is an msspoly, otherwise required)
      
      if (nargin<4) n=-1; end
      
      function a=msspolyArg(a,varname)
        % note: can see/modify n from constructor
        if (isa(a,'msspoly'))
          if (~issimple(a)) error([varname, ' must be a simple msspoly']); end
          if (~isvector(a)) error([varname, ' must be a vector']); end
          na = length(a);
          if (n>0 && n~=na) error(['length(',varname,') must equal n']); end
          n=na;
        elseif ischar(a)
          if (n<0) error('n is mandatory unless q is an msspoly'); end
          a=msspoly(a,n);
        else
          error([varname,' must be a simple msspoly or a char']);
        end
      end        
        
      obj.q=msspolyArg(q,'q');
      obj.s=msspolyArg(s,'s');
      obj.c=msspolyArg(c,'c');
      
      obj.p=obj.q;
    end
    
    function p=getmsspoly(obj)
      p=obj.p;
    end
    
    function b=isTrigOrPoly(a)
      % returns true if the TrigPoly object a has
      % contributions from q(i) OR s(i),c(i), but not both.
      
      b=true;
      for i=1:length(a.q)
        if (deg(a.p,a.q(i))>0 && (deg(a.p,a.s(i))+deg(a.p,a.c(i)))>0)
          b=false; return;
        end
      end
    end
    
    function a=clean(a,tol)
      a.p=clean(a.p,tol);
    end
    
    function a=sin(a)
      for i=1:prod(size(a.p))  % handle one element at a time (for now).  could be vectorized.
        if (deg(a.p(i))==0) a.p(i)=sin(double(a.p(i))); return; end % return constant
        if (deg(a.p(i))>1) error('sin(p) with deg(p)>1 is not allowed'); end
        v=decomp(a.p(i));
      
        % consider just the first variable, and handle the rest through recursion.
        v=v(1);
        ind = match(a.q,v);
        if isempty(ind) error('non-trigpoly msspoly inside sin is not allowed'); end
      
        [R,p]=pdecomp(a.p(i),v);
        zeroind = find(p==0);
        oneind = find(p==1);  % must be a scalar coefficient (since deg==1)
        onecoef=double(R(oneind)); 
        if (onecoef==1)
          if (isempty(zeroind)) % just sin(q(ind))
            a.p(i)=a.s(ind); 
          else 
            a.p(i)=R(zeroind);
            a(i)=a.s(ind)*cos(a(i))+a.c(ind)*sin(a(i));
          end
        elseif (onecoef-floor(onecoef))==0  % sin(c*q(ind)+..)   do sin(q(ind) + (c-1)*q(ind)+...)
          if (isempty(zeroind))
            a.p(i)=(R(oneind)-sign(onecoef))*a.q(ind);
          else
            a.p(i)=(R(oneind)-sign(onecoef))*a.q(ind)+R(zeroind);
          end          
          a(i)=a.s(ind)*cos(a(i)) + a.c(ind)*sin(a(i));
        end
      end      
    end
    
    function a=cos(a)
      for i=1:prod(size(a.p))  % handle one element at a time (for now).  could be vectorized.
        if (deg(a.p(i))==0) a.p(i)=cos(double(a.p(i))); return; end % return constant
        if (deg(a.p(i))>1) error('cos(p) with deg(p)>1 is not allowed'); end
        v=decomp(a.p(i));
      
        % consider just the first variable, and handle the rest through recursion.
        v=v(1);
        ind = match(a.q,v);
        if isempty(ind) error('non-trigpoly msspoly inside cos is not allowed'); end
      
        [R,p]=pdecomp(a.p(i),v);
        zeroind = find(p==0);
        oneind = find(p==1);  % must be a scalar coefficient (since deg==1)
        onecoef=double(R(oneind)); 
        if (onecoef==1)
          if (isempty(zeroind)) % just sin(q(ind))
            a.p(i)=a.c(ind); 
          else 
            a.p(i)=R(zeroind);
            a(i)=a.c(ind)*cos(a(i)) - a.s(ind)*sin(a(i));
          end
        elseif (onecoef-floor(onecoef))==0  % sin(c*q(ind)+..)   do sin(q(ind) + (c-1)*q(ind)+...) 
          if (isempty(zeroind))
            a.p(i)=(R(oneind)-sign(onecoef))*a.q(ind);
          else
            a.p(i)=(R(oneind)-sign(onecoef))*a.q(ind)+R(zeroind);
          end
          a(i)=a.c(ind)*cos(a(i)) - a.s(ind)*sin(a(i));
        end
      end      
    end
    
    function i=end(a,k,n)
      % I don't think I should have to overload this, but it's not working
      % like (size(x,k)) should be.  see 'doc end'
      i=size(a,k);
    end
    
    function a=ctranspose(a)
      a.p=ctranspose(a.p);
    end

    function a=diag(a)
      a.p=diag(a.p);
    end
    
    function display(a)
      display(a.p);
    end
    
    function a=horzcat(varargin)
      ind=find(cellfun('isclass',varargin,'TrigPoly'),1);
      a=varargin{ind};
      for i=ind-1:-1:1
        a.p=horzcat(varargin{i},a.p);
      end
      for i=ind+1:length(varargin)
        if (isa(varargin{i},'TrigPoly'))
          a.p=horzcat(a.p,varargin{i}.p);
        else
          a.p=horzcat(a.p,varargin{i});
        end
      end
    end
    
    function b=isempty(a)
      b=isempty(a.p);
    end
    
    function b=isequal(a,b)
      if (isa(a,'TrigPoly'))
        if (isa(b,'TrigPoly'))
          b=isequal(a.p,b.p);
        else
          b=isequal(a.p,b);
        end          
      else % only b is a TrigPoly
        b=isequal(a,b.p);
      end
    end
    
    function b=isscalar(a)
      b=isscalar(a.p);
    end
    
    function n=length(a)
      n=length(a.p);
    end
    
    function a=minus(a,b)
      if (isa(a,'TrigPoly'))
        if (isa(b,'TrigPoly'))
          a.p=minus(a.p,b.p);
        else
          a.p=minus(a.p,b);
        end          
      else % only b is a TrigPoly
        b.p=minus(a,b.p);
        a=b;
      end
    end
    
    function a=mpower(a,n)
      a.p=mpower(a.p,n);
    end
    
    function a=power(a,n)
      if ~isnumeric(n) || any(size(n) ~= 1)
        error('Power only supports constant exponents')
      end
      a.p=mpower(a,n);
    end
    
    function a=mrdivide(a,b)
      if (isa(a,'TrigPoly'))
        if (isa(b,'TrigPoly'))
          a.p=mrdivide(a.p,b.p);
        else
          a.p=mrdivide(a.p,b);
        end
      else % only b is a TrigPoly
        b.p=mrdivide(a,b.p);
        a=b;
      end
    end
    
    function a=mtimes(a,b)
      if (isa(a,'TrigPoly'))
        if (isa(b,'TrigPoly'))
          a.p=mtimes(a.p,b.p);
        else
          a.p=mtimes(a.p,b);
        end
      else % only b is a TrigPoly
        b.p=mtimes(a,b.p);
        a=b;
      end
    end
    
    function a=plus(a,b)
      if (isa(a,'TrigPoly'))
        if (isa(b,'TrigPoly'))
          a.p=plus(a.p,b.p);
        else
          a.p=plus(a.p,b);
        end
      else % only b is a TrigPoly
        b.p=plus(a,b.p);
        a=b;
      end
    end
    
    function a=repmat(a,m,n)
      a.p=repmat(a.p,m,n);
    end
    
    function a=reshape(a,m,n)
      a.p=reshape(a.p,m,n);
    end
    
    function varargout=size(a,varargin)
      varargout=cell(1,nargout);
      [varargout{:}]=size(a.p,varargin{:});
    end
    
    function a=sparse(a)
      a.p=sparse(a.p);
    end
    
    function a=subsasgn(a,s,b)
      if (isa(b,'TrigPoly'))
        a.p=subsasgn(a.p,s,b.p);
      else
        a.p=subsasgn(a.p,s,b);
      end
    end
    
    function a=subsref(a,s)
      switch s.type
        case '()'
          a.p=subsref(a.p,s);
        otherwise
          error('not implemented'); 
      end
    end
    
    function a=sum(a,b)
      if (isa(a,'TrigPoly'))
        if (isa(b,'TrigPoly'))
          a.p=sum(a.p,b.p);
        else
          a.p=sum(a.p,b);
        end
      else % only b is a TrigPoly
        b.p=sum(a,b.p);
        a=b;
      end
    end
    
    function a=times(a,b)
      if (isa(a,'TrigPoly'))
        if (isa(b,'TrigPoly'))
          a.p=times(a.p,b.p);
        else
          a.p=times(a.p,b);
        end
      else % only b is a TrigPoly
        b.p=times(a,b.p);
        a=b;
      end
    end
    
    function a=trace(a)
      a.p=trace(a.p);
    end
    
    function a=transpose(a)
      a.p=transpose(a.p);
    end
     
    function a=uminus(a)
      a.p=uminus(a.p);
    end
    
    function a=uplus(a)
      a.p=uplus(a.p);
    end
    
    function a=vertcat(varargin)
      ind=find(cellfun('isclass',varargin,'TrigPoly'),1);
      a=varargin{ind};
      for i=ind-1:-1:1
        a.p=vertcat(varargin{i},a.p);
      end
      for i=ind+1:length(varargin)
        if (isa(varargin{i},'TrigPoly'))
          a.p=vertcat(a.p,varargin{i}.p);
        else
          a.p=vertcat(a.p,varargin{i});
        end
      end
    end
  end
end
