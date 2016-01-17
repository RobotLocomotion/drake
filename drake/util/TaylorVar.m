classdef TaylorVar
% does inline autodiff
% overloads operators: http://www.mathworks.com/help/techdoc/matlab_oop/br02znk-1.html
%
% Use TaylorVar.init(...) to create a new TaylorVar.
%
% Use getmsspoly(...) to get the approximation around a point close to where
% you initialized the TaylorVar.
%
% the internal representation is
% m=prod(size(f)), n=prod(size(x));
% df{o} is a m x n^o sparse matrix , where e.g.
%        d^2 f(i,j)/dx_k dx_l =
%             df{o}(sub2ind(obj.dim,i,j),sub2ind([n n],k,l))

  properties
    f   % value at the nominal point
    df  % size(df{i}) is [size(f),repmat(size(x),1,i)]
    dim % size(f) - since I store things locally as a column vector
    nX   % size(x,1)
  end

  methods
    function obj=TaylorVar(f,df)
      obj.f = f(:);
      obj.dim = size(f);
      obj.df = df;
      obj.nX = size(df{1},2);
    end

    function varargout=eval(obj)
      varargout{1}=reshape(obj.f,obj.dim);

      for i=2:min(nargout,length(obj.df)+1)
        varargout{i}=obj.df{i-1};
      end

      % note: would love to reshape the df's into ND arrays, but matlab
      % can't handle sparse ND arrays.  and making them full could be very
      % inefficient!
    end

    function d=double(obj)
      d = reshape(obj.f,obj.dim);
      for o=1:length(obj.df)
        if (any(obj.df{o}(:)))
          warning('Drake:TaylorVar:DoubleConversion','converting taylorvar to double even though it has non-zero gradients.  gradient information will be lost!');
        end
      end
    end

    function p = getmsspoly(obj,p_xbar)
      % returns the taylor approximation as an msspoly
      % @param p_xbar should be relative to the original value of x used in
      % TaylorVar.init()

      if (~isvector(p_xbar)) error('p_xbar should be a vector'); end
      if (length(obj.dim)>2) error('msspolys are not defined for ND arrays'); end
      p_xbar=p_xbar(:); % make sure p_x is a column vector

      p=reshape(obj.f,obj.dim);
      nX=obj.nX;
      x=1;
      for o=1:length(obj.df)
        % x needs to be nX^o-by-1
        x=reshape(x(:)*p_xbar',nX^o,1)/o;
        p=p+reshape(obj.df{o}*x,obj.dim(1),obj.dim(2));
      end
    end
    
    function o = getOrder(obj)
      o = length(obj.df);
    end

    function classname=superiorfloat(varargin)
      classname='double';
    end
    function tf=isnan(obj)
      tf=isnan(reshape(obj.f,obj.dim));
    end
    function tf=isinf(obj)
      tf=isinf(reshape(obj.f,obj.dim));
    end
    function varargout=size(obj,dim)
      if (nargin>1)
        varargout{1}=obj.dim(dim);
      elseif (nargout>1)
        N=nargout;
        NDIMS=length(obj.dim);
        for i=1:N
          if (i>NDIMS) varargout{i}=1;
          else varargout{i}=obj.dim(i);
          end
        end
        if (N<NDIMS)
          vargout{N}=prod(obj.dim(N:end));
        end
      else
        varargout{1}=obj.dim;
      end
    end
    function numberOfElements = length(obj)
      numberOfElements = max(size(obj));
    end
    function numberOfElements = numel(obj,varargin)
      if (length(varargin)>0) error('not implemented yet');  end
      numberOfElements=prod(size(obj));
    end
    function lastind=end(obj,k,n)
      % end called as part of the kth index of n indices
      % 'help end' was the only way i found documentation on this
      if (n>length(obj.dim)) error('dimension mismatch'); end
      lastind=obj.dim(k);
    end
    function obj=repmat(obj,M,N)
      if nargin == 2
        if isscalar(M)
          siz = [M M];
        else
          siz = M;
        end
      else
        siz = [M N];
      end
      f_ind=1:length(obj.f);
      new_f_ind=repmat(reshape(f_ind,obj.dim),siz);
      obj.f=reshape(obj.f(new_f_ind),[],1);
      obj.dim=size(new_f_ind);
      for o=1:length(obj.df)
        obj.df{o}=obj.df{o}(new_f_ind,:);
      end
    end
    function obj=reshape(obj,varargin)
      if (nargin==2)
        siz=varargin{1};
      else
        % look for occurences of []
        emptyind = find(cellfun(@isempty,varargin));
        if (length(emptyind)>1)
          error('You can use only one occurrence of [].');
        elseif (length(emptyind)==1)
          varargin{emptyind}=prod(obj.dim)/prod([varargin{:}]);
          if rem(prod(obj.dim),prod([varargin{:}]))
            error('there is no integer size for the dimension specified by [] that would make prod(size) = prod(obj.dim)');
          end
        end
        siz=[varargin{:}];
      end
      if (prod(siz)~=prod(obj.dim))
        error('the number of elements must not change');
      end
      obj.dim=siz;
    end
    function c=bsxfun(fun,a,b)
      size_a = size(a);
      size_b = size(b);
      if (numel(size_a)~=numel(size_b))
        error('not implemented yet');
      end
      for i=1:numel(size_a)
        if (size_a(i)==1 && size_b(i)>1)
          d = 0*size_a+1; d(i)=size_b(i);
          a = repmat(a,d);
        elseif size_b(i)==1 && size_a(i)>1
          d = 0*size_a+1; d(i)=size_a(i);
          b = repmat(b,d);
        end
      end
      % make sure we insert into a TaylorVar
      if ~isa(a,'TaylorVar')
        c=b;
      else
        c=a;
      end
      for i=1:prod(size(a))
%       a(i)=feval(fun,a(i),b(i));  % want this, but can't call subsref
%       indirectly from inside the class, so do it the hard way
        si=substruct('()',{i});
        subsasgn(c,si,feval(fun,subsref(a,si),subsref(b,si)));
      end
    end

    function a=plus(a,b)
      if (~isa(a,'TaylorVar'))% then b is the TaylorVar.
        tmp=a; a=b; b=tmp;    % switch them (so we have less cases below)
      end

      if (isempty(a)) % handle the empty cases
        a=b; return;
      end
      if (isempty(b))
        return;
      end

      if (isa(b,'TaylorVar'))  % both are TaylorVars
        if (isscalar(a) && ~isscalar(b)) a=repmat(a,size(b)); end
        if (isscalar(b) && ~isscalar(a)) b=repmat(b,size(a)); end

        a.f=a.f + b.f;
        for o=1:length(a.df)
          a.df{o}=a.df{o}+b.df{o};
        end
      else  % b is a constant
        if (isscalar(a)&&~isscalar(b))
          a=repmat(a,size(b));
        end
        try
        a.f=reshape(reshape(a.f,a.dim)+b,[],1);
        catch
          keyboard;
        end
        % a.df doesn't change
      end
    end
    function tv=minus(a,b)
      tv=plus(a,uminus(b));
    end
    function a=uminus(a)
      a.f=-a.f;
      for o=1:length(a.df)
        a.df{o}=-a.df{o};
      end
    end
    function a=uplus(a)
      % intentionally empty
    end
    function a=times(a,b)  % .*
      if (~isa(a,'TaylorVar'))% then b is the TaylorVar.
        tmp=a; a=b; b=tmp;    % switch them (so we have less cases below)
      end

      if (isa(b,'TaylorVar')) % then both are taylorvars
        if (isscalar(a) && ~isscalar(b)) a=repmat(a,size(b)); end
        if (isscalar(b) && ~isscalar(a)) b=repmat(b,size(a)); end

        f=a.f .* b.f;
        m=prod(a.dim);
        ra = reshape(a,m,1); rb=reshape(b,m,1);
        dcdx = reshape(tvdiff(ra),m,a.nX).*repmat(reduceOrder(rb),1,a.nX) + ...
          repmat(reduceOrder(ra),1,a.nX).*reshape(tvdiff(b),m,b.nX);
        if (isa(dcdx,'TaylorVar'))
          a=tvint(dcdx,reshape(f,a.dim));
        else
          dcdx = reshape(dcdx,[],a.nX);
          a=TaylorVar(reshape(f,a.dim),{dcdx});
        end
      else
        if (isscalar(a) && ~isscalar(b)) a=repmat(a,size(b)); end
        if (isscalar(b) && ~isscalar(a)) b=repmat(b,size(a)); end

        b=b(:);
        a.f=a.f.*b;
        for o=1:length(a.df)
          a.df{o}=a.df{o}.*repmat(b,1,a.nX^o);
        end
      end
    end
    function a=mtimes(a,b)  % only allowed for scalars and 2D matrices
      if (~isa(a,'TaylorVar'))  % then only b is a TaylorVar
        [m,k] = size(a); [l,n]=size(b);
        if (m==1 && k==1) % handle scalar case
          f = a*reshape(b.f,b.dim);
          for o=1:length(b.df), b.df{o}=a*b.df{o}; end
        elseif (l==1 && n==1) %other scalar case
          f = a*b.f;
          for o=1:length(b.df), b.df{o}=repmat(a(:),1,b.nX^o).*repmat(b.df{o},numel(a),1); end
        else
          f = a*reshape(b.f,b.dim);
          for o=1:length(b.df)
            b.df{o} = reshape(a*reshape(b.df{o},k,n*b.nX^o),m*n,b.nX^o);
          end
        end
        a=b;
        a.f=f(:); a.dim=size(f);
      elseif (~isa(b,'TaylorVar')) % then only a is a TaylorVar
        a=(b'*a')';  % note: because of the way df is stored, this is potentially more efficient that trying to unroll df to right-multiply by b
      else % both are TaylorVars
        f = reshape(a.f,a.dim)*reshape(b.f,b.dim);
        if (length(a.df)<1 || length(b.df)<1) error('shouldn''t get here'); end
        % length of a.df and b.df should be >0 to get through the previous cases

        % want dcdx = dadx*b + a*dbdx.  tvdiff(a) gives dadx, but will be >2D for matrices,
        % so i have to handle the multiplies properly (as above)
        % dcdx is size [ma,nb,nX]
        ma=a.dim(1); na=a.dim(2); mb=b.dim(1); nb=b.dim(2); nX=a.nX;
        if isscalar(a)
          % tvdiff(a) will be [1,1,nX]
          dcdx = reshape(reshape(reduceOrder(b),mb*nb,1)*reshape(tvdiff(a),1,nX) + reduceOrder(a)*reshape(tvdiff(b),mb*nb,nX),[mb,nb,nX]);
        elseif isscalar(b)
          dcdx = reshape(reduceOrder(b)*reshape(tvdiff(a),ma*na,nX) + reshape(reduceOrder(a),ma*na,1)*reshape(tvdiff(b),1,nX),[ma,na,nX]);
        else
          % this version works, but is expensive
%          btmp = repmat({reduceOrder(b)},1,nX);
%          dcdx_slow = reshape(reshape(tvdiff(a),ma,na*nX)*blkdiag(btmp{:}),[ma,nb,nX]) + ...
%            reshape(reduceOrder(a)*reshape(tvdiff(b),mb,nb*nX),[ma,nb,nX]);
          % trying this version instead:
          dcdx=reshape(reduceOrder(a)*reshape(tvdiff(b),mb,nb*nX),[ma,nb,nX]);
          dadx=tvdiff(a);rb=reduceOrder(b);
          si=substruct('()',{':',':',1});
          for i=1:nX,
            % want:
            %   dcdx(:,:,i)=dcdx(:,:,i)+dadx(:,:,i)*rb;
            % but can't call subsref and subsasgn implicitly from
            % inside the class.  have to call it explicitly
            si.subs{end}=i;
            dcdx=subsasgn(dcdx,si,subsref(dcdx,si)+subsref(dadx,si)*rb);
          end
%          if ~isequal(dcdx,dcdx_slow), keyboard; end
          % end new version
        end
        if (isa(dcdx,'TaylorVar'))
          a=tvint(dcdx,f);
        else
          dcdx = reshape(dcdx,[],nX);
          a=TaylorVar(f,{dcdx});
        end
      end
    end
    function tv=rdivide(a,b)
      if (isscalar(a) && ~isscalar(b)) a=repmat(a,size(b)); end
      if (isscalar(b) && ~isscalar(a)) b=repmat(b,size(a)); end
      tv = a.*(b.^(-1));
    end
    function tv=ldivide(a,b)
      if (isscalar(a) && ~isscalar(b)) a=repmat(a,size(b)); end
      if (isscalar(b) && ~isscalar(a)) b=repmat(b,size(a)); end
      tv = (a.^(-1)).*b;
    end
    function a=mrdivide(a,b)
      if (~isa(b,'TaylorVar')) % then a is a TaylorVar, b is a const
        if (isscalar(b))
          a.f=a.f/b;
          for o=1:length(a.df), a.df{o}=a.df{o}/b; end
        else
          error('not implemented yet');
        end
      else
        a=a*inv(b);  % note: could do this better (more accurately)
      end
    end
    function tv=inv(a)
      % use d/dq inv(H(q)) = - inv(H(q)) [d/dq H(q)] inv(H(q))
      if (length(a.dim)>2) error('only for 2D matrices'); end
      [m,n]=size(a);nX=a.nX;
      if (m~=n) error('only for square matrices'); end
      f = inv(reshape(a.f,a.dim));
      inva = inv(reduceOrder(a));
      tmp=repmat({inva},1,nX);
      df = -reshape(inva*reshape(tvdiff(a),m,n*nX)*blkdiag(tmp{:}),m,n,nX);
      if (isa(df,'TaylorVar'))
        tv=tvint(df,f);
      else
        df = reshape(df,[],nX);
        tv=TaylorVar(f,{df});
      end
    end
    function tv=pinv(A)
      if rank(A.f)<A.dim(1)
        error('not implemented yet')
        % note: found matlab code for the gradient of pinv here:
        % http://onlinelibrary.wiley.com.libproxy.mit.edu/doi/10.1002/%28SICI%291097-0207%2819971130%2940:22%3C4211::AID-NME255%3E3.0.CO;2-8/epdf
      end
      tv=inv(A);
    end

    function tv=mldivide(a,b)
      tv=inv(a)*b;
      % todo: could make this better (more efficient/accurate?) if I handle
      % all the cases to use the actual \ call.
    end
    function tv=power(a,k)
      if (isa(k,'TaylorVar')) error('not implemented yet'); end
      if (k==0)
        tv=ones(a.dim);  % just a constant
      elseif (k==1)
        tv = a;
      else
        tv=elementwise(a,@(x)power(x,k),@(x) k.*x.^(k-1));
      end
    end
    function a=mpower(a,k)
      if (isa(k,'TaylorVar')) error('not implemented yet'); end
      % only defined for square matrices (including scalars)
      if (k==0)
        a=eye(a.dim(1));  % just a constant
      elseif (k>0)
        a=a*a^(k-1);  % compute gradients using mtimes (at least for now)
      else
        error('not implemented yet');
      end
    end

    function l=lt(a,b)
      if (isa(a,'TaylorVar'))
        a=reshape(a.f,a.dim);
      end
      if (isa(b,'TaylorVar'))
        b=reshape(b.f,b.dim);
      end
      l=lt(a,b);
    end
    function l=gt(a,b)
      if (isa(a,'TaylorVar'))
        a=reshape(a.f,a.dim);
      end
      if (isa(b,'TaylorVar'))
        b=reshape(b.f,b.dim);
      end
      l=gt(a,b);
    end
    function l=le(a,b)
      if (isa(a,'TaylorVar'))
        a=reshape(a.f,a.dim);
      end
      if (isa(b,'TaylorVar'))
        b=reshape(b.f,b.dim);
      end
      l=le(a,b);
    end
    function l=ge(a,b)
      if (isa(a,'TaylorVar'))
        a=reshape(a.f,a.dim);
      end
      if (isa(b,'TaylorVar'))
        b=reshape(b.f,b.dim);
      end
      l=ge(a,b);
    end
    function l=ne(a,b)
      if (isa(a,'TaylorVar'))
        a=reshape(a.f,a.dim);
      end
      if (isa(b,'TaylorVar'))
        b=reshape(b.f,b.dim);
      end
      l=ne(a,b);
    end
    function l=eq(a,b)
      if (isa(a,'TaylorVar'))
        a=reshape(a.f,a.dim);
      end
      if (isa(b,'TaylorVar'))
        b=reshape(b.f,b.dim);
      end
      l=eq(a,b);
    end
    function l=sign(a)
      a=reshape(a.f,a.dim);
      l=sign(a);  % no grad info
    end
    function l=and(a,b)
      if (isa(a,'TaylorVar'))
        a=reshape(a.f,a.dim);
      end
      if (isa(b,'TaylorVar'))
        b=reshape(b.f,b.dim);
      end
      l=and(a,b);
    end
    function l=or(a,b)
      if (isa(a,'TaylorVar'))
        a=reshape(a.f,a.dim);
      end
      if (isa(b,'TaylorVar'))
        b=reshape(b.f,b.dim);
      end
      l=or(a,b);
    end
    function l=not(a)
      a=reshape(a.f,a.dim);
      l=not(a)
    end
    function b=any(a)
      b=any(a.f);
    end

    function a=ctranspose(a)
      if (~any(imag(a.f)))  % strictly real
        a=transpose(a);
      else
        error('not implemented yet');
      end
    end
    function a=transpose(a)
      map = reshape(reshape(1:prod(a.dim),a.dim)',[],1);  % easy (but inefficient) way to figure out the index map
      a.f = a.f(map);
      a.dim = [a.dim(2),a.dim(1)];  % transpose on ND arrays is not defined
      for o=1:length(a.df)
        a.df{o}=a.df{o}(map,:);
      end
    end

%    function display(a)
%    end

    function tv = horzcat(varargin)
      % find the index of the first TaylorVar
      for i=1:length(varargin), if (isa(varargin{i},'TaylorVar')), obj=varargin{i}; break; end; end
      nX=obj.nX; order=length(obj.df);

      % NOTE: use df' instead of df internally here (for efficiency), then
      % convert it back at the end

      f=[];
      for o=1:order, df{o}=sparse(nX^o,0); end

      for i=1:length(varargin)
        if (isa(varargin{i},'TaylorVar'))
          % figure out indices where the new data will be inserted:
          oldtag=zeros(size(f));
          newtag=ones(varargin{i}.dim);
          tags=[oldtag,newtag];
          oldinds=find(~tags(:));
          inds=find(tags(:));

          % now insert the data
          f=[f,reshape(varargin{i}.f,varargin{i}.dim)];
          if (varargin{i}.nX ~=nX) error('dimension mismatch'); end
          if (length(varargin{i}.df) ~= order) error('order mismatch'); end
          for o=1:order
            df{o}(:,oldinds)=df{o};
            df{o}(:,inds)=varargin{i}.df{o}';
          end
        else  % vertcat in a const
          % figure out indices where the new data will be inserted:
          oldtag=zeros(size(f));
          newtag=ones(size(varargin{i}));
          tags=[oldtag,newtag];
          oldinds=find(~tags(:));
          inds=find(tags(:));

          % now insert the data
          f=[f,varargin{i}];
          n=prod(size(varargin{i}));
          for o=1:order
            df{o}(:,oldinds)=df{o};
            df{o}(:,inds)=sparse(nX^o,n);
          end
        end
      end
      for o=1:order, df{o}=df{o}'; end
      tv=TaylorVar(f,df);
    end
    function tv = vertcat(varargin)
      % find the index of the first TaylorVar
      for i=1:length(varargin), if (isa(varargin{i},'TaylorVar')), tvi=i; break; end; end
      nX=varargin{tvi}.nX; order=length(varargin{tvi}.df);

      f=[];
      for o=1:order, df{o}=sparse(nX^o,0); end

      for i=1:length(varargin)
        if (isa(varargin{i},'TaylorVar'))
          % figure out indices where the new data will be inserted:
          oldtag=zeros(size(f));
          newtag=ones(varargin{i}.dim);
          tags=[oldtag;newtag];
          oldinds=find(~tags(:));
          inds=find(tags(:));

          % now insert the data
          f=[f;reshape(varargin{i}.f,varargin{i}.dim)];
          if (varargin{i}.nX ~=nX) error('dimension mismatch'); end
          if (length(varargin{i}.df) ~= order) error('order mismatch'); end
          for o=1:order
            df{o}(:,oldinds)=df{o};
            df{o}(:,inds)=varargin{i}.df{o}';
          end
        else  % vertcat in a const
          % figure out indices where the new data will be inserted:
          oldtag=zeros(size(f));
          newtag=ones(size(varargin{i}));
          tags=[oldtag;newtag];
          oldinds=find(~tags(:));
          inds=find(tags(:));

          % now insert the data
          f=[f;varargin{i}];
          n=prod(size(varargin{i}));
          for o=1:order
            df{o}(:,oldinds)=df{o};
            df{o}(:,inds)=sparse(nX^o,n);
          end
        end
      end
      for o=1:order, df{o}=df{o}'; end
      tv=TaylorVar(f,df);
    end

    function obj = subsref(obj,s)
      % figure out indices corresponding to s
      tags = reshape(1:length(obj.f),obj.dim);
      ind = subsref(tags,s);
      if (isempty(ind)) obj=zeros(size(ind)); return; end
      obj.dim = size(ind);
      ind = ind(:);

      obj.f = obj.f(ind);
      % extract the relevant gradients
      for o=1:length(obj.df)
        obj.df{o}=obj.df{o}(ind,:);
      end
    end

    function a=subsasgn(a,s,b)
      if (~isa(a,'TaylorVar'))  % then turn it into one
        n = numel(a);
        for o=1:length(b.df)  % b must be a taylorvar
          da{o} = sparse(n,(b.nX)^o);
        end
        a=TaylorVar(a,da);
      end

      function cleanup_subs
        % turn all subsrefs into A(ind)=B(:), instead of A(sub1,sub2)=B.
        for k=1:length(s.subs)
          if (strcmp(s.subs{k},':'))
            s.subs{k}=1:a.dim(k);
          end
        end
        inds = reshape(1:prod(a.dim),a.dim);
        s.subs = {reshape(subsref(inds,s),[],1)};
      end

      if (isa(b,'TaylorVar'))
        % handle the case where the assignment will make the matrix bigger
        f = subsasgn(reshape(a.f,a.dim),s,reshape(b.f,b.dim));
        a.f = reshape(f,numel(f),1);
        cleanup_subs();
        s.subs{end+1}=':';
        for o=1:length(a.df)
          a.df{o} = subsasgn(a.df{o},s,b.df{o});
        end
        a.dim = size(f);
      else % b is a const or empty
        f = subsasgn(reshape(a.f,a.dim),s,b);
        a.f = reshape(f,numel(f),1);
        cleanup_subs();
        s.subs{end+1}=':';
        if (isempty(b)), b_df = []; else, b_df = 0; end
        for o=1:length(a.df)
            a.df{o} = subsasgn(a.df{o},s,b_df);
        end
        a.dim = size(f);
      end
    end
    function ind=subsindex(a)
      ind = reshape(a.f,a.dim);  % just return the nominal value to be used as an index
    end

    function tv = sin(obj)
      tv=elementwise(obj,@sin,@cos);
    end
    function tv = cos(obj)
      tv=elementwise(obj,@cos,@(x)-sin(x));
    end
    function tv = sec(obj)
      tv=elementwise(obj,@sec,@(x)sec(x).*tan(x));
    end
    function tv = tan(obj)
      tv=elementwise(obj,@tan,@(x)sec(x).^2);
    end
    function tv = asin(obj)
      tv=elementwise(obj,@asin,@(x) ones(size(x))./sqrt(1-x.^2));
    end
    function tv = acos(obj)
      tv=elementwise(obj,@acos,@(x) -ones(size(x))./sqrt(1-x.^2));
    end
    function tv = atan(obj)
      tv=elementwise(obj,@atan,@(x) ones(size(x))./(1+x.^2));
    end
    function tv=atan2(y,x)
      tv=elementwisetwoarg(y,x,@atan2,@(y,x) x./max(x.^2+y.^2,eps),@(y,x) -y./max(x.^2+y.^2,eps));
    end
    function tv = acot(obj)
      tv=elementwise(obj,@acot,@(x) -ones(size(x))./(1+x.^2));
    end
    function tv = tanh(obj)
      tv=elementwise(obj,@tanh,@(x) ones(size(x))-tanh(x).^2);
    end
    function tv = exp(obj)
      tv=elementwise(obj,@exp,@exp);
    end
    function a=abs(a)
      s=sign(a.f);
      a.f=abs(a.f);
      for o=1:length(a.df)
        a.df{o}=a.df{o}.*repmat(s,1,a.nX^o);
      end
    end


    function tv = diag(v,k)
      % v is a TaylorVar
      if (nargin>1) error('k handling not implemented yet'); end
      if (length(v.dim)>2) error('only for vectors and 2D matrices'); end
      if (v.dim(1)==1 || v.dim(2)==1)  % vector in, create matrix
        n=max(v.dim);
        ind=sub2ind([n,n],1:n,1:n);
        f=diag(v.f);
        for o=1:length(v.df)
          df{o}=sparse(n*n,v.nX^o);
          df{o}(ind,:)=v.df{o};
        end
      else  % matrix in, strip out diagonal vector
        n=min(v.dim);
        ind=sub2ind(v.dim,1:n,1:n);
        f=v.f(ind);
        for o=1:length(v.df)
          df{o}=v.df{o}(ind,:);
        end
      end
      if (exist('df'))
        tv=TaylorVar(f,df);
      else
        tv=f;
      end
    end

    function X=mod(X,Y)
      if (isa(Y,'TaylorVar'))
        error('not implemented yet'); % but shouldn't be hard
      else % X is TaylorVar, Y is a const
        X.f = reshape(mod(reshape(X.f,X.dim),Y),[],1);
        % no change to gradients
      end
    end

    function a=conj(a)
      % intentionally left blank.  i've assumed real everywhere in here.
    end

    function a=sqrt(a)
      a=a.^(1/2);
    end

    function a=sum(a,d)
      if (nargin<2)
        d = min(find(a.dim~=1));
        if isempty(d), d = 1; end
      end
      a.f = sum(reshape(a.f,a.dim),d);
      newdim = size(a.f);
      a.f = a.f(:);

      for o=1:length(a.df)
        a.df{o}=sparse(reshape(sum(reshape(full(a.df{o}),[a.dim,a.nX^o]),d),[prod(newdim),a.nX^o]));
      end
      a.dim = newdim;
    end

    function [a,i]=max(a,b,dim)
      if (nargin>2) error('not implemented yet'); end
      if (nargin<2)
        if (~isvector(a)) error('not implemented yet'); end
        [v,i] = max(a.f);
        si=substruct('()',{i});
        a=subsref(a,si);
      else
        if (isscalar(a) && ~isscalar(b)) a=repmat(a,size(b)); end
        if (isscalar(b) && ~isscalar(a)) b=repmat(b,size(a)); end
        if (~isa(a,'TaylorVar'))% then b is the TaylorVar.
          tmp=a; a=b; b=tmp;    % switch them (so we have less cases below)
        end
        if (isa(b,'TaylorVar')) % then both are TaylorVars
          error('not implemented yet');
        else % then just a is a TaylorVar
          c = reshape(max(reshape(a.f,a.dim),b),[],1);
          ind=find(c~=a.f);
          for o=1:length(a.df)
            a.df{o}(ind,:)=0;
          end
          a.f=c;
        end
      end
    end

    function [a,i]=min(a,b,dim)
      if (nargin>2) error('not implemented yet'); end
      if (nargin<2) % then just look over the TaylorVar a
        if (isvector(a))
          [m,i] = min(a.f);
          a = subsref(a,substruct('()',{i}));
          return;
        else
          error('not implemented yet');
        end
      end
      if (isscalar(a) && ~isscalar(b)) a=repmat(a,size(b)); end
      if (isscalar(b) && ~isscalar(a)) b=repmat(b,size(a)); end
      if (~isa(a,'TaylorVar'))% then b is the TaylorVar.
        tmp=a; a=b; b=tmp;    % switch them (so we have less cases below)
      end
      if (isa(b,'TaylorVar')) % then both are TaylorVars
        c = reshape(min(reshape(a.f,a.dim),reshape(b.f,b.dim)),[],1);
        ind=find(c~=a.f);
        for o=1:length(a.df)
          a.df{o}(ind,:)=b.df{o}(ind,:);
        end
        a.f=c;
      else % then just a is a TaylorVar
        c = reshape(min(reshape(a.f,a.dim),b),[],1);
        ind=find(c~=a.f);
        for o=1:length(a.df)
          a.df{o}(ind,:)=0;
        end
        a.f=c;
      end
    end

    function B = cumprod(A,dim)
      if (nargin<2) dim=1; end
      if (dim~=1) error('not implemented yet'); end
      if (~ismatrix(A)) error('not implemented yet'); end
      B = A;
      for i=2:size(A,1)
        % B(i,:)=B(i-1,:).*A(i,:);  % can't call subsref indirectly from
        % inside the class, so call it explicitly:
        si=substruct('()',{i,':'});
        sim=substruct('()',{i-1,':'});
        subsasgn(B,si,subsref(B,sim).*subsref(A,si));
      end
    end

    function n=norm(A,p)
      if (~isvector(A)) error('not implemented yet'); end
      if (nargin<2) p=2; end
      if (isinf(p))
        if (p>0) n=max(abs(A));
        else     n=min(abs(A));
        end
      elseif p==2  % break out special case just because mpower isn't full implemented yet
        n=sqrt(sum(A.^2));
      else
        % will error until I complete mpower
        n=sum(abs(A).^p)^(1/p);
      end
    end

    function y = diff(x,n,dim)
      if (nargin>1 && n~=1) error('higher order diff not implemented yet - but should be trivial with a recursion'); end

      if (nargin<3) % then operate along first non-singleton dimension
        dim=find(x.dim>1,1);
      end
      if isempty(dim)
        y=[];
        return;
      end

      m=x.dim(dim);
      s1=substruct('()',[repmat({':'},1,dim-1),[2:m],repmat({':'},1,length(x.dim)-dim)]);
      s2=substruct('()',[repmat({':'},1,dim-1),[1:m-1],repmat({':'},1,length(x.dim)-dim)]);

      y = subsref(x,s1)-subsref(x,s2);
    end

    function v = ppval(pp,xx)
      % equivalent to ppval.m, but made taylorvar compatible

      %  obtain the row vector xs equivalent to XX
      sizexx = size(xx); lx = numel(xx); xs = reshape(xx,1,lx);
      %  if XX is row vector, suppress its first dimension
      if length(sizexx)==2&&sizexx(1)==1, sizexx(1) = []; end

      % take apart PP
      [b,c,l,k,dd] = unmkpp(pp);

      % for each evaluation site, compute its breakpoint interval
      % (mindful of the possibility that xx might be empty)
      index = sum(repmat(xs.f,1,length(b))>=repmat(b,lx,1),2);

      % now go to local coordinates ...
      xs = xs-b(index);

      d = prod(dd);
      if d>1 % ... replicate xs and index in case PP is vector-valued ...
        xs = reshape(repmat(xs,d,1),1,d*lx);
        index = d*index; temp = (-d:-1).';
        index = reshape(1+index(ones(d,1),:)+temp(:,ones(1,lx)), d*lx, 1 );
      else
        if length(sizexx)>1, dd = []; else dd = 1; end
      end
      v = c(index,1);
      for i=2:k
        v = xs'.*v + c(index,i);
      end
      v = reshape(v,[dd,sizexx]);

    end


%    function obj=blkdiagcpy(obj,c)
%      if (length(obj.dim)~=2) error('only for 2D matrices'); end
%      m=obj.dim(1);n=obj.dim(2);
%      a=reshape(obj.f,obj.dim);
%      s=sparse([],[],[],m*c,n*c,c*m*n);
%      for i=0:c-1
%        s(i*m+(1:m),i*n+(1:n))=a;
%      end
%      ind=find(s);
%      obj.f=s(:);
%      obj.dim=[m*c,n*c];
%      error('need to finish df terms');
%    end
  end

  methods (Access=private)

    function tv=tvdiff(obj)  % removes one order  (tv.f=obj.df{1}, etc)
      f=reshape(full(obj.df{1}),[obj.dim,obj.nX]);
      m=prod(obj.dim); n=obj.nX;
      if (length(obj.df)>1)
        for o=2:length(obj.df)
          df{o-1}=reshape(obj.df{o},m*n,n^(o-1));
        end
        tv=TaylorVar(f,df);
      else
        tv=f;
      end
    end
    function tv=tvint(obj,f) % tv.f=f tv.df{1}=obj.f tv.df{2}=obj.df{1}, etc
      dim=size(f);
      if (obj.dim(end)~=obj.nX)
        error('dimension mismatch');
      end
      df{1}=reshape(sparse(obj.f),[prod(dim),obj.nX]);
      m=prod(obj.dim(1:end-1)); n=obj.nX;
      for o=1:length(obj.df)
        df{o+1}=reshape(obj.df{o},[m,n^(o+1)]);
      end
      tv=TaylorVar(f,df);
    end
    function obj=reduceOrder(obj,order)
      if (nargin<2) order=1; end
      if (order>=length(obj.df)) % then just return f as a double (or whatever)
        obj=reshape(obj.f,obj.dim);
      else
        obj.df = {obj.df{1:end-order}};
      end
    end
    function a=elementwise(a,fun,dfun)  % fun and dfun must be vectorized
      f=fun(a.f);
      if (length(a.df)>0)
        df=tvdiff(a);
        dfsub=repmat(dfun(reduceOrder(a)),[0*a.dim+1,a.nX]).*df;

        % dim=size(df);
        %        dfsub=repmat(dfun(reduceOrder(obj)),[1,dim(2:end)]).*df;
        %        dfsub=diag(dfun(reduceOrder(obj)))*tvdiff(obj);
        if (isa(dfsub,'TaylorVar'))
          a=tvint(dfsub,reshape(f,a.dim));
        else
          a.f = f;
          a.df = {reshape(dfsub,[],a.nX)};
        end
      else
        a=reshape(f,a.dim);
      end
    end
    function a=elementwisetwoarg(a,b,fun,dfunda,dfundb)
      if (~isa(b,'TaylorVar')) error('not implemented yet'); end
      if (length(a.df)~=length(b.df)) error('not implemented yet'); end
      if (any(size(a)~=size(b))) error('not implemented yet'); end
      f=fun(a.f,b.f);
      if (length(a.df)>0)
        da=tvdiff(a);
        db=tvdiff(b);

        dfsub=repmat(dfunda(reduceOrder(a),reduceOrder(b)),[0*a.dim+1,a.nX]).*da + repmat(dfundb(reduceOrder(a),reduceOrder(b)),[0*b.dim+1,b.nX]).*db;
        if (isa(dfsub,'TaylorVar'))
          a=tvint(dfsub,reshape(f,a.dim));
        else
          a.f = f;
          a.df = {reshape(dfsub,[],a.nX)};
        end
      else
        a=reshape(f,a.dim);
      end
    end
  end

  methods (Static)
    function tv=init(x,order)
      dim=size(x);
      if (length(dim)>2 || dim(2)~=1), error('x must be a column vector'); end
      n = length(x);
      dx{1} = eye(n);
      for o=2:order
        dx{o} = sparse(n,n^o);
      end
      tv=TaylorVar(x,dx);
    end
  end

end
