function [x,y,info] = mysedumi(A,b,c,K,pars)

% todo: handle optional input arguments

[x,y,info] = sedumi(A,b,c,K,pars);

if (info.numerr)  % try running one more time with zero elements removed
  tol = 1e-6;
  keep = (abs(x)>=tol);

  b=0;
  if isfield(K,'f')
    a=1;b=b+K.f;
    K.f = sum(keep(a:b));
  end
  if isfield(K,'l')
    a=b+1;b=b+K.l;
    K.l = sum(keep(a:b));
  end
  if isfield(K,'q')
    for i=1:numel(K.q)
      a=b+1;b=b+K.q(i);
      if (zeroind(a)) error('have to handle this case'); end
      K.q(i)=sum(keep(a:b));
    end
  end
  if isfield(K,'r')
    for i=1:numel(K.r)
      a=b+1;b=b+K.r(i);
      if (zeroind(a) || zeroind(a+1)) error('have to handle this case'); end
      K.r(i)=sum(keep(a:b));
    end
  end
  if isfield(K,'s')
    for i=1:numel(K.s)
      a=b+1;b=b+K.s(i);
      K.s(i)=sum(keep(a:b));
    end
  end
  
  A=A(:,keep);
  C=C(keep);
  
  [x(keep),y,info]=sedumi(A,B,optimize*C,K,pars);
  x(~keep) = 0;
  
  if (info.numerr<1)
    disp('yeah. my trick worked!');
  end
end
