function [A,iAfun,jAvar,iGfun,jGvar] = testcute(prob)

if (nargin > 0)
  unix(['compileMex ',prob]);
end

options(1) =  1; % efirst  = true
options(2) =  0; % lfirst  = true
options(3) =  1; % nvfirts = true

[x0,xlow,xupp,lambda,clow,cupp,equatn,linear]=csetup(options);
[pname,xnames,gnames]=cnames; % This is initNames

F = cutefun(x0);
neF = length(F);

[A,iAfun,jAvar,iGfun,jGvar] = snJac('cutefun',x0,neF);
