function [x0, xlow, xupp, xmul, xstate, ...
	  Flow, Fupp, Fmul, Fstate, ...
	  ObjAdd, ObjRow] = userdata( )

global problemDefinition

options(1) =  1; % efirst  = true
options(2) =  0; % lfirst  = true
options(3) =  1; % nvfirts = true

if isempty(problemDefinition)
  [x0,xlow,xupp,lambda,clow,cupp,equatn,linear]=csetup(options);
  [pname,xnames,gnames]=cnames; % This is initNames
else
  [x0,xlow,xupp,lambda,clow,cupp,equatn,linear] =...
      feval(problemDefinition, 0, options);
end


n      = length(x0);
nc     = length(lambda);
neF    = nc+1;
xmul   = zeros(n,1);
xstate = zeros(n,1);
Flow   = [-1e20; clow];
Fupp   = [ 1e20; cupp];

Fmul   = zeros(neF,1);
Fstate = zeros(neF,1);
ObjAdd = 0;
ObjRow = 1;


