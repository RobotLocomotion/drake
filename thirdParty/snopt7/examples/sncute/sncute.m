function [x,F,xmul,Fmul,INFO, xstate, ...
	  Fstate, ns, ninf, sinf, ...
	  mincw, miniw, minrw] = sncute(varargin)
%Testing ... testing, 1, 2, 3.

global iGfun jGvar

usrd = 'userdata';
usrf = 'userfun';
conf = 'cuteobjcon';

if (nargin > 0 )
  probname = varargin{1};
  unix(['./compileMex ', probname]);
end

[x0, xlow, xupp, xmul, xstate, Flow, Fupp, Fmul, Fstate, ...
 ObjAdd, ObjRow] = feval( usrd );

n   = length(x0);
neF = length(Flow);

[A,iAfun,jAvar,iGfun,jGvar] = snJac(conf,x0,xlow,xupp,neF);

iGfun = [iAfun; iGfun];
jGvar = [jAvar; jGvar];
A = []; iAfun = []; jAvar = [];

solveopt = 1;
[x, F, xmul, Fmul, INFO, xstate, Fstate, ns, ninf, sinf, ...
 mincw, miniw, minrw] = ...
    snoptcmex( solveopt, x0, xlow, xupp, xmul, xstate, ...
	       Flow, Fupp, Fmul, Fstate, ...
	       ObjAdd, ObjRow, A, iAfun, jAvar,...
	       iGfun, jGvar, usrf );


