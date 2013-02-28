function [x,F,xmul,Fmul,INFO] = hs13()
% HS 13 (modified so CQ holds)
%
%     Minimize        x(1) + x(2)
%
%     subject to      x(1)^3 - x(2)  >= 0
%
%                              x(2) >= 1.
%
%

snprint('hs13.out');  % By default, screen output is off;
hs13.spc = which('hs13.spc');
snspec (hs13.spc);
snseti ('Major Iteration limit', 250);

[x,xlow,xupp,xmul,xstate,Flow,Fupp,Fmul,Fstate,ObjAdd,ObjRow,    ...
   A,iAfun,jAvar,iGfun,jGvar] = hs13data;

[x,F,xmul,Fmul,INFO]= snsolve( x, xlow, xupp, xmul, xstate,    ...
                               Flow, Fupp, Fmul, Fstate,       ...
                               ObjAdd, ObjRow, A, iAfun, jAvar,...
                               iGfun, jGvar, 'hs13userfun');

snprint off;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x,xlow,xupp,xmul,xstate,Flow,Fupp,Fmul,Fstate,ObjAdd,ObjRow, ...
          A,iAfun,jAvar,iGfun,jGvar] = hs13data()

ObjRow = 1;
ObjAdd = 0;

x      = [ 3 1]';
xlow   = [-Inf,  1 ]';
xupp   = [ Inf, Inf]';
xmul   = zeros(2,1);
xstate = zeros(2,1);

Flow   = [ -Inf,   0]';
Fupp   = [  Inf, Inf]';
Fmul   = zeros(2,1);
Fstate = zeros(2,1);

%     ------------------------------------------------------------------
%     The nonzero pattern of the Jacobian is as follows:
%
%             Column
%            | 1   2
%            +--------
%     row 1  | G   G     Objective row
%         2  | G   G
%
%

A     = [];
iAfun = [];
jAvar = [];

G = [ 1,  1;
      1,  2;
      2,  1;
      2,  2 ];

iGfun = G(:,1); jGvar = G(:,2);
