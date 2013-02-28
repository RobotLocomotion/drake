function [x,F,xmul,Fmul,INFO] = sntoy()
%Mimics sntoyA.f in $SNOPT/examples
%
%     Minimize      3*x(1) + (x(1) + x(2) + x(3))^2 + 5*x(4)
%
%     subject to             4*x(2)   + 2*x(3)               >= 0
%                     x(1) +   x(2)^2 +   x(3)^2              = 2
%                              x(2)^4 +   x(3)^4   +   x(4)   = 4
%
%                     x(1) >= 0,                       x(4) >= 0.
%
%

snprint('sntoy.out');  % By default, screen output is off;
sntoy.spc = which('sntoy.spc');
snspec (sntoy.spc);
snseti ('Major Iteration limit', 250);

[x,xlow,xupp,xmul,xstate,Flow,Fupp,Fmul,Fstate,ObjAdd,ObjRow,    ...
   A,iAfun,jAvar,iGfun,jGvar] = toydata;

[x,F,xmul,Fmul,INFO]= snsolve( x, xlow, xupp, xmul, xstate,    ...
                               Flow, Fupp, Fmul, Fstate,       ...
                               ObjAdd, ObjRow, A, iAfun, jAvar,...
                               iGfun, jGvar, 'toyusrfun');

snprint off;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x,xlow,xupp,xmul,xstate,Flow,Fupp,Fmul,Fstate,ObjAdd,ObjRow, ...
          A,iAfun,jAvar,iGfun,jGvar] = toydata()

ObjRow = 1;
ObjAdd = 0;

x      = ones(4,1);
xlow   = [  0,-Inf,-Inf,   0]';
xupp   = [Inf, Inf, Inf, Inf]';
xmul   = zeros(4,1);
xstate = zeros(4,1);

Flow   = [ -Inf,   0, 2, 4]';
Fupp   = [  Inf, Inf, 2, 4]';
Fmul   = zeros(4,1);
Fstate = zeros(4,1);

%     ------------------------------------------------------------------
%     The nonzero pattern of the Jacobian is as follows:
%
%              Column
%            | 1   2    3    4
%            +------------------
%     row 1  | G   G    G    G    Objective row
%         2  |     G    G
%         3  | G   G    G
%         4  |     G    G    G
%
%

A     = [];
iAfun = [];
jAvar = [];

G = [ 1,  1;
      1,  2;
      1,  3;
      1,  4;
      2,  2;
      2,  3;
      3,  1;
      3,  2;
      3,  3;
      4,  2;
      4,  3;
      4,  4 ];

iGfun = G(:,1); jGvar = G(:,2);
