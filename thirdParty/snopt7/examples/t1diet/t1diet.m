function [x,F,xmul,Fmul,INFO] = t1diet()

snprint('t1diet.out');

t1diet.spc = which('t1diet.spc');
snspec ( t1diet.spc );

[x,xlow,xupp,xmul,xstate,Flow,Fupp,Fmul,Fstate,ObjAdd,ObjRow,    ...
                                 A,iAfun,jAvar,iGfun,jGvar] = dietdata;

[x,F,xmul,Fmul,INFO]= snsolve( x, xlow, xupp, xmul, xstate,    ...
			       Flow, Fupp, Fmul, Fstate,       ...
			       ObjAdd, ObjRow, A, iAfun, jAvar,...
			       iGfun, jGvar, 't1dietusrfun');
snprint off;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x,xlow,xupp,xmul,xstate,Flow,Fupp,Fmul,Fstate,ObjAdd,ObjRow, ...
          A,iAfun,jAvar,iGfun,jGvar] = dietdata()
%
%     dietdata defines input data for the Diet LP problem of Chvatal, 1983.
%
%
%           ( 110  205  160  160  420  260 )
%     A  =  (   4   32   13    8    4   14 )
%           (   2   12   54  285   22   80 )
%           (   3   24   13    9   20   19 ) ( = objective row c')
%

neF    = 4;
n      = 6;
Obj    = 4;
ObjRow = 4;

G = []; iGfun = []; jGvar = [];

% Assign the constant part of the Jacobian
%    (i,j,Aij) = (rowNumber, colNumber, Element)

A = [   1, 1, 110
        1, 2, 205
        1, 3, 160
        1, 4, 160
        1, 5, 420
        1, 6, 260
        2, 1,   4
        2, 2,  32
        2, 3,  13
        2, 4,   8
        2, 5,   4
        2, 6,  14
        3, 1,   2
        3, 2,  12
        3, 3,  54
        3, 4, 285
        3, 5,  22
        3, 6,  80
      Obj, 1,   3
      Obj, 2,  24
      Obj, 3,  13
      Obj, 4,   9
      Obj, 5,  20
      Obj, 6,  19 ];

iAfun = A(:,1); jAvar = A(:,2); A = A(:,3);

ObjAdd = 0;

% Initial x.

x      = ones (n,1);

xlow   = zeros(n,1);
xupp   = [ 4
           3
           2
           8
           2
           2 ];

Flow   = [ 2000;
             55;
            800;
           -Inf ];
Fupp   =  Inf*ones(neF,1);

xstate =     zeros(n,1);
xmul   =     zeros(n,1);

Fstate =   zeros(neF,1);
Fmul   =   zeros(neF,1);
