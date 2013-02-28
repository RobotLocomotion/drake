function [x,F,xmul,Fmul,INFO] = hsmain()

snprint('hsmain.out');

hsmain.spc = which('hsmain.spc');
snspec (hsmain.spc);

snseti('Major Iteration limit', 250);

[x,xlow,xupp,xmul,xstate,Flow,Fupp,Fmul,Fstate,ObjAdd,ObjRow,    ...
 A,iAfun,jAvar,iGfun,jGvar] = hs47data;

[x,F,xmul,Fmul,INFO]= snsolve( x, xlow, xupp, xmul, xstate,    ...
			       Flow, Fupp, Fmul, Fstate,       ...
			       ObjAdd, ObjRow, A, iAfun, jAvar,...
			       iGfun, jGvar, 'hsmainusrfun');

snprint off; % Closes the file and empties the print buffer


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x,xlow,xupp,xmul,xstate,Flow,Fupp,Fmul,Fstate,ObjAdd,ObjRow, ...
          A,iAfun,jAvar,iGfun,jGvar] = hs47data()
%
%     hs47data defines a modified form of the problem HS47.
%
%     Minimize    (x(1)-x(2))^2 + (x(2)-x(3))^3 + (x(3)-x(4))^4
%                                                 + (x(4)-x(5))^4
%
%     subject to                        x(3)    + x(4) + x(5) >= 3
%                  x(1)      + x(2)^2 + x(3)^3                 = 3
%                  x(1)      + x(2)                           >= 1
%                              x(2)   - x(3)^2  + x(4)         = 1
%                  x(1)*x(5)                                   = 1
%
%     The pattern of nonzeros in the Jacobian is as follows, where
%     A = constant element, G = nonlinear element.
%
%              Column
%            | 1   2    3    4    5
%            +----------------------
%         1  |          A    A    A
%         2  | A   G    G
%         3  | A   A
%         4  |     A    G    A
%         5  | G                  G
%     row 6  | G   G    G    G    G    Objective row
%
%

neF    = 6;
n      = 5;
Obj    = 6;
ObjRow = 6;

%     First we assign the list of nonlinear derivative entries.
%     The pattern of nonlinear elements is as follows:
%
%              Column
%            | 1   2    3    4    5
%            +----------------------
%         1  |
%         2  |     6    7
%         3  |
%         4  |          8
%         5  | 9                 10
%     row 6  | 1   2    3    4    5    Objective row
%
%
%     ------------------------------------------------------------------

G = [ Obj,    1;
      Obj,    2;
      Obj,    3;
      Obj,    4;
      Obj,    5;
        2,    2;
        2,    3;
        4,    3;
        5,    1;
        5,    5 ];

iGfun = G(:,1); jGvar = G(:,2);

% Assign the list of constant derivative entries.
% The order of the constant elements is as follows:
%
%              Column
%            | 1   2    3    4    5
%            +----------------------
%         1  |          4    5    6
%         2  | 1
%         3  | 7   8
%         4  |     2         3
%         5  |
%     row 6  |                         Objective row
%

% Assign the constant part of the Jacobian  (i,j,Aij)

A = [ 2,  1,  1;
      4,  2,  1;
      4,  4,  1;
      1,  3,  1;
      1,  4,  1;
      1,  5,  1;
      3,  1,  1;
      3,  2,  1 ];

iAfun = A(:,1); jAvar = A(:,2); A = A(:,3);

ObjAdd = 0;

% Initial x.

x = [  2;
       sqrt(2) - 1 ;
       sqrt(2) - 1 ;
       2;
       0.5   ];

xlow   = -Inf*ones(n,1);
xupp   =  Inf*ones(n,1);
xstate =  zeros(n,1);
xmul   =  zeros(n,1);

Flow   = -Inf*ones(neF,1);
Fupp   =  Inf*ones(neF,1);

Flow(1) = 3;
Flow(2) = 3;
Fupp(2) = 3;
Flow(3) = 1;
Flow(4) = 1;
Fupp(4) = 1;
Flow(5) = 1;
Fupp(5) = 1;

Fmul    = zeros(neF,1);
Fstate  = zeros(neF,1);
