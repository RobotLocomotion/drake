function tesths
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
clear;
snprint('tesths.out');

A = [  0  0 -1 -1 -1;
      -1 -1  0  0  0 ];
b = [ -3; -1 ];

Aeq = []; beq = []; xlow = []; xupp = [];
x0  = [ 2; sqrt(2) - 1; sqrt(2) - 1; 2; 0.5 ];

options = optimset('Algorithm','active-set');

[x,fval] = snpmat(@testhsobj,x0,A,b,Aeq,beq,xlow,xupp,@testhsnlc,options)

snprint off; % Closes the file and empties the print buffer


