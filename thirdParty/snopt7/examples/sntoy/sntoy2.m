 function [x,F,inform] = sntoy2()
%function [x,F,inform] = sntoy2()
%Mimics sntoyA.f in $SNOPT/examples
%     Minimize      3*x(1) + (x(1) + x(2) + x(3))^2 + 5*x(4)
%
%     subject to             4*x(2)   + 2*x(3)              >= 0
%                     x(1) +   x(2)^2 +   x(3)^2             = 2
%                              x(2)^4 +   x(3)^4   +   x(4)  = 4
%
%                     x(1) >= 0,                       x(4) >= 0.
%

snprint('sntoy2.out');  % By default, screen output is on;
sntoy2.spc = which('sntoy2.spc');
snspec (sntoy2.spc);
snseti ('Major Iteration limit', 250);

x    = ones(4,1);
xlow = [   0,-Inf,-Inf,   0]';
xupp = [ Inf, Inf, Inf, Inf]';
Flow = [-Inf,   0,   2,   4]';
Fupp = [ Inf, Inf,   2,   4]';

[x,F,inform] = snopt(x,xlow,xupp,Flow,Fupp,'toyusrfun2');

snset ('Defaults');
snprint off;
