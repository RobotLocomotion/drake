%Test Script.

format compact;
setpath;  % defines the path

snscreen on

fprintf('\n============================================================= ');
fprintf('\nsntoy: Solving diet LP problem using snsolve ... ');
[x,xmul,F,Fmul,info] = t1diet;

fprintf('\n============================================================= ');
fprintf('\nsntoy: Solving toy problem using snsolve ... ');
snset ('Defaults');    % Advisable between runs of different problems
[x,xmul,F,Fmul,info] = sntoy;

fprintf('\n============================================================= ');
fprintf('\nsnsolvemain: snsolve solves hexagon ... ');
snset ('Defaults');
[x,xmul,F,Fmul,info] = snsolvemain;

fprintf('\n============================================================= ');
fprintf('\nhsmain: snsolve solves hs47 ... ');
snset ('Defaults');
[x,xmul,F,Fmul,info] = hsmain;

fprintf('\n============================================================= ');
fprintf('\nsntoy2: snopt solves toy problem ... ');
snset ('Defaults');
[x,F,info] = sntoy2;

fprintf('\n============================================================= ');
fprintf('\nsnoptmain: snopt solves hexagon with no derivatives ... ');
snset ('Defaults');
[x,F,info] = snoptmain;

fprintf('\n============================================================= ');
fprintf('\nsnoptmain2: snopt solves hexagon with dense Jacobian ... ');
snset ('Defaults');
[x,F,info] = snoptmain2;

fprintf('\n============================================================= ');
fprintf('\nsnoptmain3: snopt solves hexagon with some derivatives ... ');
snset ('Defaults');
[x,F,info] = snoptmain3;

snscreen off
