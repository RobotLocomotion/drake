%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [F,G] = cutefun(x)

[f,c] = cfn(x);
F = [f;c];
G = [];