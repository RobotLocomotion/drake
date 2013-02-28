function [F,G] = cuteobjcon(x)

[f,c]  = cfn(x);
F      = [f;c];
G      = [];