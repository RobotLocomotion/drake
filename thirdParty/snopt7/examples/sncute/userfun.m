function [F,G] = userfun(x)

global iGfun jGvar

[f,c]  = cfn(x);
F      = [f;c];
[g,cjac] = csgr(x);
G      = [g ;cjac];
G = sparse(full(G));
%check
[i,j,g] = find(G);
if (length(g) > length(iGfun))
 error('Something may be wrong with snJac');
end

%G = findG( iGfun, jGvar, G );



