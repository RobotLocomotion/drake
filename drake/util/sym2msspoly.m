function pm = sym2msspoly(xs,xm,ps)

% symbolic matlab to SPOT converter
% flips an sym polynomial, ps, which depends on variables xs, and
% outputs the msspoly polynomial, pm, in terms of the syms xm.

typecheck(xm,{'msspoly','TrigPoly'});
typecheck(xs,'sym');
typecheck(ps,'sym');

[M,N] = size(ps);
pm = msspoly(zeros(M,N));
for m = 1:M
    for n = 1:N
        poly = ps(m,n);
        pm(m,n) = sym2msspolyRecursion(poly,xs,xm);
    end
end
end


function outmss = sym2msspolyRecursion(spoly,xs,xm)
% Base case where spoly are not functions of xs - a constant term
if isempty(xs) || isempty(xm)
    outmss = double(spoly);
    return
end

c = coeffs(spoly, xs(1));
clen = length(c);
msscoeffs = msspoly(zeros(1,clen));
for i=1:clen
    polyi = c(i);
    msscoeffs(i) = sym2msspolyRecursion(polyi,xs(2:end),xm(2:end));
end
outmss = (xm(1).^(0:(clen-1)))*(msscoeffs)';
end