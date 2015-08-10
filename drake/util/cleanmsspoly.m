function fy = clean_mss(f,tol)
% Cleans up all terms in f whose coefficients are less than tol in abs
% value

[x,p,M] = decomp(f);
M(abs(M) < tol) = 0;
fy = recomp(x,p,M);

end


