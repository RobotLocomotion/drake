function dH = quadFormGrad(X, P, dX)
% QUADFORMGRAD computes the gradient of H = X(q)' * P * X(q) with respect 
% to a vector q, given X, P, and dX/dq

nq = size(dX, 2);
F = P * X;
dF = matGradMultMat(P, X, sparse(numel(P), nq), dX);
% H = X' * F;
dH = matGradMultMat(X', F, transposeGrad(dX, size(X)), dF);

end