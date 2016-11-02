% Quadratic Form "Balancing"
%
%      T = balqf(S,P)
%
% Input:
%   S -- n-by-n symmetric positive definite.
%   P -- n-by-n symmetric, full rank.
%
% Finds a T such that:
%   T'*S*T = D
%   T'*P*T = D^(-1)
%
function [T,D] = balanceQuadraticForm(S,P)
    if norm(S - S',1) > 1e-8, error('S must be symmetric'); end
    if norm(P - P',1) > 1e-8, error('P must be symmetric'); end
    if cond(P) > 1e10, error('P must be full rank'); end
    
    V = inv(chol(S)); % Tests if S positive def. for us.
    [U,M] = svd(V'*P*V);
    l = diag(M);
    T = V*U*diag(l.^(-1/4));
    D = diag(l.^(-1/2));
end

