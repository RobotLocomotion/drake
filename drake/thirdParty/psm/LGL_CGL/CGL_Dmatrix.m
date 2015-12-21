%--------------------------------------------------------------------------
% CGL_Dmatrix.m
% determines approximate differentiation matrix for Legendre-based method
% with CGL nodes
%--------------------------------------------------------------------------
% D = CGL_Dmatrix(tau)
% tau: CGL nodes
%   D: differentiation matrix
%--------------------------------------------------------------------------
% Author: Daniel R. Herber, Graduate Student, University of Illinois at
% Urbana-Champaign
% Date: 06/04/2015
%--------------------------------------------------------------------------
function D = CGL_Dmatrix(tau)
    % number of nodes
    N = length(tau)-1;
    
    % p. 54, Spectral Methods in MATLAB, Lloyd N. Trefethen
    % CHEB compute D = differentiation matrix, x = Chebyshev grid
    % function [D,x] = cheb(N)
    if N==0, D=0; x=1; return, end
    x = cos(pi*(0:N)/N)';
    c = [2; ones(N-1,1); 2].*(-1).^(0:N)';
    X = repmat(x,1,N+1);
    dX = X-X';
    D = (c*(1./c)')./(dX+(eye(N+1))); % off-diagonal entries
    D = D - diag(sum(D')); % diagonal entries
    
    D = -D; % since tau from [-1,1] not [1 -1]    
end