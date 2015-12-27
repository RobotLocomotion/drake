%--------------------------------------------------------------------------
% CGL_weights.m
% determines Gaussian quadrature weights using Chebyshev-Gauss-Lobatto
% (CGL) nodes
%--------------------------------------------------------------------------
% w = CGL_weights(tau)
% tau: CGL nodes
%   w: Gaussian quadrature weights
%--------------------------------------------------------------------------
% Author: Daniel R. Herber, Graduate Student, University of Illinois at
% Urbana-Champaign
% Date: 06/04/2015
%--------------------------------------------------------------------------
function w = CGL_weights(tau)
    % number of nodes
    N = length(tau)-1;

    % p. 128, Spectral Methods in MATLAB, Lloyd N. Trefethen
    % CLENCURT nodes x (Chebyshev points) and weights w
    % for Clenshaw-Curtis quadrature
    % function [x,w] = clencurt(N)
    theta = pi*(0:N)'/N; x = cos(theta);
    w = zeros(1,N+1); ii = 2:N; v = ones(N-1,1);
    if mod(N,2)==0
    w(1) = 1/(N^2-1); w(N+1) = w(1);
    for k=1:N/2-1, v = v - 2*cos(2*k*theta(ii))/(4*k^2-1); end
    v = v - cos(N*theta(ii))/(N^2-1);
    else
    w(1) = 1/N^2; w(N+1) = w(1);
    for k=1:(N-1)/2, v = v - 2*cos(2*k*theta(ii))/(4*k^2-1); end
    end
    w(ii) = 2*v/N;
    
    w = flipud(w)'; % since tau from [-1,1] not [1 -1]
end