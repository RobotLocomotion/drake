%--------------------------------------------------------------------------
% LGL_weights.m
% determines Gaussian quadrature weights using Lagrange-Gauss-Lobatto (LGL)
% nodes..... no the author was wrong... LGL refers to Legendre-Gauss-Lobatto
%--------------------------------------------------------------------------
% w = LGL_weights(tau)
% tau: LGL nodes
%   w: Gaussian quadrature weights
%--------------------------------------------------------------------------
% Author: Daniel R. Herber, Graduate Student, University of Illinois at
% Urbana-Champaign
% Date: 06/04/2015
%--------------------------------------------------------------------------
function w = LGL_weights(tau)
    % number of nodes
    N = length(tau)-1;
    
    % See Page 99 of the book: J. Shen, T. Tang and L. Wang, Spectral Methods:
    % Algorithms, Analysis and Applications, Springer Series in Compuational
    % Mathematics, 41, Springer, 2011. 
    % Uses the function: lepoly() 
    % Original function: [varargout] = legslb(n) located at
    % http://www1.spms.ntu.edu.sg/~lilian/bookcodes/legen/legslb.m
    [~,y] = lepoly(N,tau(2:end-1));
    % Use the weight expression (3.188) to compute the weights
    w = [2/(N*(N+1));2./(N*(N+1)*y.^2);2/(N*(N+1))]; 

end