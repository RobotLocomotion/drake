%--------------------------------------------------------------------------
% CGL_nodes.m
% determines Chebyshev-Gauss-Lobatto (CGL) nodes
%--------------------------------------------------------------------------
% tau = CGL_nodes(N)
%   N: number of nodes minus 1, should be an integer greater than 0
% tau: CGL nodes
%--------------------------------------------------------------------------
% Examples:
% tau = CGL_nodes(1)
% -1     1
% tau = CGL_nodes(2)
% -1     0     1
% tau = CGL_nodes(3)
% -1  -0.5   0.5   1
%--------------------------------------------------------------------------
% Author: Daniel R. Herber, Graduate Student, University of Illinois at
% Urbana-Champaign
% Date: 06/04/2015
%-------------------------------------------------------------------------
function tau = CGL_nodes(obj)

	N=obj.N-1;
    syms n
    % calculate node locations
    for k = 0:N
        tau(k+1,1) = -cos(pi*k/n); % symbolically to maintain precision
    end
    tau = double(subs(tau,'n',N));
end