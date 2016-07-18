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
    % calculate node locations
    k=0:N;
    tau=(-cos(pi*k/N))';
end