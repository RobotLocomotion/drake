%--------------------------------------------------------------------------
% LGL_nodes.m
% determines Lagrange-Gauss-Lobatto (LGL) nodes
%--------------------------------------------------------------------------
% tau = LGL_nodes(N)
%   N: number of nodes minus 1, should be an integer greater than 0
% tau: LGL nodes
%--------------------------------------------------------------------------
% Examples:
% tau = LGL_nodes(1)
% -1     1
% tau = LGL_nodes(2)
% -1     0     1
% tau = LGL_nodes(3)
% -1  -0.44721   0.44721   1
%--------------------------------------------------------------------------
% Author: Daniel R. Herber, Graduate Student, University of Illinois at
% Urbana-Champaign
% Date: 06/04/2015
%--------------------------------------------------------------------------
function tau = LGL_nodes(obj)
    % See Page 99 of the book: J. Shen, T. Tang and L. Wang, Spectral Methods:
    % Algorithms, Analysis and Applications, Springer Series in Compuational
    % Mathematics, 41, Springer, 2011. 
    % Uses the function: lepoly() 
    % Original function: [varargout] = legslb(n) located at
    % http://www1.spms.ntu.edu.sg/~lilian/bookcodes/legen/legslb.m

    % Compute the initial guess of the interior LGL points
    N=obj.N-1;
    thetak = (4*[1:N]-1)*pi/(4*N+2);
    sigmak = -(1-(N-1)/(8*N^3)-(39-28./sin(thetak).^2)/(384*N^4)).*cos(thetak);
    ze = (sigmak(1:N-1)+sigmak(2:N))/2;  
    ep = eps*10;                           % error tolerance for stopping iteration
    ze1 = ze+ep+1;
 
    while max(abs(ze1-ze))>=ep,            % Newton's iteration procedure
      ze1 = ze;
      [dy,y] = lepoly(N,ze);
      ze = ze-(1-ze.*ze).*dy./(2*ze.*dy-N*(N+1)*y);  % see Page 99 of the book
    end;                                   % around 6 iterations are required for n=100

    tau=[-1,ze,1]'; % column vector

end