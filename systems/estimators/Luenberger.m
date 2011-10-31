classdef Luenberger < SmoothRobotLibSystem
% Implements the Luenberger observer.
% <pre>
% For the system
%   \dot{x} = f(x)
%   y = g(x)
%  
% The Luenberger observer is a simple (nonlinear) observer of the form 
%  \dot{xhat} = f(xhat) + L*(y - g(xhat))
% where L is a matrix observer gain.
% </pre>
  
  properties
    L=[];  % observer gain (matrix)
  end

  methods 
    function obj = Luenberger(plant,L)
      error('not implemented yet');
    end
  end
  
end
