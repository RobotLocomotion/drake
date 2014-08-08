classdef PPRateTransition < DrakeSystem
% PIECEWISE-POLYNOMIAL RATE TRANSITION SYSTEM
% Some systems, such as a trajectory planner running as a 
% model-predictive controller, run at low sample rates, but 
% think enough about the future to be able to produce continuous 
% outputs.  This class is intended as the solution to that.
% The output of the planning controller can be the parameters
% of a piecewise polynomial, which are fed (via zero-order hold)
% into the input of this PPRateTransition block.  This block
% will then produce the smooth, continuous output described by
% that polynomial.

  methods
    function obj = PPRateTransition(num_pieces,order,dimension)
      % PPRateTransition(num_pieces,order,dimension)
      % parameters mimic the mkpp and unmkpp syntax
      % @param num_pieces the number of breaks is num_pieces+1
      % @param order one + the degree of each piecewise polynomial
      %       (a cubic polynomial has order=4)
      % @param dimension the length of the vector output
      
      % the input to the system is 
      %  [breaks(:); coeffs(:)]
      obj = obj@DrakeSystem(0,0,(num_pieces+1)+(num_pieces*order*dimension),dimension,true,false);
      obj.num_pieces = num_pieces;
      obj.order = order;
      obj.dimension = dimension;
      
      obj = setSampleTime(obj,[0;0]);
    end
    
    function y = output(obj,t,x,u)
      breaks = u(1:(obj.num_pieces+1));
      coefs = reshape(u((obj.num_pieces+2):end),obj.dimension,obj.num_pieces,obj.order);
      pp = mkpp(breaks,coefs,obj.dimension);
      y = ppvalSafe(pp,t,false,false);
    end
  end
  
  properties
    num_pieces
    order
    dimension
  end
end