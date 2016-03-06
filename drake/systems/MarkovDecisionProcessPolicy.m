classdef MarkovDecisionProcessPolicy < DrakeSystem
% takes a probability distribution over discrete states in 
% and outputs the expected action
  
  properties
    A  % actions; if vectors, then A(:,i) is action i
    PI % num_discrete_states-by-1 encoding of the policy
  end

  methods
    function obj = MarkovDecisionProcessPolicy(A,PI)
      obj = obj@DrakeSystem(0,0,size(PI,1),size(A,1),false,true);
      obj.A = A;
      obj.PI = PI;
    end
    
    function y = output(obj,t,~,u)
      % u is a distribution over finite states
      % PI maps that to a distribution over finite actions
      % y returns the expected action
      y = obj.A(:,obj.PI)*u;
    end
  end
end