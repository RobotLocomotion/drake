classdef ExpPlusPPTrajectory < Trajectory
% useful for solutions to linear systems driven by pptrajectory inputs
%   in interval j, we have
%  y(t) = K * exp(A*(t-tj)) * alpha(:,j) + sum_i gamma(:,j,i) (t-tj)^i

  properties (Access=private)
    breaks  % with n+1 elements
    pporder 
    K       % must be dim x size(A,1)
    A       % square matrix from the matrix exponential
    alpha   % must be size(A,1) x n
    gamma   % must be dim x n x pporder
  end    

  methods
    function obj = ExpPlusPPTrajectory(breaks,K,A,alpha,gamma)
      obj = obj@Trajectory(size(K,1));
      obj.breaks = breaks(:)';
      obj.tspan = breaks([1,end]);
      n = numel(breaks)-1;
      obj.K = K;
      sizecheck(A,[size(K,2),size(K,2)]);
      obj.A = A;
      sizecheck(alpha,[size(K,2),n]);
      obj.alpha = alpha;

      sizecheck(gamma,[obj.dim,n,nan]);
      obj.gamma = gamma;
      obj.pporder = size(gamma,3);
    end
    
    function [y,jj] = eval(obj,t)
      y = zeros(size(obj.K,1),length(t));
      jj = zeros(length(t));
      for k=1:length(t)
        j = find(t(k)>=obj.breaks(1:end-1),1,'last');
        if isempty(j), j=0; end %j=length(obj.breaks)-1; end   % kaess: I believe this was wrong
        trel = t(k)-obj.breaks(j);
        y(:,k) = obj.K*expm(obj.A*trel)*obj.alpha(:,j) + squeeze(obj.gamma(:,j,:))*(trel.^(0:obj.pporder-1)');
        jj(k) = j;
      end
    end

    function t = getBreaks(obj)
      t = obj.breaks;
    end
  end
end