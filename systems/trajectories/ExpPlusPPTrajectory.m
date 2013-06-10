
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
    mex_ptr = 0 % higher speed eval in C++
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
      
      gamma_reshaped = reshape(obj.gamma, ...
          size(obj.gamma,1)*size(obj.gamma,2), size(obj.gamma,3));
      obj.mex_ptr = SharedDataHandle(ExpPlusPPTrajectoryEvalmex(...
          obj.breaks, obj.K, obj.A, obj.alpha, gamma_reshaped)); %,@ExpPlusPPTrajectoryEvalmex);
    end
  end
  
  methods (Static)
    function obj = loadobj(a)
      obj = ExpPlusPPTrajectory(a.breaks,a.K,a.A,a.alpha,a.gamma);
    end
  end
  
  methods  
    function [y,jj] = eval(obj,t)
      if true && obj.mex_ptr~=0
        % fast C++ version
        [y,jj] = ExpPlusPPTrajectoryEvalmex(obj.mex_ptr.getData, t);
      else
        y = zeros(size(obj.K,1),length(t));
        jj = zeros(length(t));
        for k=1:length(t)
          j = find(t(k)>=obj.breaks(1:end-1),1,'last');
          if isempty(j), j=1; end 
          trel = t(k)-obj.breaks(j);
          y(:,k) = obj.K*expm(obj.A*trel)*obj.alpha(:,j) ...
              + squeeze(obj.gamma(:,j,:))*(trel.^(0:obj.pporder-1)');
          jj(k) = j;
        end
        %if sum (abs(y-y_) > (0.0000001*max(abs(y),abs(y_))+eps)) > 0
        %    'error in y_', y, y_
        %end
      end
    end

    function t = getBreaks(obj)
      t = obj.breaks;
    end
  end
end
