classdef BMIspotless < spotsosprog
  % Please make sure you have spotless on your machine. If not, call
  % 'checkDependency('spotless')'
  properties(SetAccess = protected)
    w    % All the variables whose bilinear terms would appear
    W    % supposed to be the bilinear matrix w*w'.*blk1s
    blk1s % A block diagonal matrix with some blocks being 1s, and 0s for off diagonal blocks. supposedly W = w*w'.*blk1s
  end
  
  properties
    res_tol; % When the rank residue res = trace(obj.W)-obj.w'*obj.w is less than this res_tol, the iteration terminates successfully
    itr_max; % The maximum iterations the program will run
    alpha_covW % The amount of randomness in w_direction. Default is 0.01
    trial_max; % The maximum number of trials the program will run. In each trial, if the sequential SDP converges to a non-zero value, then rerun the program, until either sequential SDP converges to 0, or trial_max is reached.
    % At `early_terminate_itr`, the program will check the residue, if the residue is
    % larger than `early_terminate_tol`, the program will terminate, as the problem is
    % unlikely to converge. early_terminate_itr should be less than itr_max
    early_terminate_itr
    early_terminate_tol
    
    plot_iteration % True if plot function is called in every iteration. @default is false
    
    final_backoff_flag % A flag, true if we choose to backoff the solution in the last iteration. Default is false
    final_backoff_scale % A scaler, the amount of backoff. Default is 0.00001
    
    use_lcmgl   % A boolean, true if using lcmgl for visualization, false if using MATLAB PLOT, default is true
  end
  
  properties(Access = private)
    solver
  end
  
  methods
    function obj = BMIspotless()
      % Please make sure you have spotless on your machine. If not, call
      % 'checkDependency('spotless')'
      obj = obj@spotsosprog();
      obj.w = [];
      obj.W = [];
      obj.blk1s = [];
      obj.res_tol = 1e-3;
      obj.alpha_covW = 0.01;
      obj.itr_max = 200;
      obj.trial_max = 3;
      obj.early_terminate_itr = 100;
      obj.early_terminate_tol = 10;
      obj.plot_iteration = false;
      obj.final_backoff_flag = false;
      obj.final_backoff_scale = 0.00001;
      obj.use_lcmgl = true;
      if(checkDependency('mosek'))
        obj.solver = @spot_mosek;
      elseif(checkDependency('sedumi'))
        obj.solver = @spot_sedumi;
      else
        checkDependency('mosek');
      end
    end
    
    function [obj,w_new_ind] = addBilinearVariable(obj,w_new,W_new)
      % @retval w_new_ind   The indices of w_new in obj.w, obj.w(w_new_ind)
      % = w_new;
      w_new_ind = length(obj.w)+(1:length(w_new))';
      obj.w = [obj.w;w_new];
      obj.W = blkdiag(obj.W,W_new);
      obj.blk1s = blkdiag(obj.blk1s,ones(size(W_new)));
      obj = obj.withPSD([W_new w_new;w_new' 1]);
    end
    
    function [solver_sol,info,itr,solver_time] = optimize(obj,w_guess)
      % @param w_guess  A guess value for obj.w, default is zero
      % @retval info    -- 0, the rank relaxed problem is infeasible
      %                 -- 1, the problem converges
      %                 -- 2, the problem does not converge within maximal allowable
      %                 iterations
      %                 -- 3, at iterations obj.early_terminate_itr, the residue is still
      %                 larger than obj.early_terminate_tol. So the program does an early
      %                 termination, as it is unlikely to converge.
      if(nargin>1)
        if(numel(w_guess)~=length(obj.w))
          error('w_guess does not have the right size');
        end
        w_direction_init = w_guess(:);
      else
        w_direction_init = zeros(size(obj.w));
      end
      options = spot_sdp_default_options();
      options.verbose = 0;
      info = 2;
      trial_count = 1;
      while(trial_count <= obj.trial_max && info > 1)
        converged = false;
        itr = 1;
        solver_time = 0;
        w_direction = w_direction_init;
        while(~converged && itr<=obj.itr_max)
          solver_sol = obj.minimize(trace(obj.W)-2*w_direction'*obj.w,obj.solver,options);
          solver_time = solver_time + solver_sol.info.wtime;
          if(obj.plot_iteration)
            [sol,sol_bilinear] = obj.retrieveSolution(solver_sol);
            obj.plotSolution(sol,sol_bilinear);
          end
          if(~solver_sol.isPrimalFeasible)
            info = 0;
            return;
          end

          sol_w = double(solver_sol.eval(obj.w));
          sol_W = double(solver_sol.eval(obj.W));
          res = trace(sol_W)-sol_w'*sol_w;

          fprintf('iteration: %d, residue:%5.5f\n',itr,res);
          if(itr == obj.early_terminate_itr && res>obj.early_terminate_tol)
            info = 3;
            trial_count = trial_count + 1;
            break;
          end
          if(res<obj.res_tol)
            converged = true;
            % backoff
            if(obj.final_backoff_flag)
              prev_objective = double(solver_sol.eval(trace(obj.W)-2*w_direction'*obj.w));
              backoff_feasible = false;
              backoff_iter = 1;
              while(~backoff_feasible)
                if(prev_objective<0)
                  obj_backoff = obj.withPos((1-obj.final_backoff_scale*backoff_iter)*prev_objective-(trace(obj.W)-2*w_direction'*obj.w));
                else
                  obj_backoff = obj.withPos((1+obj.final_backoff_scale*backoff_iter)*prev_objective-(trace(obj.W)-2*w_direction'*obj.w));
                end
                solver_sol = obj_backoff.minimize(0,@spot_mosek,options);
                if(solver_sol.isPrimalFeasible())
                  sol_w = double(solver_sol.eval(obj.w));
                  sol_W = double(solver_sol.eval(obj.W));
                  res = trace(sol_W)-sol_w'*sol_w;

                  fprintf('backoff residue:%5.5f\n',res);
                  backoff_feasible = true;
                else
                  backoff_iter = backoff_iter+1;
                end
              end
            end
          else
            w_direction = descentDirection(obj,sol_w,sol_W);
            itr = itr+1;
          end
        end
        if(info ~= 3)
          if converged
            info = 1;
          else
            info = 2;
            trial_count = trial_count + 1;
          end
        end
      end
    end
    
    function w_direction = descentDirection(obj,sol_w,sol_W)
      covW = sol_W-(sol_w*sol_w').*obj.blk1s;
      min_eig_covW = min(eig(covW));
      w_direction = mvnrnd(sol_w,obj.alpha_covW*(covW+max(abs(1.1*min_eig_covW),1e-4)*eye(size(covW))),1)';
    end
    
    function plotSolution(obj,sol,sol_bilinear)
      % This function can be reloaded. If obj.plot_iteration = true, this function is
      % called in every iteration
      if(obj.use_lcmgl)
        checkDependency('lcmgl');
      else
        figure(1);
        clf;
      end
    end
    
    function [sol,sol_bilinear] = retrieveSolution(obj,solver_sol)
      sol.w = double(solver_sol.eval(obj.w));
      if(nargout>1)
        sol_bilinear.W = double(solver_sol.eval(obj.W));
      end
    end
  end
end