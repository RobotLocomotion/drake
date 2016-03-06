classdef OptimalGrasp
  % Find the contacts with the largest ellipse within the convex hull of
  % the primitive wrenches, using a bilinear alternation
  properties
    initial_grasp_prog % A ForceClosureContacts object, used to find the initial force closure grasp
    lagrangian_step % A FixedContactsSearchQ1Base object
    contact_step  % A SearchContactsBase step
  end
  
  properties
    iter_max
    bilinear_use_prev_w  % A flag, set to true if the bilinear alternation contact step should use the solution of the previous contact step, default to false
    optimal_tol % Used in the bilinear alternation. If the increase of SOS verified Q1 between two iterations is below this tolerance, then stop the bilinear alternation
  end
  
  methods
    function obj = OptimalGrasp(initial_grasp_prog,lagrangian_step,contact_step)
      if(~isa(initial_grasp_prog,'ForceClosureContacts'))
        error('initial_grasp_prog should be a ForceClosureContacts object');
      end
      obj.initial_grasp_prog = initial_grasp_prog;
      if(~isa(lagrangian_step,'FixedContactsSearchQ1Base'))
        error('lagrangian_step should be a FixedContactsSearchQ1Base object');
      end
      obj.lagrangian_step = lagrangian_step;
      if(~isa(contact_step,'SearchContactsBase'))
        error('contact_step should be a SearchContactsBase object');
      end
      obj.contact_step = contact_step;
      obj.iter_max = 50;
      obj.bilinear_use_prev_w;
      obj.optimal_tol = 1e-3;
    end
    
    function [sol0,sol_bilinear0,solver_sol0,info0,solver_time] = findInitialGrasp(obj)
      [solver_sol0,info0,~,solver_time] = obj.initial_grasp_prog.optimize();
      [sol0,sol_bilinear0] = obj.initial_grasp_prog.retrieveSolution(solver_sol0);
    end
    
    function [sol_cell,sol_bilinear_cell,info,Q1_radius,solver_time] = bilinearAlternation(obj,friction_cones0)
      % @param friction_cones0  % A cell of FrictionCone objects. The
      % initial guess of the friction cones
      % @retval info     1   The bilinear alternation converges
      %                  2   The bilinear alternation reaches iteration
      %                  limit before convergence
      %                  0   The problem is not solved. One of the bilinear
      %                  alternation fails
      solver_time = 0;
      sol_cell = cell(obj.iter_max,1);
      sol_bilinear_cell = cell(obj.iter_max,1);
      Q1_radius = zeros(1,obj.iter_max); 
      iter = 1;
      l_step = true;
      
      fc_iter = friction_cones0;
      is_feasible = true;
      contact_step_w = zeros(length(obj.contact_step.w),1);
      is_optimal = false;
      L0_mat_iter = obj.lagrangian_step.findL0Mat(fc_iter);
      while(iter<=obj.iter_max && is_feasible && ~is_optimal)
        if(l_step)
          [solver_sol,info,solver_time_l] = obj.lagrangian_step.searchQ1(L0_mat_iter,fc_iter);
          sol = obj.lagrangian_step.retrieveSolution(solver_sol);
          Q1_radius(iter) = sol.radius;
          solver_time = solver_time+solver_time_l;
          if(info ~= 1)
            is_feasible = false;
          else
            l_step = false;
          end
          
          if(iter > 1)
            if(Q1_radius(iter) - Q1_radius(iter-1) < obj.optimal_tol)
              is_optimal = true;
            end
          end
          iter = iter+1;
        else
          if(obj.bilinear_use_prev_w)
            if(isa(obj.contact_step,'SearchContactsLinFC'))
              [solver_sol,info,solver_time_c] = obj.contact_step.findContactsGivenLagrangian(sol.L2_mat,Q1_radius(iter-1),contact_step_w); 
            elseif(isa(obj.contact_step,'SearchContacts'))
              [solver_sol,info,solver_time_c] = obj.contact_step.findContactsGivenLagrangian(sol.L1_mat,sol.L2_mat,Q1_radius(iter-1),contact_step_w); 
            end
          else
            if(isa(obj.contact_step,'SearchContactsLinFC'))
              [solver_sol,info,solver_time_c] = obj.contact_step.findContactsGivenLagrangian(sol.L2_mat,Q1_radius(iter-1)); 
            elseif(isa(obj.contact_step,'SearchContacts'))
              [solver_sol,info,solver_time_c] = obj.contact_step.findContactsGivenLagrangian(sol.L1_mat,sol.L2_mat,Q1_radius(iter-1)); 
            end
          end
          solver_time = solver_time+solver_time_c;
          if(info ~= 1)
            is_feasible = false;
          else
            l_step = true;
          end
          [sol_cell{iter},sol_bilinear_cell{iter}] = obj.contact_step.retrieveSolution(solver_sol);
          fc_iter = sol_cell{iter}.fc;
          
          L0_mat_iter = sol_cell{iter}.L0_mat;
          contact_step_w = double(solver_sol.eval(obj.contact_step.w));
        end
      end
      if(is_optimal)
        sol_cell = sol_cell(1:iter-1);
        sol_bilinear_cell = sol_bilinear_cell(1:iter-1);
        Q1_radius = Q1_radius(1:iter-1);
        info = 1;
      else
        if(iter > obj.iter_max)
          info = 2;
        else
          info = 0;
        end
      end
    end
    
    function [sol,sol_bilinear,info,solver_time] = findOptimalGrasp(obj)
      solver_time = 0;
      [sol0,sol_bilinear0,solver_sol0,info0,solver_time0] = findInitialGrasp(obj);
      if(info0 ~= 1)
        error('Faild to find force closure contacts in the initial step');
      end
      solver_time = solver_time+solver_time0;
      [sol,sol_bilinear,info,Q1_radius,solver_time1] = bilinearAlternation(obj,sol0.fc);
      solver_time = solver_time+solver_time1;
      if(info == 0)
        error('The bilinear alternation fails to find a solution');
      end
      sol{1} = sol0;
      sol_bilinear{1} = sol_bilinear0;
      for i = 1:length(sol)
        sol{i}.Q1_radius = Q1_radius(i);
        if(isa(obj.contact_step,'SearchContactsLinFC'))
          sol{i}.Q1_radius_true = computeQ1LinFC(sol{i}.contact_pos,obj.contact_step.disturbance_pos,sol{i}.fc_edges,obj.contact_step.Qw);
        else
          sol{i}.Q1_radius_true = computeQ1FromNLP(sol{i}.contact_pos,sol{i}.c,obj.contact_step.disturbance_pos,obj.contact_step.mu_face,obj.contact_step.Qw);
        end
      end
    end
  end
end
