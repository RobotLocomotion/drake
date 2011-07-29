classdef StochasticPolynomialSystem < StochasticRobotLibSystem & PolynomialSystem
  
  properties (SetAccess=private)
    p_w
    p_stochastic_dynamics % msspoly representations of dynamics, update,and output
    p_stochastic_update
    p_stochastic_output
  end
  
  methods
    function obj = StochasticPolynomialSystem(num_xc,num_xd,num_u,num_y,direct_feedthrough_flag,time_invariant_flag,num_w,ts_w); % todo: handle msspoly inputs if/when necessary
      obj = obj@StochasticRobotLibSystem(num_xc,num_xd,num_u,num_y,direct_feedthrough_flag,time_invariant_flag,num_w,ts_w);
      obj = obj@PolynomialSystem(num_xc,num_xd,num_u,num_y,direct_feedthrough_flag,time_invariant_flag);

      % Now create the msspoly versions of the dynamics,update,and output:
      obj.p_w=msspoly('w',num_w);
      
      % these will error if the system is not polynomial (should I catch
      % and rethrow the error with more information?)
      if (num_xc>0)
        obj.p_stochastic_dynamics=obj.stochasticDynamics(obj.p_t,obj.p_x,obj.p_u,obj.p_w);
      end
      if (num_xd>0)
        obj.p_stochastic_update=obj.stochasticUpdate(obj.p_t,obj.p_x,obj.p_u,obj.p_w);
      end
      if (num_y>0)
        obj.p_stochastic_output=obj.stochasticOutput(obj.p_t,obj.p_x,obj.p_u,obj.p_w);
      end
    end

    % resolve multiple inheritance conflicts
    function xcdot = dynamics(obj,t,x,u)
      xcdot = dynamics@StochasticRobotLibSystem(obj,t,x,u);
    end
    function xdn = update(obj,t,x,u)
      xdn = update@StochasticRobotLibSystem(obj,t,x,u);
    end
    function y = output(obj,t,x,u)
      y = output@StochasticRobotLibSystem(obj,t,x,u);
    end
      
  end
  
  
end
