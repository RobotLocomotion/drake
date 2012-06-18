classdef PPTrajectory < Trajectory
  
  properties
    pp
  end
  
  methods
    function obj = PPTrajectory(ppform)
      obj = obj@Trajectory(ppform.dim);
      obj.pp = ppform;
      obj.tspan = [min(obj.pp.breaks) max(obj.pp.breaks)];
    end
    function y = eval(obj,t)
      t=max(min(t,obj.tspan(end)),obj.tspan(1));
      y = ppvalSafe(obj.pp,t);  % still benefits from being safe (e.g. for supporting TaylorVar)
    end

    function dtraj = fnder(obj)
      dtraj = PPTrajectory(fnder(obj.pp));
    end
    
    function obj = shiftTime(obj,offset)
      typecheck(offset,'double');
      sizecheck(offset,[1 1]);
      obj.tspan = obj.tspan + offset;
      obj.pp.breaks = obj.pp.breaks + offset;
    end
    
    function obj = uminus(obj)
      obj.pp.coefs = -obj.pp.coefs;
    end
    
    function t = getBreaks(obj)
      t = obj.pp.breaks;
    end
    
    function newtraj = append(obj, trajAtEnd)
      % Append a PPTrajectory to this one, creating a new trajectory that
      % starts where this object starts and ends where the given trajectory
      % ends.
      %
      % This will throw an error if the trajectory to append does not start
      % where the first trajectory ends.  This is useful if you did a bunch
      % of peicewise simulation and now want to combine them into one
      % object.
      %
      % @param trajAtEnd trajectory to append
      % @retval newtraj new PPTrajectory object that is the combination of
      % both trajectories
      
      
      % check for time condition
      firstEnd = obj.pp.breaks(end);
      secondStart = trajAtEnd.pp.breaks(1);
      
      if (firstEnd ~= secondStart)
        keyboard
        error(strcat('Cannot append trajectories that do not start/end at the same time.', ...
          'First trajectory ends at t = ',num2str(firstEnd), ...
          ' but the second trajectory starts at t = ', num2str(secondStart)));
      end
      
      
      % check for join condition (1st trajectory must end where the second
      % trajectory begins)
      
      firstEnd = obj.eval(trajAtEnd.pp.breaks(1));
      secondStart = trajAtEnd.eval(trajAtEnd.pp.breaks(1));
      if max(abs(firstEnd - secondStart)) > 1e-2
        
        error(strcat('Cannot append trajectories that do not start/end at the same spot.', ...
          'First trajectory ends at x = ', mat2str(firstEnd, 3), ...
          ' but the second trajectory starts at x = ', mat2str(secondStart, 3)));
      end
      
      % check for the same dimensions
      if (obj.pp.dim ~= trajAtEnd.pp.dim)
        error(strcat('Cannot append trajectories with different dimensionality.', ...
          'First trajectory has pp.dim = ', num2str(obj.pp.dim), ...
          ' but the second trajectory has pp.dim = ', num2str(trajAtEnd.pp.dim)));
      end
      
      % check for the same dimensions
      if (obj.dim ~= trajAtEnd.dim)
        error(strcat('Cannot append trajectories with different dimensionality.', ...
          'First trajectory has dim = ', num2str(obj.pp.dim), ...
          ' but the second trajectory has dim = ', num2str(trajAtEnd.pp.dim)));
      end
      
      % check for the same order
      if (obj.pp.order ~= trajAtEnd.pp.order)
        error(strcat('Cannot append trajectories with different order.', ...
          'First trajectory has pp.order = ', num2str(obj.pp.order), ...
          ' but the second trajectory has pp.order = ', num2str(trajAtEnd.pp.order)));
      end
      
      
      % check for same umin / umax
      if (obj.umax ~= trajAtEnd.umax)
        error(strcat('Cannot append trajectories with different umax.', ...
          'First trajectory has umax = ', num2str(obj.umax), ...
          ' but the second trajectory has umax = ', num2str(trajAtEnd.umax)));
      end
      
      if (obj.umin ~= trajAtEnd.umin)
        error(strcat('Cannot append trajectories with different umin.', ...
          'First trajectory has umin = ', num2str(obj.umin), ...
          ' but the second trajectory has umin = ', num2str(trajAtEnd.umin)));
      end
      
      % flags
      if (obj.input_angle_flag ~= trajAtEnd.input_angle_flag)
        error(strcat('Cannot append trajectories with different input_angle_flags.', ...
          'First trajectory has input_angle_flag = ', num2str(obj.input_angle_flag), ...
          ' but the second trajectory has input_angle_flag = ', num2str(trajAtEnd.input_angle_flag)));
      end
      
      if (obj.state_angle_flag ~= trajAtEnd.state_angle_flag)
        error(strcat('Cannot append trajectories with different state_angle_flag.', ...
          'First trajectory has state_angle_flag = ', num2str(obj.state_angle_flag), ...
          ' but the second trajectory has state_angle_flag = ', num2str(trajAtEnd.state_angle_flag)));
      end
      
      if (obj.output_angle_flag ~= trajAtEnd.output_angle_flag)
        error(strcat('Cannot append trajectories with different output_angle_flag.', ...
          'First trajectory has output_angle_flag = ', num2str(obj.output_angle_flag), ...
          ' but the second trajectory has output_angle_flag = ', num2str(trajAtEnd.output_angle_flag)));
      end
      
      
      newtraj = obj;
      
      newtraj.tspan = [min(obj.pp.breaks) max(trajAtEnd.pp.breaks)];
      
      newtraj.pp.dim = obj.pp.dim;
      newtraj.pp.order = obj.pp.order;
      
      newtraj.pp.pieces = obj.pp.pieces + trajAtEnd.pp.pieces;
      
      newtraj.dim = obj.dim;
      
      newtraj.pp.breaks = [obj.pp.breaks trajAtEnd.pp.breaks(2:end)];
      newtraj.pp.coefs = [obj.pp.coefs; trajAtEnd.pp.coefs];
      
      newtraj = setInputLimits(newtraj, obj.umin, obj.umax);
      
      newtraj = setAngleFlags(newtraj, obj.input_angle_flag, obj.state_angle_flag, obj.output_angle_flag);
      
      
    end % append
    
    % should getParameters and setParameters include the breaks? or just
    % the actual coefficients?  
  end
end
