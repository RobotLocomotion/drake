classdef PointMassBiped < LinearSystem
  properties
    omega
  end
  methods(Static)
    function frame = constructStateFrame()
      state_coords = {'com_x', 'com_y', 'com_xdot', 'com_ydot'};
      frame = SingletonCoordinateFrame('PointMassBipedState', length(state_coords), 'x', state_coords);
    end

    function frame = constructInputFrame()
      input_coords = {'cop_x', 'cop_y', 'rfoot_x', 'rfoot_y', 'lfoot_x', 'lfoot_y', 'rfoot_contact', 'lfoot_contact', 'com_des_x', 'com_des_y'};
      frame = SingletonCoordinateFrame('PointMassBipedInput', length(input_coords), 'u', input_coords);
    end
  end

  methods
    function obj = PointMassBiped(omega)
      Ac = [0, 0, 1, 0;
           0, 0, 0, 1;
           omega^2, 0, 0, 0;
           0, omega^2, 0, 0];
      Bc = [zeros(2, 10);
            [-omega^2, 0;
             0, -omega^2], zeros(2, 8)];

      C = [eye(4); zeros(10, 4)];
      D = [zeros(4, 10); eye(10)];
      obj = obj@LinearSystem(Ac, Bc, [], [], C, D);
      obj.omega = omega;

      obj = obj.setStateFrame(PointMassBiped.constructStateFrame());

      obj = obj.setInputFrame(PointMassBiped.constructInputFrame());
      
      obj = obj.setOutputFrame(MultiCoordinateFrame({obj.getStateFrame(), obj.getInputFrame()}));
    end

    function v = constructVisualizer(obj)
      function draw(t, x)
        p = Point(obj.getOutputFrame, x);

        cla
        hold on
        plot(p.com_x, p.com_y, 'mo');
        plot(p.com_des_x, p.com_des_y, 'ko'); 
        plot(p.cop_x, p.cop_y, 'b+');

        q_ic = [p.com_x; p.com_y] + 1/obj.omega * [p.com_xdot; p.com_ydot];
        plot(q_ic(1), q_ic(2), 'm^')

        % fprintf(1, 'contact r: %f l: %f\n', p.rfoot_contact, p.lfoot_contact);
        if p.rfoot_contact > 0.5
          plot(p.rfoot_x, p.rfoot_y, 'gx', 'MarkerSize', 10);
        else
          plot(p.rfoot_x, p.rfoot_y, 'go');
        end

        if p.lfoot_contact > 0.5
          plot(p.lfoot_x, p.lfoot_y, 'rx', 'MarkerSize', 10);
        else
          plot(p.lfoot_x, p.lfoot_y, 'ro');
        end
        
  %       legend('CoM actual', 'CoM plan', 'CoP', 'ICP actual', 'right foot', 'left foot')

        axis equal
        xlim([-0.5, 1.5])
        ylim([-0.5, 0.5])
      end
      v = FunctionHandleVisualizer(obj.getOutputFrame(), @draw);
    end
  end
end
