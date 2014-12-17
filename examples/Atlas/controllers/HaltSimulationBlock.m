classdef HaltSimulationBlock < DrakeSystem
  properties
    lc;
    monitor
    fig
    halt_button
    halting
  end

  methods
    function obj = HaltSimulationBlock(frame, show_button, use_lcm)
      if nargin < 3
        use_lcm = true;
      end
      if nargin < 2
        show_button = true;
      end
      obj = obj@DrakeSystem(0, 0,  size(frame.coordinates,1), size(frame.coordinates,1), false, true);
      if use_lcm
        checkDependency('lcm');
        obj.lc = lcm.lcm.LCM.getSingleton();
        obj.monitor = drake.util.MessageMonitor(drc.utime_t, 'utime');
        obj.lc.subscribe('HALT_DRAKE_SIMULATION', obj.monitor);
      end
      if show_button
        obj.fig = figure(142);
        set(obj.fig, 'Position', [600,100,350,100]);
        clf;
        obj.halt_button = uicontrol('Style', 'pushbutton', 'String', 'Halt Simulation', ...
          'Position', [5, 5, 340, 90],...
          'Callback', {@(s, e) set_param(bdroot(), 'SimulationCommand', 'Stop')},...
          'FontSize', 28,...
          'ForegroundColor', 'w',...
          'BackgroundColor', [.7,.2,.2]);
      end
      obj = obj.setInputFrame(frame);
      obj = obj.setOutputFrame(frame);
    end

    function y = output(obj, t, x, u)
      y = u;
      if ~isempty(obj.monitor)
        msg = obj.monitor.getNextMessage(0);
        if ~isempty(msg)
          set_param(bdroot(), 'SimulationCommand', 'stop');
        end
      end
    end
  end
end


