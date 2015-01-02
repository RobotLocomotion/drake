classdef HaltSimulationBlock < DrakeSystem
% This block provides an easy way to halt your running simulation gently (i.e. without ctrl+c).
% If you cascade the HaltSimulationBlock with your current system (for example, Atlas), then
% a button will be created which, when clicked, ends the simulation without throwing an error. 
% This might be useful if, for example, you would like to see results plotted from a simulation
% without running it to completion.
  properties
    lc;
    monitor
    fig
    halt_button
    halting
  end

  methods
    function obj = HaltSimulationBlock(frame, show_button, use_lcm)
      % Construct a block to easily halt a running simulation. 
      % @param frame the output frame of your current plant
      % @param show_button whether to show a Matlab button labeled 'Halt Simulation'
      % @param use_lcm whether to also listen on the LCM channel HALT_DRAKE_SIMULATION.
      %                if true, a drc.utime_t message on that channel will also halt sim.
      % example usage:
      %    r = Atlas(fullfile(getDrakePath,'examples','Atlas','urdf','atlas_minimal_contact.urdf'))
      %    sys = cascade(r, HaltSimulationBlock(r.getOutputFrame()));
      %    simulate(sys, [0, 5]);
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


