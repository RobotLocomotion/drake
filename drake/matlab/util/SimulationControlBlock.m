classdef SimulationControlBlock < DrakeSystem
% This block provides an easy way to halt your running simulation gently (i.e. without ctrl+c).
% Pass the options gui_control_interface and/or lcm_control_interface into the simulate method to use it.
% This might be useful if, for example, you would like to see results plotted from a simulation
% without running it to completion.
  properties
    lc;
    monitor
    fig;
    halt_button
    halting
  end

  methods
    function obj = SimulationControlBlock(mdl,show_gui, lcm_channel)
      % Construct a block to easily halt a running simulation. 
      % @param show_gui whether to show a Matlab button labeled 'Halt Simulation'
      % @param lcm_channel channel on which to listen for
      % drake.lcmt_simulation_command messages
      if nargin < 2
        lcm_channel = '';
      end
      if nargin < 1
        show_gui = true;
      end
      obj = obj@DrakeSystem(0, 0, 0, 0, false, true);
      if ~isempty(lcm_channel)
        checkDependency('lcm');
        obj.lc = lcm.lcm.LCM.getSingleton();
        obj.monitor = drake.matlab.util.MessageMonitor(drake.lcmt_simulation_command(), 'timestamp');
        obj.lc.subscribe(lcm_channel, obj.monitor);
        fprintf(1,'Enabling LCM simultion control on channel %s\n',lcm_channel);
      end
      if show_gui
        obj.fig = figure(142);
        set(obj.fig, 'Position', [600,100,350,100]);
        clf;
        % todo: add run, pause, stop etc buttons?
        obj.halt_button = uicontrol('Style', 'pushbutton', 'String', 'Halt Simulation', ...
          'Position', [5, 5, 340, 90],...
          'Callback', {@(s, e) set_param(mdl, 'SimulationCommand', 'Stop')},...
          'FontSize', 28,...
          'ForegroundColor', 'w',...
          'BackgroundColor', [.7,.2,.2]);
      end
    end

    function y = output(obj, t, ~, ~)
      y=[];
      if ~isempty(obj.monitor)
        msg = obj.monitor.getNextMessage(0);
        if ~isempty(msg)
          msg = drake.lcmt_simulation_command(msg);
          
          % note: according to http://www.mathworks.com/help/simulink/ug/using-the-set-param-command.html
          % these will not work when matlab is run in -nodisplay mode
          % perhaps there is another way?
          % (e.g. http://www.mathworks.com/help/simulink/sfg/sssetstoprequested.html)
          % Despite the documentation, 'stop' seems to work even if the
          % simulation is started using the sim command.  but
          % 'start' and 'pause' seem to have no effect.
          % 'shutdown' seems easy, but you can't exit when a simulation is
          % running and force_close_system() didn't work.
          switch(msg.command_type)
            case msg.STOP
              disp('Received STOP LCM simulation command.');
              set_param(bdroot(), 'SimulationCommand', 'stop');
            otherwise
              disp(['Received unsupported LCM simulation command:',num2str(msg.command_type)]);
          end
        end
      end
    end
  end
end


