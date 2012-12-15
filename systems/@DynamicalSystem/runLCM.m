function runLCM(obj,x0,options)
% Runs the system as an lcm node.
% 
%  @param x0 initial conditions.  Use [] for the default initial
%  conditions.
%
%  @option tspan a 1x2 vector defining the start and end time of the simulation.  default [0,inf]


if (nargin<2) x0=[]; end
if (nargin<3) options = struct(); end
if (isfield(options,'tspan'))
  typecheck(options.tspan,'double');
  sizecheck(options.tspan,[1 2]);
else options.tspan = [0,inf]; end

% todo: handle options sampleTime, and ttl (others?)

checkDependency('lcm_enabled');
fin = obj.getInputFrame;
fout = obj.getOutputFrame;
if obj.getNumInputs>0
  typecheck(fin,'LCMSubscriber');
  if (~isfield(options,'inchannel')) options.inchannel = fin.defaultChannel(); end
end
if obj.getNumOutputs>0
  typecheck(fout,'LCMPublisher');
  if (~isfield(options,'outchannel')) options.outchannel = fout.defaultChannel(); end
end

if (obj.getNumInputs>0 && getNumStates(obj)<1) % if there are no state variables, then just trigger on input
  fin = subscribe(fin,options.inchannel);
  
  global g_scope_enable; g_scope_enable = true;
  
  % just run as fast as possible
  t=options.tspan(1); tic;
  while (t<=options.tspan(2))
    [u,t] = getNextMessage(fin,1000);
    if isempty(t)
      t=options.tspan(1)+toc;
      fprintf(1,'waiting... (t=%f)\n',t);
    else      
      y = obj.output(t,[],u);
      if (getNumOutputs(obj)>0)
        publish(fout,t,y,options.outchannel);
      end
    end
  end
else % otherwise set up the LCM blocks and run simulink.
  mdl = ['LCM_',datestr(now,'MMSSFFF')];  % use the class name + uid as the model name
  new_system(mdl,'Model');
  set_param(mdl,'SolverPrmCheckMsg','none');  % disables warning for automatic selection of default timestep
  
  load_system('simulink3');
  add_block('simulink3/Subsystems/Subsystem',[mdl,'/system']);
  Simulink.SubSystem.deleteContents([mdl,'/system']);
  Simulink.BlockDiagram.copyContentsToSubSystem(obj.getModel(),[mdl,'/system']);
  
  load_system('drake');
  if getNumInputs(obj)>0
    assignin('base',[mdl,'_subscriber'],fin);
    add_block('drake/lcmInput',[mdl,'/lcmInput'],'channel',['''',options.inchannel,''''],'dim',num2str(fin.dim),'lcm_subscriber',[mdl,'_subscriber']);
    add_line(mdl,'lcmInput/1','system/1');
  end
  % note: if obj has inputs, but no lcminput is specified, then it will just have the default input behavior (e.g. zeros)
  
  if getNumOutputs(obj)>0
    assignin('base',[mdl,'_publisher'],fout);
    add_block('drake/lcmOutput',[mdl,'/lcmOutput'],'channel',['''',options.outchannel,''''],'dim',num2str(fout.dim),'lcm_publisher',[mdl,'_publisher']);
    add_line(mdl,'system/1','lcmOutput/1');
  elseif (getNumOutputs(obj)>0)
    add_block('simulink3/Sinks/Terminator',[mdl,'/terminator']);
    add_line(mdl,'system/1','terminator/1');
  end

  % add realtime block
  add_block('drake/realtime',[mdl,'/realtime']);
  
  pstruct = obj.simulink_params;
  pstruct.StartTime = num2str(options.tspan(1));
  pstruct.StopTime = num2str(options.tspan(2));

  if (~isempty(x0)) % handle initial conditions
    x0 = obj.stateVectorToStructure(x0,mdl);
    assignin('base',[mdl,'_x0'],x0);
    pstruct.InitialState = [mdl,'_x0'];
    pstruct.LoadInitialState = 'on';

    if (~isempty(find_system(mdl,'ClassName','InitialCondition')))
      warning('Your model appears to have an initial conditions block in it (e.g., from SimMechanics).  That block will overwrite any initial conditions that you pass in to simulate.');
    end
  end  
  
  sim(mdl,pstruct);
end

end
