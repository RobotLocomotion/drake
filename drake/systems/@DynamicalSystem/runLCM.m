function runLCM(obj,x0,options)
% Runs the system as an lcm node.
% 
%  @param x0 initial conditions.  Use [] for the default initial
%  conditions.
%
%  @option tspan a 1x2 vector defining the start and end time of the simulation.  default [0,inf]
%  @option timekeeper the full name (system/block) of a simulink block that maintains the simulation
%  time (e.g. by sleeping).  set to '' to have no timekeeper.  @default 'drake/realtime'
%  @option realtime_factor @default 1  (only guaranteed to work for
%  drake/realtime timekeeper)
%  @option input_sample_time sets the sample time of all lcm inputs.  can
%  be any valid simulink sample time (e.g. [-1 0] for inherited).  
%  @default .005  
%  @option output_sample_time sets the publishing rate of any lcm outputs.  
%  can be any valid simulink sample time (e.g. set to [-1 0] for inherited.)  @default .005

if (nargin<2) x0=[]; end
if (nargin<3) options = struct(); end
if (isfield(options,'tspan'))
  typecheck(options.tspan,'double');
  sizecheck(options.tspan,[1 2]);
else options.tspan = [0,inf]; end

% todo: handle options sampleTime, and ttl (others?)

checkDependency('lcm');
fin = obj.getInputFrame;
fout = obj.getOutputFrame;
if obj.getNumOutputs>0 && typecheck(fout,'LCMPublisher');
  if ~isfield(options,'outchannel'), options.outchannel = fout.defaultChannel(); end
  if ~isfield(options,'input_sample_time'), options.input_sample_time = [.005,0]; end
  if ~isfield(options,'output_sample_time'), options.output_sample_time = [.005,0]; end
end

if (obj.getNumInputs>0 && getNumStates(obj)<1 && isa(fin,'LCMSubscriber')) 
  % if there are no state variables, and the input frame is a simple
  % (one channel) lcm input, then just trigger on that input
  
  if (~isfield(options,'inchannel')) options.inchannel = fin.defaultChannel(); end
  subscribe(fin,options.inchannel);
  b_lcm_output = isa(fout,'LCMPublisher');
  
  global g_scope_enable; g_scope_enable = true;
  
  % just run as fast as possible
  t=options.tspan(1); tic;
  while (t<=options.tspan(2))
    [u,t] = getNextMessage(fin,1000);
    if isempty(t)
      t=toc;
      fprintf(1,'waiting... (t=%f)\n',t);
    else
      tic;
      y = obj.output(t,[],u);
      if (getNumOutputs(obj)>0 && b_lcm_output)
        publish(fout,t,y,options.outchannel);
      end
    end
  end
else % otherwise set up the LCM blocks and run simulink.
  if ~isfield(options,'timekeeper') 
    options.timekeeper = 'drake/realtime'; 
  elseif ~isempty(options.timekeeper)
    typecheck(options.timekeeper,'char');
    
    % try to load the system in case it's necessary
    ind = find(options.timekeeper=='/',1);
    load_system(options.timekeeper(1:ind-1));
  end
  
  mdl = ['LCM_',datestr(now,'MMSSFFF')];  % use the class name + uid as the model name
  new_system(mdl,'Model');
  set_param(mdl,'SolverPrmCheckMsg','none');  % disables warning for automatic selection of default timestep
  mdl = SimulinkModelHandle(mdl);  
  
  mdl.addSubsystem('system',obj.getModel());
  
  load_system('drake');
  if getNumInputs(obj)>0
    setupLCMInputs(fin,mdl,'system',1,options);
  end
  % note: if obj has inputs, but no lcminput is specified, then it will just have the default input behavior (e.g. zeros)
  
  if getNumOutputs(obj)>0 && typecheck(fout,'LCMPublisher')
    setupLCMOutputs(fout,mdl,'system',1,options);
  end

  % add realtime block
  if ~isempty(options.timekeeper) 
    if isfield(options,'realtime_factor')
      add_block(options.timekeeper,[mdl,'/timekeeper'],'speed',num2str(options.realtime_factor));
    else
      add_block(options.timekeeper,[mdl,'/timekeeper']);
    end
  end
  
  pstruct = obj.simulink_params;
  pstruct.StartTime = num2str(options.tspan(1));
  pstruct.StopTime = num2str(options.tspan(2));

  if (~isempty(x0)) % handle initial conditions
    x0 = obj.stateVectorToStructure(double(x0),mdl);
    pstruct.InitialState = registerParameter(mdl,x0,'x0');
    pstruct.LoadInitialState = 'on';

    if (~isempty(find_system(mdl,'ClassName','InitialCondition')))
      warning('Your model appears to have an initial conditions block in it (e.g., from SimMechanics).  That block will overwrite any initial conditions that you pass in to simulate.');
    end
  end  
  
  sim(mdl,pstruct);
  
end

end
