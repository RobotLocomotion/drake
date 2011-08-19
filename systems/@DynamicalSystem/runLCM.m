function runLCM(obj,lcmCoder,in,out,x0,options)
% Runs the system as an lcm node.
% 
%  @param lcmCoder an LCMCoder object which defines all of the messages
%  @param in the name of the message to subscribe to for the input.  Must be
%    'x','xhat','u', or 'y'.  The robotname is automatically prepended.
%  @param out the name of the message to publish for the output.  Must be
%    'x','xhat','u', or 'y'.  The robotname is automatically prepended.
%  @param x0 initial conditions.  Use [] for the default initial
%  conditions.
%
%  @option tspan a 1x2 vector defining the start and end time of the simulation.  default [0,inf]


if (nargin<5) x0=[]; end
if (nargin<6) options = struct(); end
if (isfield(options,'tspan'))
  typecheck(options.tspan,'double');
  sizecheck(options.tspan,[1 2]);
else options.tspan = [0,inf]; end

% todo: handle options x0, sampleTime, and ttl (others?)

checkDependency('lcm_enabled');
if (~isempty(in))
  switch(in)
    case {'x','xhat'}
      dim_in = lcmCoder.dim_x;
      decoder = '@(msg)decodeX(lcmCoder,msg)';
    case 'u'
      dim_in = lcmCoder.dim_u;
      decoder = '@(msg)decodeU(lcmCoder,msg)';
    case 'y'
      dim_in = lcmCoder.dim_y;
      decoder = '@(msg)decodeY(lcmCoder,msg)';
    otherwise
      error(['don''t know how to decode the ', in ,' message']);
  end
  if (dim_in ~= obj.getNumInputs())
    error('input dimensions do not match');
  end
end
if (~isempty(out))
  switch(out)
    case {'x','xhat'}
      dim_out = lcmCoder.dim_x;
      encoder = '@(t,x)encodeX(lcmCoder,t,x)';
    case 'u'
      dim_out = lcmCoder.dim_u;
      encoder = '@(t,u)encodeU(lcmCoder,t,u)';
    case 'y'
      dim_out = lcmCoder.dim_y;
      encoder = '@(t,y)encodeY(lcmCoder,t,y)';
    otherwise
      error('don''t know how to publish your out message');
  end
  if (dim_out ~= obj.getNumOutputs())
    error('output dimensions do not match')
  end
end
name=lcmCoder.getRobotName();

if (~isempty(in) && getNumStates(obj)<1) % if there are no state variables, then just trigger on input
  decoder = eval(decoder);
  if (~isempty(out)) encoder = eval(encoder); end
  lc = lcm.lcm.LCM.getSingleton(); %('udpm://239.255.76.67:7667?ttl=1');
  aggregator = lcm.lcm.MessageAggregator();
  aggregator.setMaxMessages(1);  % make it a last-message-only queue
  
  lc.subscribe([lower(name),'_',in],aggregator);
  outchannel = [lower(name),'_',out];
  
  global g_scope_enable; g_scope_enable = true;
  
  % just run as fast as possible
  t=options.tspan(1);
  while (t<=options.tspan(2))
    umsg = getNextMessage(aggregator,1000);
    if (~isempty(umsg))
      [u,t] = decoder(umsg);
      y = obj.output(t,[],u);
      if (~isempty(out))
        ymsg = encoder(t,y);
        lc.publish(outchannel,ymsg);
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
  
  assignin('base',[mdl,'_lcmCoder'],lcmCoder);
  load_system('robotlib');
  if (in)
    channel = ['''',lower(getRobotName(lcmCoder)),'_',in,''''];
    i=findstr(decoder,'lcmCoder');
    decoder = [decoder(1:(i-1)),mdl,'_',decoder(i:end)];
    add_block('robotlib/lcmInput',[mdl,'/lcmInput'],'channel',channel,'dim',num2str(dim_in),'decode_fcn',decoder);
    add_line(mdl,'lcmInput/1','system/1');
  end
  % note: if obj has inputs, but no lcminput is specified, then it will just have the default input behavior (e.g. zeros)
  
  if (out)
    channel = ['''',lower(getRobotName(lcmCoder)),'_',out,''''];
    i=findstr(encoder,'lcmCoder');
    encoder = [encoder(1:(i-1)),mdl,'_',encoder(i:end)];
    add_block('robotlib/lcmOutput',[mdl,'/lcmOutput'],'channel',channel,'dim',num2str(dim_out),'encode_fcn',encoder);
    add_line(mdl,'system/1','lcmOutput/1');
  elseif (getNumOutputs(obj)>0)
    add_block('simulink3/Sinks/Terminator',[mdl,'/terminator']);
    add_line(mdl,'system/1','terminator/1');
  end

  % add realtime block
  add_block('robotlib/realtime',[mdl,'/realtime']);
  
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
