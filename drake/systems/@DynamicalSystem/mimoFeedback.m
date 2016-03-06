function newsys = mimoFeedback(sys1,sys2,sys1_to_sys2_connection,sys2_to_sys1_connection,input_select,output_select)
% Feedback combination of two (possibly multi-input, multi-output systems)
%
% @param sys1_to_sys2_connection an optional struct with fields 
% "from_output" and "to_input" specifying the connections FROM sys1 TO 
% sys2.  This matrix can have integer values (indicating the input/output 
% number) or CoordinateFrame values.  @default attempts to automatically 
% connect all signals that have an available transform.  If this connection 
% structure is not unique, then an error is thrown (informing you that you
% must specify things manually).
%   Example:
%      connection(1).from_output = 1;
%      connection(1).to_input = 2;
%      connection(2).from_output = 2;
%      connection(2).to_input = 4;
%    connects output 1 of sys1 to input 2 of sys2, and output 2 to input 4.
%   Example:
%       connection(1).from_output = robot.getStateFrame();
%       connection(1).to_input = 2;
%    connect the state output (which must be a subset of the robot
%    outputframe for this example to work) to the second input of the sys2.
%
% @param sys2_to_sys1_connection work just like sys1_to_sys2_connection,
% but for the other half of the connections.
%
% @param input_select an optional structure with fields "system" and "input"
% indicating the inputs that should be collected as inputs to the new
% system.  The values of the system field must be either 1 or 2.  The 
% values of the "input" field can either be the input number or an input 
% frame (which must match one of the frames exactly, not through a 
% coordinate transformation).  @default all of the unused non-empty inputs from sys1 
% followed by all unused non-empty inputs from sys2.
%   Example:
%      input_select(1).system = 1;
%      input_select(1).input = 2;
%      input_select(2).system = 2;
%      input_select(2).input = robot.getStateFrame();
%
% @param output_select an optional structure with fields "system" and 
% "output" representing the outputs from sys1 and sys2 (analogous to
% input_select).  @default all output from sys1  
%
% Notes:
%  - The inputs to a system may not be used by more than one input (from
%    sys1, or from the outside world).  Unused inputs will be left
%    unconnected (effectively setting them to zero).
%  - All proposed connections are still checked to make sure they
%    are valid using CoordinateFrames.

typecheck(sys1,'DynamicalSystem');
%sizecheck(sys1,1);
typecheck(sys2,'DynamicalSystem');
%sizecheck(sys2,1);
sys{1}=sys1; sys{2}=sys2;

if (nargin<3) sys1_to_sys2_connection=[]; end
sys1_to_sys2_connection = autoConnect(sys1.getOutputFrame,sys2.getInputFrame,sys1_to_sys2_connection);
if (nargin<4) sys2_to_sys1_connection=[]; end
sys2_to_sys1_connection = autoConnect(sys2.getOutputFrame,sys1.getInputFrame,sys2_to_sys1_connection);

if (nargin>3 && ~isempty(input_select))
  typecheck(input_select,'struct');
  if ~isempty(setxor(fieldnames(input_select),{'system','input'}))
    error('input_select must be a struct with fields "system" and "input"');
  end
  for i=1:length(input_select)
    typecheck(input_select(i).system,'numeric');
    rangecheck(input_select(i).system,1,2);
    if isa(input_select(i).input,'CoordinateFrame')
      input_select(i).input = getFrameNum(sys{input_select(i).system}.getInputFrame,input_select(i).input);
    end
    typecheck(input_select(i).input,'numeric');
    rangecheck(input_select(i).input,1,getNumFrames(sys{input_select(i).system}.getInputFrame));
  end
else
  sys1inputs = setdiff(1:getNumFrames(sys1.getInputFrame),[sys2_to_sys1_connection.to_input]);
  nonemptyframe = @(b) (b.dim>0);
  sys1inputs = sys1inputs(arrayfun(@(a) nonemptyframe(getFrameByNum(sys1.getInputFrame,a)), sys1inputs));
  sys2inputs = setdiff(1:getNumFrames(sys2.getInputFrame),[sys1_to_sys2_connection.to_input]);
  sys2inputs = sys2inputs(arrayfun(@(a) nonemptyframe(getFrameByNum(sys2.getInputFrame,a)), sys2inputs));
  input_select=[];
  for i=1:length(sys1inputs)
    input_select(i).system=1;
    input_select(i).input=sys1inputs(i);
  end
  for i=1:length(sys2inputs)
    input_select(end+1).system=2;
    input_select(end).input=sys2inputs(i);
  end
end

if (nargin>4 && ~isempty(output_select))
  typecheck(output_select,'struct');
  if ~isempty(setxor(fieldnames(output_select),{'system','output'}))
    error('output_select must be a struct with fields "system" and "output"');
  end
  for i=1:length(output_select)
    typecheck(output_select(i).system,'numeric');
    rangecheck(output_select(i).system,1,2);
    if isa(output_select(i).output,'CoordinateFrame')
      output_select(i).output = getFrameNum(sys{output_select(i).system}.getOutputFrame,output_select(i).output);
    end
    typecheck(output_select(i).output,'numeric');
    rangecheck(output_select(i).output,1,getNumFrames(sys{output_select(i).system}.getOutputFrame));
  end
else
  for i=1:getNumFrames(sys1.getOutputFrame)
    output_select(i).system=1;
    output_select(i).output=i;
  end
end

% check that no input is used more than once
in1=[sys2_to_sys1_connection.to_input];
if ~isempty(input_select)
  in1 = [in1,[input_select([input_select.system]==1).input]];
end
if length(unique(in1))<length(in1)
  in1
  error('you cannot use an input to sys1 more than once');
end
in2=[sys1_to_sys2_connection.to_input];
if ~isempty(input_select)
  in2 = [in2,[input_select([input_select.system]==2).input]];
end
if length(unique(in2))<length(in2)
  in2
  error('you cannot use an input to sys2 more than once');
end

% now start constructing the simulink model
mdl = ['MIMOFeedback_',datestr(now,'MMSSFFF')];  % use the class name + uid as the model name
new_system(mdl,'Model');
set_param(mdl,'SolverPrmCheckMsg','none');  % disables warning for automatic selection of default timestep
mdl = SimulinkModelHandle(mdl);

% construct subsystem (including mux/demux if necessary) and output
% number for sys1
mdl.addSubsystem('system1',getModel(sys1));
in{1} = setupMultiInput(sys1.getInputFrame,mdl,'system1');
out{1} = setupMultiOutput(sys1.getOutputFrame,mdl,'system1');

% construct subsystem (including mux/demux if necessary) and input number
% for sys2
mdl.addSubsystem('system2',getModel(sys2));
in{2} = setupMultiInput(sys2.getInputFrame,mdl,'system2');
out{2} = setupMultiOutput(sys2.getOutputFrame,mdl,'system2');

% make internal connections (adding transforms if necessary)
connection = [arrayfun(@(a)setfield(a,'from_system',1),arrayfun(@(a)setfield(a,'to_system',2),sys1_to_sys2_connection)), ...
  arrayfun(@(a)setfield(a,'from_system',2),arrayfun(@(a)setfield(a,'to_system',1),sys2_to_sys1_connection))];
for i=1:length(connection)
  fr1 = getFrameByNum(sys{connection(i).from_system}.getOutputFrame,connection(i).from_output);
  fr2 = getFrameByNum(sys{connection(i).to_system}.getInputFrame,connection(i).to_input);
  if (fr1==fr2)
    add_line(mdl,[out{connection(i).from_system},'/',num2str(connection(i).from_output)],[in{connection(i).to_system},'/',num2str(connection(i).to_input)]);
  else
    tf = findTransform(fr1,fr2,struct('throw_error_if_fail',true));
    mdl.addSubsystem(['tf',num2str(i)],getModel(tf));
    add_line(mdl,[out{connection(i).from_system},'/',num2str(connection(i).from_output)],['tf',num2str(i),'/1']);
    add_line(mdl,['tf',num2str(i),'/1'],[in{connection(i).to_system},'/',num2str(connection(i).to_input)]);
  end
end

% add input
if length(input_select)>0
  add_block('simulink3/Sources/In1',[mdl,'/in']);
  for i=1:length(input_select)
    infr{i}=getFrameByNum(sys{input_select(i).system}.getInputFrame,input_select(i).input);
  end
  newInputFrame=MultiCoordinateFrame.constructFrame(infr);
  newsysin = setupMultiOutput(newInputFrame,mdl,'in');
  for i=1:length(input_select)
    add_line(mdl,[newsysin,'/',num2str(i)],[in{input_select(i).system},'/',num2str(input_select(i).input)]);
  end
else
  newInputFrame=[];
end

% add output
if length(output_select)>0
  add_block('simulink3/Sinks/Out1',[mdl,'/out']);
  for i=1:length(output_select)
    outfr{i}=getFrameByNum(sys{output_select(i).system}.getOutputFrame,output_select(i).output);
  end
  newOutputFrame=MultiCoordinateFrame.constructFrame(outfr);
  newsysout = setupMultiInput(newOutputFrame,mdl,'out');
  for i=1:length(output_select)
    add_line(mdl,[out{output_select(i).system},'/',num2str(output_select(i).output)],[newsysout,'/',num2str(i)]);
  end
else
  newOutputFrame=[];
end

% add terminators to all non-used outputs
used_outputs = sys1_to_sys2_connection.from_output;
if ~isempty(output_select)
  used_outputs = [used_outputs, [output_select([output_select.system]==1).output]];
end
for i=setdiff(1:getNumFrames(sys1.getOutputFrame),used_outputs);
  add_block('simulink3/Sinks/Terminator',[mdl,'/sys1term',num2str(i)]);
  add_line(mdl,[out{1},'/',num2str(i)],['sys1term',num2str(i),'/1']);
end
used_outputs = sys2_to_sys1_connection.from_output;
if ~isempty(output_select)
  used_outputs = [used_outputs, [output_select([output_select.system]==2).output]];
end
for i=setdiff(1:getNumFrames(sys2.getOutputFrame),used_outputs)
  add_block('simulink3/Sinks/Terminator',[mdl,'/sys2term',num2str(i)]);
  add_line(mdl,[out{2},'/',num2str(i)],['sys2term',num2str(i),'/1']);
end

% finally construct the new dynamical system
if isempty(newInputFrame) 
  newsys = SimulinkModel(mdl,0);
else 
  newsys = SimulinkModel(mdl,newInputFrame.dim);
  newsys = setInputFrame(newsys,newInputFrame);
end
if (getNumStates(sys2)==0)
  newsys = setStateFrame(newsys,getStateFrame(sys1));
elseif (getNumStates(sys1)==0)
  newsys = setStateFrame(newsys,getStateFrame(sys2));
end
if ~isempty(newOutputFrame)
  newsys = setOutputFrame(newsys,newOutputFrame);
end
newsys.time_invariant_flag = sys1.time_invariant_flag && sys2.time_invariant_flag;
newsys.simulink_params = catstruct(sys1.simulink_params,sys2.simulink_params);

