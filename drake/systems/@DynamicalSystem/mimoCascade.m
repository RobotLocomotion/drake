function newsys = mimoCascade(sys1,sys2,connection,input_select,output_select)
% Cascades two systems which can have multiple input/output
% frames
%
% @param connection an optional struct with fields "from_output" and 
% "to_input" specifying the connections FROM sys1 TO sys2.  This matrix 
% can have integer values (indicating the input/output number) or 
% CoordinateFrame values.  @default attempts to automatically connect all
% signals that have an available transform.  If this connection structure
% is not unique, then an error is thrown (informing you that you must 
% specify things manually).
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
% @param input_select an optional structure with fields "system" and "input"
% indicating the inputs that should be collected as inputs to the new
% system.  The values of the system field must be either 1 or 2.  The 
% values of the "input" field can either be the input number or an input 
% frame (which must match one of the frames exactly, not through a 
% coordinate transformation).  @default all of the non-empty inputs from sys1 
% followed by any unused non-empty inputs from sys2.
%   Example:
%      input_select(1).system = 1;
%      input_select(1).input = 2;
%      input_select(2).system = 2;
%      input_select(2).input = robot.getStateFrame();
%
% @param output_select an optional structure with fields "system" and 
% "output" representing the outputs from sys1 and sys2 (analogous to
% input_select).  @default all of the unused outputs from sys1
% followed by all of the outputs from sys2.
%
% Notes:
%  - The inputs to sys2 may not be used by more than one input (from
%    sys1, or from the outside world).  Unused inputs will be left
%    unconnected (effectively setting them to zero).
%  - All proposed connections are still checked to make sure they
%    are valid using CoordinateFrames.

typecheck(sys1,'DynamicalSystem');
%sizecheck(sys1,1);
typecheck(sys2,'DynamicalSystem');
%sizecheck(sys2,1);
sys{1}=sys1; sys{2}=sys2;

if (nargin<3) connection=[]; end
connection = autoConnect(sys1.getOutputFrame,sys2.getInputFrame,connection);

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
  sys1inputs = 1:getNumFrames(sys1.getInputFrame);
  nonemptyframe = @(b) (b.dim>0);
  sys1inputs = sys1inputs(arrayfun(@(a) nonemptyframe(getFrameByNum(sys1.getInputFrame,a)), sys1inputs));
  sys2inputs = setdiff(1:getNumFrames(sys2.getInputFrame),[connection.to_input]);
  sys2inputs = sys2inputs(arrayfun(@(a) nonemptyframe(getFrameByNum(sys2.getInputFrame,a)), sys2inputs));
  input_select=struct('system',{},'input',{});
  for i=1:length(sys1inputs)
    if sys1.getInputFrame.getFrameByNum(i).dim<1, continue; end
    input_select(i).system=1;
    input_select(i).input=sys1inputs(i);
  end
  for i=1:length(sys2inputs)
    if sys2.getInputFrame.getFrameByNum(i).dim<1, continue; end
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
  sys1outputs = setdiff(1:getNumFrames(sys1.getOutputFrame),[connection.from_output]);
  sys2outputs = 1:getNumFrames(sys2.getOutputFrame);
  output_select=struct('system',{},'output',{});
  for i=1:length(sys1outputs)
    if sys1.getOutputFrame.getFrameByNum(i).dim<1, continue; end
    output_select(i).system=1;
    output_select(i).output=sys1outputs(i);
  end
  for i=1:length(sys2outputs)
    if sys2.getOutputFrame.getFrameByNum(i).dim<1, continue; end
    output_select(end+1).system=2;
    output_select(end).output=sys2outputs(i);
  end
end

% check that no input is used more than once
if ~isempty(input_select)
  in1=[input_select([input_select.system]==1).input];
  if length(unique(in1))<length(in1)
    in1
    error('you cannot use an input to sys1 more than once');
  end
end
in2=[connection.to_input];
if ~isempty(input_select)
  in2 = [in2,[input_select([input_select.system]==2).input]];
end
if length(unique(in2))<length(in2)
  in2
  error('you cannot use an input to sys2 more than once');
end

% now start constructing the simulink model
mdl = ['MIMOCascade_',datestr(now,'MMSSFFF')];  % use the class name + uid as the model name
new_system(mdl,'Model');
set_param(mdl,'SolverPrmCheckMsg','none');  % disables warning for automatic selection of default timestep
mdl = SimulinkModelHandle(mdl);

% construct subsystem (including demux if necessary) and output
% number for sys1
mdl.addSubsystem('system1',getModel(sys1));
sys1out = setupMultiOutput(sys1.getOutputFrame,mdl,'system1');

% construct subsystem (including mux if necessary) and input number
% for sys2
mdl.addSubsystem('system2',getModel(sys2));
sys2in = setupMultiInput(sys2.getInputFrame,mdl,'system2');

% make internal connections (adding transforms if necessary)
for i=1:length(connection)
  fr1 = getFrameByNum(sys1.getOutputFrame,connection(i).from_output);
  fr2 = getFrameByNum(sys2.getInputFrame,connection(i).to_input);
  if (fr1==fr2)
    add_line(mdl,[sys1out,'/',num2str(connection(i).from_output)],[sys2in,'/',num2str(connection(i).to_input)]);
  else
    tf = findTransform(fr1,fr2,struct('throw_error_if_fail',true));
    mdl.addSubsystem(['tf',num2str(i)],getModel(tf));
    add_line(mdl,[sys1out,'/',num2str(connection(i).from_output)],['tf',num2str(i),'/1']);
    add_line(mdl,['tf',num2str(i),'/1'],[sys2in,'/',num2str(connection(i).to_input)]);
  end
end

% add input
if length(input_select)>0
  add_block('simulink3/Sources/In1',[mdl,'/in']);
  if ~any([input_select.system]==2) && isequal([input_select.input],1:getNumFrames(sys1.getInputFrame))
    % don't add the a mux+demux unnecessarily
    add_line(mdl,'in/1','system1/1');
    newInputFrame=sys1.getInputFrame();
  else
    for i=1:length(input_select)
      infr{i}=getFrameByNum(sys{input_select(i).system}.getInputFrame,input_select(i).input);
    end
    newInputFrame=MultiCoordinateFrame.constructFrame(infr);
    newsysin = setupMultiOutput(newInputFrame,mdl,'in');
    sys1in = setupMultiInput(sys1.getInputFrame,mdl,'system1');
    sysin={sys1in,sys2in};
    for i=1:length(input_select)
      add_line(mdl,[newsysin,'/',num2str(i)],[sysin{input_select(i).system},'/',num2str(input_select(i).input)]);
    end
  end
else
  newInputFrame=[];
end

% add output
if length(output_select)>0
  add_block('simulink3/Sinks/Out1',[mdl,'/out']);
  if ~any([output_select.system]==1) && isequal([output_select.output],1:getNumFrames(sys2.getOutputFrame))
    add_line(mdl,'system2/1','out/1');
    newOutputFrame=sys2.getOutputFrame();
  else
    for i=1:length(output_select)
      outfr{i}=getFrameByNum(sys{output_select(i).system}.getOutputFrame,output_select(i).output);
    end
    newOutputFrame=MultiCoordinateFrame.constructFrame(outfr);
    newsysout = setupMultiInput(newOutputFrame,mdl,'out');
    sys2out = setupMultiOutput(sys2.getOutputFrame,mdl,'system2');
    sysout={sys1out,sys2out};
    for i=1:length(output_select)
      add_line(mdl,[sysout{output_select(i).system},'/',num2str(output_select(i).output)],[newsysout,'/',num2str(i)]);
    end
  end
else
  newOutputFrame=[];
end

% add terminators to all non-used outputs
if (getNumOutputs(sys1)>0)
  for i=setdiff(1:getNumFrames(sys1.getOutputFrame),[[connection.from_output],[output_select([output_select.system]==1).output]]);
    add_block('simulink3/Sinks/Terminator',[mdl,'/sys1term',num2str(i)]);
    add_line(mdl,[sys1out,'/',num2str(i)],['sys1term',num2str(i),'/1']);
  end
end
if (getNumOutputs(sys2)>0)
  for i=setdiff(1:getNumFrames(sys2.getOutputFrame),[output_select([output_select.system]==2).output]);
    add_block('simulink3/Sinks/Terminator',[mdl,'/sys2term',num2str(i)]);
    add_line(mdl,[sys2out,'/',num2str(i)],['sys2term',num2str(i),'/1']);
  end
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

%m = Simulink.Mask.create([mdl,'/system1']);
%m.set('Display',['fprintf(''',class(sys1),''')']);

%m = Simulink.Mask.create([mdl,'/system2']);
%m.set('Display',['fprintf(''',class(sys2),''')']);
