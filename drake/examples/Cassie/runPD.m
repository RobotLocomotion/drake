function runPD

options.use_bullet = false;
options.floating = true;
urdf = fullfile(getDrakePath(), 'examples', 'Cassie', 'urdf', 'cassie_no_loops.urdf');
r = Cassie(urdf,options);

v = r.constructVisualizer;


load(fullfile(getDrakePath(), 'examples', 'Cassie', 'data', 'standing_state.mat'));
x0=xstar;
u0 = zeros(r.getNumInputs,1);

% v.draw(0,x0);
% v.inspector
% keyboard

% [~,df] = r.update(10,x0,u0);
% 
% A = df(:,1+(1:r.getNumStates));
% B = df(:,1+r.getNumStates+(1:r.getNumInputs));
% Q = eye(r.getNumStates);
% R = 0.01*eye(r.getNumInputs);
% K = lqr(A,B,Q,R);

kp = 20;
kd = sqrt(kp)*1.5;

[~,~,B] = r.manipulatorDynamics(x0,u0);
K = B'*[kp*eye(r.getNumPositions),kd*eye(r.getNumVelocities)];

ltisys = LinearSystem([],[],[],[],[],-K);
ltisys = setInputFrame(ltisys,CoordinateFrame([r.getStateFrame.name,' - ', mat2str(x0,3)],length(x0),r.getStateFrame.prefix));
r.getStateFrame.addTransform(AffineTransform(r.getStateFrame,ltisys.getInputFrame,eye(length(x0)),-x0));
ltisys.getInputFrame.addTransform(AffineTransform(ltisys.getInputFrame,r.getStateFrame,eye(length(x0)),+x0));
ltisys = setOutputFrame(ltisys,r.getInputFrame);

sys = feedback(r,ltisys);
% Forward simulate dynamics with visulazation, then playback at realtime
S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);
warning(S);
xtraj = simulate(sys,[0 2],x0);
playback(v,xtraj,struct('slider',true));

end