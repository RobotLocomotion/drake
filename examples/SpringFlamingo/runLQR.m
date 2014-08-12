function runLQR(segment_number)

if nargin < 1
  segment_number=1;
end

options.twoD = true;
options.view = 'right';
options.floating = true;
options.terrain = RigidBodyFlatTerrain();
s = 'urdf/spring_flamingo_passive_ankle.urdf';
dt = 0.001;
w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
r = TimeSteppingRigidBodyManipulator(s,dt,options);
warning(w);

v = r.constructVisualizer;
v.display_dt = 0.01;

nx = getNumStates(r);
nq = getNumPositions(r);
nu = getNumInputs(r);

data_dir = fullfile(getDrakePath,'examples','SpringFlamingo','data');
traj_file = strcat(data_dir,'/traj-08-08-14.mat');
% if exist(traj_file, 'file') ~= 2
%   !wget "http://www.dropbox.com/s/i2g6fz45hl97si4/traj.mat" --no-check-certificate 
%   system(['mv traj.mat ',data_dir]);
% end
load(traj_file);
xtraj = xtraj.setOutputFrame(getStateFrame(r));
% v.playback(xtraj,struct('slider',true));

% create affine controller system
R = eye(nu);
Rinv = inv(R);
ts = Btraj{segment_number}.getBreaks;
Kbreaks = zeros(nu,nx,length(ts));
for i=1:length(ts)
  Bi = Btraj{segment_number}.eval(ts(i));
  Kbreaks(:,:,i) = -Rinv*Bi'*Straj_full{segment_number}.eval(ts(i));
end
K = PPTrajectory(foh(ts,Kbreaks));

% create new coordinate frames
iframe = CoordinateFrame([r.getStateFrame.name,' - x0(t)'],nx,r.getStateFrame.prefix);
r.getStateFrame.addTransform(AffineTransform(r.getStateFrame,iframe,eye(nx),-xtraj));
iframe.addTransform(AffineTransform(iframe,r.getStateFrame,eye(nx),xtraj));

oframe = CoordinateFrame([r.getInputFrame.name,' + u0(t)'],nu,r.getInputFrame.prefix);
oframe.addTransform(AffineTransform(oframe,r.getInputFrame,eye(nu),utraj));
r.getInputFrame.addTransform(AffineTransform(r.getInputFrame,oframe,eye(nu),-utraj));

ltvsys = AffineSystem([],[],[],[],[],[],[],K,[]);
ltvsys = setInputFrame(ltvsys,iframe);
ltvsys = setOutputFrame(ltvsys,oframe);

sys = feedback(r,ltvsys);

S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);
warning(S);

t0 = ts(1);
tf = ts(end);

traj = simulate(sys,[t0 tf],xtraj.eval(t0));
playback(v,traj,struct('slider',true));

end

