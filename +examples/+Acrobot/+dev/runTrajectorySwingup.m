function runTrajectorySwingup
import examples.Acrobot.*;
import examples.Acrobot.dev.TrajectorySwingup;

d = AcrobotPlant;
v = AcrobotVisualizer(d);
c = TrajectorySwingup(d);

for i=1:5
  xtraj = simulate(d,c,[0 6],.01*randn(4,1));
  playback(v,xtraj);
end
