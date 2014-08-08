function createBandS
load('data/traj.mat');

ts = {};
B_points = {};
for i=1:length(Btraj)
  ts{i} = Btraj{i}.getBreaks();
  B_points{i} = Btraj{i}.eval(ts{i});
end
B_ts = horzcat(ts{:});
B_points = cat(3,B_points{:});
B = PPTrajectory(foh(B_ts,B_points));

ts = {}; % should be the same as above, but just in case
S_points = {};
for i=1:length(S_full)
  ts{i} = S_full{i}.getBreaks();
  S_points{i} = S_full{i}.eval(ts{i});
end
S_ts = horzcat(ts{:});
S_points = cat(3,S_points{:});
S = PPTrajectory(foh(S_ts,S_points));

save('traj.mat','xtraj','utraj','S','S_full','S_ts','B','Btraj','B_ts');

