function runSwingUp
r = TWIP();
[~, xtraj] = r.swingUpTrajectory();
v = TWIPVisualizer(r);
v.playback(xtraj, struct('slider', true));
end

