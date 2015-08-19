function runSwingUp

p = CartPolePlant();

[utraj,xtraj]=swingUpTrajectory(p);

v = CartPoleVisualizer(p);
v.playback(xtraj);

