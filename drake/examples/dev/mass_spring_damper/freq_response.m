function freq_response

N = 20;

p = MassSpringDamperPlant();
v = MassSpringDamperVisualizer(p);

f = 10*pi/8;
u = FunctionHandleTrajectory(@(t)10*sin(f*t),1,[0 10]);
u = setOutputFrame(u,p.getInputFrame);
sys = cascade(u,p);

xtraj = simulate(sys,[0,10],zeros(2,1));
t=xtraj.getBreaks();
stem(t,eval(x,t));
%v.playback(xtraj);