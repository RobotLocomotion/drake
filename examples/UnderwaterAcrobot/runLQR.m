function runLQR()
%% run the lqr controller from a handful of initial conditions
disp('Initializing Plant');
p = UnderwaterAcrobotPlant;
v = UnderwaterAcrobotVisualizer(p);

disp('Generating Controller');
c = balanceLQR(p);
sys = feedback(p,c);

tmax = 5;

for i=1:5
    disp(['Simulation ' mat2str(i,3)]);
    x0 = double(p.xG)+0.01*randn(4,1);
    xtraj=simulate(sys,[0 tmax],x0);
    v.playback(xtraj);
    if max(abs(eval(xtraj,tmax)-double(p.xG)))>.01
        warning('Did not stabilize');
    end
end

%playbackAVI(v,xtraj,'lqr');
end
