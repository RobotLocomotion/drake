function runValueIteration

plant = PendulumPlant;
options.dt = 1e-2;
options.gamma = .95;

xbins = {[0:.2:2*pi],[-4:.2:4]};
mdp = MarkovDecisionProcess.discretizeSystem(plant,@costfun,xbins,linspace(-1,1,9),options);

function drawfun(J)
  surf(xbins{1},xbins{2},reshape(J,length(xbins{1}),length(xbins{2}))');
  xlabel('theta');
  ylabel('thetadot');
  view(0,90);
  axis tight;
end

valueIteration(mdp,0.01,@drawfun);

end

function g = costfun(sys,x,u)
  xd = [pi;0];
  g = (x-xd)'*(x-xd) + u^2;
end

