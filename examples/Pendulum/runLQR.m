function runLQR

pd = PendulumPlant;
pv = PendulumVisualizer;
c = PendulumLQR(pd);
sys = cascade(feedback(pd,c),pv);

for i=1:5
  simulate(sys,[0 4],[pi;0]+0.2*randn(2,1));
end

return;


c.rho

% First generate points on the boundary.
K = 5;
ball = randn(2,2*K);
ball = ball./repmat(sqrt(sum(ball.^2,1)),2,1);
bndr = (c.S^(-1/2))*ball*sqrt(c.rho);
bndr = repmat(c.x0,1,2*K) + ...
       [bndr(:,1:K) 1.1*bndr(:,K+(1:K))];

for i = 1:2*K
    x0 = bndr(:,i);
    bVerify = c.isVerified(x0);
    xtraj=simulate(sys,[0 3],x0);
    playback(pv,xtraj);
    breaks=xtraj.getBreaks();
    xf = xtraj.eval(breaks(end));
    if (bVerify && any(abs(xf-c.x0)>.1)) error('Verified initial condition didn''t make it to the goal'); end
end

end




