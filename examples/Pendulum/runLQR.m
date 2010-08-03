function runLQR

pd = PendulumDynamics;
pv = PendulumVisualizer;
c = PendulumLQR(pd);

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
    xtraj=simulate(pd,[0 3],x0,c);
    playback(pv,xtraj);
    breaks=xtraj.getBreaks();
    xf = xtraj.eval(breaks(end));
    if (bVerify && any(abs(xf-c.x0)>.1)) error('Verified initial condition didn''t make it to the goal'); end
end

end




