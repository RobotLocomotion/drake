function runLQR

d = CartPolePlant;
v = CartPoleVisualizer;
c = CartPoleLQR(d);

sys = cascade(feedback(d,c),v); 

for i=1:5
  simulate(sys,[0 8],c.x0+.3*randn(4,1));
end

return


c.rho
c.S

% First generate points on the boundary.
K = 5;
ball = randn(4,2*K);
ball = ball./repmat(sqrt(sum(ball.^2,1)),4,1);
bndr = (c.S^(-1/2))*ball*sqrt(c.rho);
bndr = repmat(c.x0,1,2*K) + ...
       [bndr(:,1:K) 1.1*bndr(:,K+(1:K))];

for i = 1:2*K
    x0 = bndr(:,i);
    bVerify = c.isVerified(x0);
    xtraj=simulate(d,c,[0 6],x0);
    figure(1);
    b = 0:0.1:6;
    x = eval(xtraj,b);
    plot(b,x);
    playback(v,xtraj);
    breaks=xtraj.getBreaks();
    xf = xtraj.eval(breaks(end));
%    if (checkDependency('spot_enabled') && bVerify && any(abs(xf-c.x0)>.1)) error('Verified initial condition didn''t make it to the goal'); end
end

end




