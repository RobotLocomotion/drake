function runLQR

d = AcrobotPlant;
v = AcrobotVisualizer(d);
c = AcrobotLQR(d);


%options = struct('plant_order',3);
%c = c.verify(options);

c.rho

for i = 1:5
  x0 = c.x0 + .1*randn(4,1);
  bVerify = c.isVerified(x0);
  xtraj=simulate(d,c,[0 3],x0);
  playback(v,xtraj);
  breaks=xtraj.getBreaks();
  xf = xtraj.eval(breaks(end));
  if (bVerify && any(abs(xf-c.x0)>.1)) error('Verified initial condition didn''t make it to the goal'); end
end

end




