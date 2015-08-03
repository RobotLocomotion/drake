function testAngleAverage()

for i=1:100
  % test that it's in the correct range
  dim = ceil(10*rand());
  mu = angleAverage(10*randn(dim,1),10*randn(dim,1));
  rangecheck(mu,-pi,pi);
end

% test a couple cases where naive average fails
valuecheck(angleAverage(-pi,pi+1e-8),-pi);
valuecheck(angleAverage(0,2*pi),0);

end