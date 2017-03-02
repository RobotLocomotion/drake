function f = costFun1Step_x(psi,x0,plantSim,c)

%Adjust the set angle here:
c.psiDes = psi;

output_select(1).system = 1;
output_select(1).output = plantSim.getOutputFrame();
output_select(2).system = 2;
output_select(2).output = c.getOutputFrame();

sys = mimoFeedback(plantSim,c,[],[],[],output_select);

[ytraj,~] = simulate(sys,[0 2.5],x0);
tt=getBreaks(ytraj);
yAll = ytraj.eval(tt);

%Find changes
jumpIdx = find(diff(yAll(1,:)));
% xn = yAll(2:9,jumpIdx(1));
try
    xnplus = yAll(2:9,jumpIdx(3));
catch
    xnplus = NaN*ones(8,1);
end

% xnplus1 = yAll(2:9,jumpIdx(1));
% xnplus2 = yAll(2:9,jumpIdx(3));

xid = [-0.045; 4; 0; -3.1321];

% alpha = 0.2;
% f = xid - xnplus([3:4,7:8]);
% f = xid - alpha*xnplus1([3:4,7:8]) - (1-alpha)*xnplus2([3:4,7:8]);
f = xnplus(3);
end