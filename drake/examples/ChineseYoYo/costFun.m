function phi = costFun(psi, x, plantSim, c)

xd = [-0.045; 4; 0; -3.1321];
N = 5;      % Desired number of bounces
% N = round(psi(end),0);

xplus = x;
for i = 1:N
    c.psiDes = psi(i);

    output_select(1).system = 1;
    output_select(1).output = plantSim.getOutputFrame();
    output_select(2).system = 2;
    output_select(2).output = c.getOutputFrame();
    
    sys = mimoFeedback(plantSim,c,[],[],[],output_select);

    [ytraj,~] = simulate(sys,[0 2.5],xplus);
    tt=getBreaks(ytraj);
    yAll = ytraj.eval(tt);
    
    % Find mode changes
    jumpIdx = find(diff(yAll(1,:)));
%     try
        xplus = yAll(1:9,jumpIdx(3));
%     catch
%         xplus = [1; Inf*ones(8,1)];
%     end
end

phi = xd - xplus([3:4,7:8]+1);
% phi = N/10*phi;