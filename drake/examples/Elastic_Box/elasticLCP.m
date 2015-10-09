function [states, times] = elasticLCP(r, x0, tf, eps)
% Solves an elastic LCP as described below:
% 
% v_{k+1} = v_{k} + H^-1*(B_{k}*u-C_{k} + J_{k}'*(lambda_c + lambda_d))
% [[lambda_d = e*lambda_c where e is restitution coefficient]]
% v_{c} = v_{k} + H^-1*(B_{k}*u-C_{k}+J_{k}'*lambda_c)
% 0 <= lambda_c \perp phi_{k} + h*J_{k}*v_{c} >= 0
% 

states = x0;
times = 0;
phi_tol = 1e-3;
h = 0.001;
t = 0;
q = x0(1:3);
v = x0(4:6);

while(t < tf)
    kinsol = r.doKinematics(q, v);
    [phi, ~, ~, ~, ~, ~, ~, ~, n, ~] = r.contactConstraints(kinsol, false);
    
    contact_inds = find(phi + h*n*v < phi_tol);
    
    [H, C] = r.manipulatorDynamics(q, v, 0);
    kinsol = r.doKinematics(q+h*v, v);
    [~, ~, ~, ~, ~, ~, ~, mu, n, D] = r.contactConstraints(kinsol, false);
    D = vertcat(D{:});
    impulse = -h*C; % -Hv - h*(B*u-C) --> presently no input
    nC = numel(contact_inds);
    
    z = elasticLCP(H, v, impulse, mu(contact_inds),...
        n(contact_inds, :), D([contact_inds, contact_inds+4], :), nC, eps);

    v = z(1:3);
    q = q+h*v;
    t = t+h;
    
    states = [states, [q; v]];
    times = [times, t];
end

xtraj_elastic = PPTrajectory(foh(times, states));
vv = r.constructVisualizer();
xtraj_elastic = xtraj_elastic.setOutputFrame(r.getStateFrame);
vv.playback(xtraj_elastic, struct('slider', true));
 
function [z] = elasticLCP(H, v, impulse, mu, n, D, nC, eps)
if (nC > 0) % to avoid path complaining about empty matrices in the falling case
    %display('Collision!');
    mcpmat = [H, -n', -D', zeros(3, nC);...
        n, zeros(nC, 4*nC);
        D, zeros(2*nC, 3*nC), [eye(nC); eye(nC)];...
        zeros(nC, 3), diag(mu), -eye(nC), -eye(nC), zeros(nC, nC)];
    mcpvec = [-H*v-impulse; 2*eps*n*v; zeros(3*nC, 1)];
    
    H = mcpmat(1:3, 1:3);
    G = -mcpmat(1:3, 4:end);
    N = mcpmat(4:end, 4:end);
    
    b = mcpvec(1:3);
    w = mcpvec(4:end);
    
    lcpmat = G'*(H\G) + N;
    lcpvec = w-G'*(H\b);
    
    z = pathlcp(lcpmat, lcpvec, zeros(numel(lcpvec),1), inf(numel(lcpvec),1));
    v = (1+eps)*H\(G*z - b);
    z = [v; z];

else
    lcpmat = H;
    lcpvec = -H*v-impulse;
    
    z = pathlcp(lcpmat, lcpvec, -inf(numel(lcpvec), 1), inf(numel(lcpvec), 1));
end
end

end