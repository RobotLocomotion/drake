v = sym('v',[3,1]);
assume(v, 'real');
syms theta real;
th = simplify(norm(v));
dtheta_dv = subs(jacobian(th, v), th, theta);
q = simplify([cos(theta/2); sin(theta/2)*v/theta]);
dq = simplify(jacobian(q, v) + jacobian(q, theta)*dtheta_dv);
dq_reshaped = reshape(dq, [], 1);
ddq_reshaped = simplify(jacobian(dq_reshaped, v) + jacobian(dq_reshaped, theta)*dtheta_dv);
ddq_3d = reshape(ddq_reshaped, [4,3,3]);

ddq = reshape(permute(ddq_3d,[3,2,1]),[9,4])';
q_degenerate = subs(simplify(subs(q, {sin(theta/2), cos(theta/2)}, {taylor(sin(theta/2),theta),taylor(cos(theta/2),theta)})),{theta^4,theta^5},{0,0});
dq_degenerate = subs(simplify(subs(dq, {sin(theta/2), cos(theta/2)}, {taylor(sin(theta/2),theta),taylor(cos(theta/2),theta)})),{theta^4,theta^5},{0,0});
ddq_degenerate = subs(simplify(subs(ddq, {sin(theta/2), cos(theta/2)}, {taylor(sin(theta/2),theta),taylor(cos(theta/2),theta)})),{theta^4,theta^5},{0,0});

filename = fullfile(getDrakePath(), 'util', 'exp2quatNonDegenerate');
matlabFunction(q, dq, ddq, 'Vars', {v,theta}, 'File',filename);

filename = fullfile(getDrakePath(), 'util', 'exp2quatDegenerate');
matlabFunction(q_degenerate, dq_degenerate, ddq_degenerate, 'Vars', {v,theta}, 'File',filename);