v = sym('v',[3,1]);
assume(v, 'real');
syms th real;
tt = simplify(norm(v));
dtheta_dv = subs(jacobian(tt, v), tt, th);
q = simplify([cos(th/2); sin(th/2)*v/th]);
dq = simplify(jacobian(q, v) + jacobian(q, th)*dtheta_dv);
ddq = simplify(jacobian(reshape(dq, [], 1), v) + jacobian(reshape(dq, [], 1), th)*dtheta_dv);

q_degenerate = subs(simplify(subs(q, {sin(th/2), cos(th/2)}, {taylor(sin(th/2),th),taylor(cos(th/2),th)})),{th^4,th^5},{0,0});
dq_degenerate = subs(simplify(subs(dq, {sin(th/2), cos(th/2)}, {taylor(sin(th/2),th),taylor(cos(th/2),th)})),{th^4,th^5},{0,0});
ddq_degenerate = subs(simplify(subs(ddq, {sin(th/2), cos(th/2)}, {taylor(sin(th/2),th),taylor(cos(th/2),th)})),{th^4,th^5},{0,0});

filename = fullfile(getDrakePath(), 'util', 'expmap2quatNonDegenerate');
matlabFunction(q, dq, ddq, 'Vars', {v,th}, 'File',filename);

filename = fullfile(getDrakePath(), 'util', 'expmap2quatDegenerate');
matlabFunction(q_degenerate, dq_degenerate, ddq_degenerate, 'Vars', {v,th}, 'File',filename);

%dir = pwd;
%cd(fullfile(getDrakePath(), 'util'));
%v_num = zeros(3,1);
%codegen expmap2quatImpl.m -args {v_num}
%cd(dir);

path = fullfile(getDrakePath(), 'util');
filename = 'expmap2quat';
function_name_cell = {};
output_cell = {};
inputs_cell = {};

function_name_cell{end+1} = 'expmap2quatNonDegenerate';
output_cell{end+1} = struct('var', q, 'name', 'q');
inputs_cell{end+1} = struct('var', {v, th}, 'name', {'v', 'theta'});

function_name_cell{end+1} = 'expmap2quatDegenerate';
output_cell{end+1} = struct('var', q_degenerate, 'name', 'q');
inputs_cell{end+1} = struct('var', {v, th}, 'name', {'v', 'theta'});

function_name_cell{end+1} = 'dexpmap2quatNonDegenerate';
output_cell{end+1} = struct('var', dq, 'name', 'q');
inputs_cell{end+1} = struct('var', {v, th}, 'name', {'v', 'theta'});

function_name_cell{end+1} = 'dexpmap2quatDegenerate';
output_cell{end+1} = struct('var', dq_degenerate, 'name', 'q');
inputs_cell{end+1} = struct('var', {v, th}, 'name', {'v', 'theta'});

function_name_cell{end+1} = 'ddexpmap2quatNonDegenerate';
output_cell{end+1} = struct('var', ddq, 'name', 'q');
inputs_cell{end+1} = struct('var', {v, th}, 'name', {'v', 'theta'});

function_name_cell{end+1} = 'ddexpmap2quatDegenerate';
output_cell{end+1} = struct('var', ddq_degenerate, 'name', 'q');
inputs_cell{end+1} = struct('var', {v, th}, 'name', {'v', 'theta'});

ccodeWithEigenRef(path, filename, function_name_cell, output_cell, inputs_cell);
