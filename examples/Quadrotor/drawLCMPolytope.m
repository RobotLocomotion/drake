function drawLCMPolytope(A, b, id, highlight, lc)
if nargin < 5
  lc = lcm.lcm.LCM.getSingleton();
end
if nargin < 4
  highlight = false;
end
if nargin < 3
  id = 1;
end
if highlight
  id = -id;
end

msg = drc.lin_con_t();
msg.m = size(A, 2);
msg.b = repmat(id, msg.m, 1);

if id ~= 0
  V = iris.thirdParty.polytopes.lcon2vert(A,b)';
  msg.n = size(V, 2);
  msg.m_times_n = msg.m * msg.n;
  msg.A = reshape(V, [], 1);
end
lc.publish('DRAW_POLYTOPE', msg);

end