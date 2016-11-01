%NOTEST
function Tdot = computeTdots(T, twist)
Tdot = cell(length(T), 1);
for i = 1 : length(T)
  Tdot{i} = twistToTildeForm(twist{i}) * T{i};
end
end

function ret = twistToTildeForm(twist)
omega = twist(1 : 3);
v = twist(4 : 6);
ret = [vectorToSkewSymmetric(omega), v;
       zeros(1, 4)];
end