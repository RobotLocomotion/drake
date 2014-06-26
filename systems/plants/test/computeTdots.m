%NOTEST
function Tdot = computeTdots(T, twist)
Tdot = cell(length(T), 1);
for i = 1 : length(T)
  Tdot{i} = twistToTildeForm(twist{i}) * T{i};
end
end