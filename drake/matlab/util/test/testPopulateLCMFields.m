function testPopulateLCMFields()

checkDependency('lcm');

msg = drake.lcmt_force_torque();
data.timestamp = randi(100);
data.fx = rand(1);
data.fy = rand(1);
data.fz = rand(1);
data.tx = rand(1);
data.ty = rand(1);
data.tz = rand(1);
defaultval = rand(1);

msg = populateLCMFields(msg, data, defaultval);

for f=fieldnames(data)'
  valuecheck(msg.(f{1}),data.(f{1}));
end
