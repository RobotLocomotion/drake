function testPopulateLCMFields()

checkDependency('lcm');

msg = drake.examples.Pendulum.lcmt_pendulum_x();
data = struct('timestamp', randi(100), 'theta', rand(1));
defaultval = rand(1);

msg = populateLCMFields(msg, data, defaultval);
valuecheck(msg.timestamp, data.timestamp);
valuecheck(msg.theta, data.theta);
valuecheck(msg.thetaDot, defaultval);