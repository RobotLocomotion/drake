function testDoublePendulumSim

cd(fullfile(getDrakePath,'examples','SimpleDoublePendulum'));
runSimplePend;    % feedback linearization to simulate a pendulum
runSimplePendInv; % and a gravity-inverted pendulum

