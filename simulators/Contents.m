% 
% MIT Robot Locomotion Group
% RobotLib Simulators
%
% Dynamics classes:
%   Dynamics            - An abstract class which wraps a dynamics function xdot=f(t,x,u).
%   SecondOrderDynamics - An abstract class that wraps qddot = f(t,q,qdot,u).
%   ManipulatorDynamics - An abstract class that wraps H(q)qddot + C(q,qdot) = B(q)u.
%   HybridDynamics      - A class that contains mode Dynamics and mode switches.
%
