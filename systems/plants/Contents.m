% 
% MIT Robot Locomotion Group
% RobotLib Plant Dynamics
%
% Plant classes:
%   SecondOrderPlant               - An abstract class that wraps qddot = f(t,q,qdot,u).
%   ManipulatorPlant               - An abstract class that wraps H(q)qddot + C(q,qdot,f_ext) = B(q)u.
%   SimulationConstructionSetPlant - Wraps a java SCS Robot object
%
%
% Supporting classes and utilities:


%
%    todo: add support for discrete state plants (discrete time)
%    todo: add support for stochastic plants (continuous and discrete in state and time)
