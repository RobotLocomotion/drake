function [utraj,xtraj,info] = rrtwrap(sys,cost,x0,utraj0,con,options)

% Wrapper function to provide the trajectory optimization interface to the
% rrt code. Allows the rrt to be used as a drop-in replacement for the other
% trajectory optimizers. 

