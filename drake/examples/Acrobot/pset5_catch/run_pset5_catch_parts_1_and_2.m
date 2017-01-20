% RUN THIS to generate your solution
megaclear

[p,xtraj,utraj,v,x0] = pset5_catch;

% submit x_grade below
x_grade = [xtraj.eval(xtraj.pp.breaks) repmat(xtraj.tspan(2),10,1)]';
format short
display(x_grade)
format long