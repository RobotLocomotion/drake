function [utraj,xtraj,info] = trajectoryOptimization(sys,costFun,finalCostFun,x0,utraj0,con,options)
% trajectoryOptimization
%
% Todo:
%
% Help file for trajectory design constraints. 
%
% All of the trajectory design algorithms support all of 
% these constraints (or produce a warning if they do not).
%
% Assuming that the constraint structure is name 'con', valid 
% constraints specified in the structure are:
%
% Input constraints:
%  con.u.lb        forall t, lb <= u(t) <= ub
%  con.u.ub 
%
% State constraints: 
%  con.x.lb        forall t, lb <= x(t) <= ub
%  con.x.ub 
%  con.x.A         forall t, A x(t) <= b
%  con.x.b
%  con.x.Aeq       forall t, Aeq x(t) = beq
%  con.x.beq
%  con.x.c         forall t, c(x(t)) <= 0
%  con.x.ceq       forall t, ceq(x(t)) == 0
%     % to provide gradients of the constraint, these functions should
%     % output  [f,dfdx] = c(x) , where size(dfdx) = [len(f),len(x)]
%
% Initial State Constraints:
%  con.x0.*  
% can take any of the fields described for con.x , but applies only for t = t0.
%
% Final State Constraints:
%  con.xf.*  
% can take any of the fields described for con.x , but applies only for t = tf.
%
% Time Constraints:
%  con.T.lb     lb <= tf - t0 <= ub
%  con.T.ub 
%
% Hybrid Mode Constraints
% For hybrid systems, all of the constraints above can exist inside a
% mode cell array, e.g.
%  con.mode{i}.mode_num    
%  con.mode{i}.u    input (for this mode) constraint structure defined above
%  con.mode{i}.x    state (for this mode) constraint structure defined above
%  con.mode{i}.x0   initial (for this mode) state constraint structure defined above
%  con.mode{i}.xf   final (for this mode) state constraint structure defined above
%  con.mode{i}.T    time (for this mode) constraint structure defined above
% Note that the mode indices enforce a total ordering of the modes in the solution 
%
% Periodicity Constraints
%  con.periodic = true       x(t0) = x(tf)
%
%
% The following options are supported by one or more of the trajectory
% design algorithms:
%
%  ode_solver   - name of a <a href="http://www.mathworks.com/help/toolbox/simulink/ug/f11-69449.html">Simulink ode solver</a>
%  optimizer    - 'fmincon','snopt', ... 
%  grad_method  - 'numerical', 'bptt', 'rtrl', ...
%  grad_test    - true or false



% More things to implement / think about:
%  - do i ever need to handle c(x,u) type constraints?
%  - will users want to call c and ceq at the same time, e.g. to save
%    computation time?  it's important at the trajectory level, but probably
%    not at the individual x level, right?=======
% todo:  do i ever need to handle c(t,x,u) type constraints?





global SNOPT_USERFUN;

checkDependency('snopt_enabled');

if (nargin<6) con = struct(); end
if (nargin<7) options = struct(); end
if (~isfield(options,'grad_test')) options.grad_test = false; end
if (~isfield(options,'warning')) options.warning = true; end
if (~isfield(options,'method')) options.method = 'dircol'; end

switch (options.method)
  case 'dircol'
    transcriptionFun=@dircolSNOPTtranscription;
  case 'rtrl'
    transcriptionFun=@rtrlSNOPTtranscription;
  case 'multipleShooting'
    transcriptionFun=@multipleShootingSNOPTtranscription;
  otherwise
    error(['method ', options.method,' unknown, or not implemented yet']);
end

[w0,wlow,whigh,Flow,Fhigh,A,iAfun,jAvar,iGfun,jGvar,SNOPT_USERFUN,wrapupfun,iname,oname] = transcriptionFun(sys,costFun,finalCostFun,x0,utraj0,con,options);

if (options.grad_test)
  gradTest(@(w)gradTestFun(w,A,iAfun,jAvar,iGfun,jGvar),w0,struct('input_name',{{iname}},'output_name',{oname},'tol',.01));
end

%% Run SNOPT
snset('superbasics=200');  % to do: make this an option?
[w,F,info] = snopt(w0,wlow,whigh,Flow,Fhigh,'snoptUserfun',0,1,A,iAfun,jAvar,iGfun,jGvar);

if (info~=1 && options.warning) 
  [str,cat] = snoptInfo(info);
  warning('SNOPT:InfoNotOne',['SNOPT exited w/ info = ',num2str(info),'.\n',cat,': ',str,'\n  Check p19 of Gill06 for more information.']);  
end

[utraj,xtraj] = wrapupfun(w);

end


function [f,df] = gradTestFun(w,A,iAfun,jAvar,iGfun,jGvar)
  [f,G] = snoptUserfun(w);
  f  = sparse(iAfun,jAvar,A,length(f),length(w))*w + f;
  df = sparse(iAfun,jAvar,A,length(f),length(w)) + sparse(iGfun,jGvar,G,length(f),length(w));
end


