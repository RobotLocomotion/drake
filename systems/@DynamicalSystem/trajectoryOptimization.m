function [utraj,xtraj,info,trans_info] = trajectoryOptimization(sys,costFun,finalCostFun,x0,utraj0,con,options)
% trajectoryOptimization
%
% This file describes the interface used by all of the trajectory
% optimization tools in drake.  
%
% @param costFun handle to scalar function g(t,x(t),u(t)).  total cost is
%            integral of g over the trajectory + h
% @param finalCostFun handle to a scalar function h(tf,x(tf)).  
% @param x0 a double vector which describes the initial condition
% @param utraj0 a Trajectory object which the initial u(t)
% @param con a structure describing the constraints (details below)
% @param options a structure describing the options (details below)
%
% All of the trajectory design algorithms support all of 
% these constraints (or produce a warning if they do not).
%
% Assuming that the constraint structure is name 'con', valid 
% constraints specified in the structure are:
%
% <pre>
% State constraints: 
%  con.x.lb        forall t, lb <= x(t) <= ub
%  con.x.ub 
%  con.x.A         forall t, A x(t) <= b
%  con.x.b
%  con.x.Aeq       forall t, Aeq x(t) = beq
%  con.x.beq
%  con.x.c         forall t, c(x(t)) <= 0
%  con.x.ceq       forall t, ceq(x(t)) == 0
%      to provide gradients of the constraint, these functions should
%      output  [f,dfdx] = c(x) , where size(dfdx) = [len(f),len(x)]
%
% Initial state Constraints:
%  con.x0.*  
% can take any of the fields described for con.x , but applies only for t = t0.
%
% Final state Constraints:
%  con.xf.*  
% can take any of the fields described for con.x , but applies only for t = tf.
%
%
% Input constraints:
%  con.u.lb        forall t, lb <= u(t) <= ub
%  con.u.ub 
%
% Initial input constraints:
%  con.u0.*
% Final input constraints:
%  con.uf.*
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
%  con.mode{i}.u0
%  con.mode{i}.uf
%  con.mode{i}.x    state (for this mode) constraint structure defined above
%  con.mode{i}.x0   initial (for this mode) state constraint structure defined above
%  con.mode{i}.xf   final (for this mode) state constraint structure defined above
%  con.mode{i}.T    time (for this mode) constraint structure defined above
% Note that the mode indices enforce a total ordering of the modes in the solution 
% You can also impose the following inter-mode constraints:
%  con.u_const_across_transitions = true
%
% Periodicity Constraints
%  con.periodic = true       x(t0)=x(tf) and u(t0)=u(tf)
% </pre>
%
% The following options are supported by one or more of the trajectory
% design algorithms:
%
%  @options ode_solver name of a <a href="http://www.mathworks.com/help/toolbox/simulink/ug/f11-69449.html">Simulink ode solver</a>
%  @options optimizer 'fmincon','snopt', ... 
%  @options grad_method 'numerical', 'bptt', 'rtrl', ...
%  @options grad_test true or false
%  @options xtape0 'rand', 'simulate', 'x0toxf', ...
%  @options trajectory_cost_fun function handle to a one-shot cost function Jtraj(T,X,U),
%        with T=[t0,t1,...tN],X=[x0,x1,...,xN],U=[u0,u1,...,uN], which is
%        added to the original cost.  e.g., 
%           J = Jtraj(T,X,U) + h(tf,x(tf)) + int_t0^tf g(t,x,u)dt
%  @options plan_publisher handle to an object that publishes the plan (in
%  progress with plan_publisher.publish(t,x,u);


% More things to implement / think about:
%  - do i ever need to handle c(x,u) type constraints?
%  - will users want to call c and ceq at the same time, e.g. to save
%    computation time?  it's important at the trajectory level, but probably
%    not at the individual x level, right?=======
% todo:  do i ever need to handle c(t,x,u) type constraints?





global SNOPT_USERFUN;

checkDependency('snopt');

if (nargin<6) con = struct(); end
if (nargin<7) options = struct(); end
if (~isfield(options,'grad_test')) options.grad_test = false; end
if (~isfield(options,'warning')) options.warning = true; end
if (~isfield(options,'method')) options.method = 'dircol'; end


switch (options.method)
  case 'dircol'
    transcriptionFun=@dircolSNOPTtranscription;
  case 'implicitdirtran'
    transcriptionFun=@implicitDirtranSNOPTtranscription;
  case 'rtrl'
    transcriptionFun=@rtrlSNOPTtranscription;
  otherwise
    error(['method ', options.method,' unknown, or not implemented yet']);
end

[w0,wlow,whigh,Flow,Fhigh,A,iAfun,jAvar,iGfun,jGvar,SNOPT_USERFUN,wrapupfun,iname,oname] = transcriptionFun(sys,costFun,finalCostFun,x0,utraj0,con,options);

if (options.grad_test)
  gradTest(@(w)gradTestFun(w,A,iAfun,jAvar,iGfun,jGvar),w0,struct('input_name',{{iname}},'output_name',{oname},'tol',.01));
end

%% set up snopt params
% note: use options.param = val to set, where param is the string below
% with all whitespaces removed!
setSNOPTParam(options,'Major Iterations Limit',1000);
setSNOPTParam(options,'Minor Iterations Limit',500);
setSNOPTParam(options,'Major Optimality Tolerance',1e-6);
setSNOPTParam(options,'Major Feasibility Tolerance',1e-6);
setSNOPTParam(options,'Minor Feasibility Tolerance',1e-6);
setSNOPTParam(options,'Superbasics Limit',200);
setSNOPTParam(options,'Derivative Option',0);
setSNOPTParam(options,'Verify Level',0);
setSNOPTParam(options,'Iterations Limit',10000);


%% Run SNOPT
[w,F,info] = snopt(w0,wlow,whigh,Flow,Fhigh,'snoptUserfun',0,1,A,iAfun,jAvar,iGfun,jGvar);

if (info~=1 && options.warning) 
  [str,cat] = snoptInfo(info);
  warning('SNOPT:InfoNotOne',['SNOPT exited w/ info = ',num2str(info),'.\n',cat,': ',str,'\n  Check p19 of Gill06 for more information.']);  

%   if (floor(info/10)==4)  % in the 40's... gradients are suspicious
%     disp('since the gradients are suspicious, i''ll run gradtest:');
%     options.grad_test=true;
%     [w0,wlow,whigh,Flow,Fhigh,A,iAfun,jAvar,iGfun,jGvar,SNOPT_USERFUN,wrapupfun,iname,oname] = transcriptionFun(sys,costFun,finalCostFun,x0,utraj0,con,options);
%     gradTest(@(x)gradTestFun(x,A,iAfun,jAvar,iGfun,jGvar),w,struct('input_name',{{iname}},'output_name',{oname},'tol',.01));
%   end
end

try
  [utraj,xtraj,trans_info] = wrapupfun(w);
catch
  [utraj,xtraj] = wrapupfun(w);
end

end


function [f,df] = gradTestFun(w,A,iAfun,jAvar,iGfun,jGvar)
  [f,G] = snoptUserfun(w);
  f  = sparse(iAfun,jAvar,A,length(f),length(w))*w + f;
  df = sparse(iAfun,jAvar,A,length(f),length(w)) + sparse(iGfun,jGvar,G,length(f),length(w));
end

function setSNOPTParam(options,paramstring,default)
  str=paramstring(~isspace(paramstring));
  if (isfield(options,str))
    snset([paramstring,'=',num2str(getfield(options,str))]);
  else
    snset([paramstring,'=',num2str(default)]);
  end
end
