function [utraj,xtraj,info] = dircol(sys,costFun,finalCostFun,x0,utraj0,con,options)
% Direct collocation method.
%   Basic algorithm:  
%     u(t) is represented as a first-order spline
%     xc(t) is represented as a cubic spline
%     xd(t) is represented as a zero-order spline (zoh)
%   At every knot point, add a constraint enforcing the derivatives and
%   updates to match the spline derivatives, etc.
%
%  I use the algorithm as described in Enright91 and Hargraves86

%  todo: handle full set of constraints / options
%  todo: cost function should be on the outputs?  or just on the states?

global SNOPT_USERFUN;

checkDependency('snopt_enabled');

if (nargin<6) con = struct(); end
if (nargin<7) options = struct(); end
if (~isfield(options,'grad_test')) options.grad_test = false; end
if (~isfield(options,'warning')) options.warning = true; end

[w0,wlow,whigh,Flow,Fhigh,iGfun,jGvar,SNOPT_USERFUN,wrapupfun,iname,oname] = dircol_setup(sys,costFun,finalCostFun,x0,utraj0,con,options);

if (options.grad_test)
  gradTest(@(w)gradTestFun(w,iGfun,jGvar),w0',struct('input_name',{{iname}},'output_name',{oname},'tol',.01));
end

%% Run SNOPT
snset('superbasics=100');  % to do: make this an option?
[w,F,info] = snopt(w0',wlow',whigh',Flow',Fhigh','snopt_userfun',0,1,[],[],[],iGfun',jGvar');

if (info~=1 && options.warning) 
  [str,cat] = snopt_info(info);
  warning(['SNOPT exited w/ info = ',num2str(info),'.\n',cat,': ',str,'\n  Check p19 of Gill06 for more information.']);  
end

[utraj,xtraj] = wrapupfun(w);

end


function [f,df] = gradTestFun(w,iGfun,jGvar)
  [f,G] = snopt_userfun(w);
  df = sparse(iGfun,jGvar,G);
end


