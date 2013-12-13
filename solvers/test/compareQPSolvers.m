function compareQPSolvers(Q,f,Ain,bin,Aeq,beq,lb,ub,active)

n = length(f);
if nargin<3 || isempty(Ain), Ain=zeros(0,n); end
if nargin<4, bin=[]; end
if nargin<5 || isempty(Aeq), Aeq=zeros(0,n); end
if nargin<6, beq=[]; end
if nargin<7, lb=[]; end
if nargin<8, ub=[]; end
if nargin<9, active=[]; end

options.solver = 'quadprog';
fprintf('calling quadprog. ');
tic;
[x_quadprog,fval_quadprog,info_quadprog,active_quadprog] = quadraticProgram(Q,f,Ain,bin,Aeq,beq,lb,ub,active,options);
toc;
if info_quadprog<0
  error('not worth comparing infeasible problem');
end

for solver = {'fastQP','gurobi','gurobi_mex'}
  fprintf('checking %s against quadprog. ',solver{1});
  options.solver = solver{1};
  tic
  [x,fval,info,active] = quadraticProgram(Q,f,Ain,bin,Aeq,beq,lb,ub,active,options);
  toc
  if (info_quadprog==1)
%    valuecheck(info,1);
    valuecheck(x,x_quadprog,1e-5);
    valuecheck(fval,fval_quadprog,1e-5);
    % todo: enable this:
    %  valuecheck(active,active_quadprog);
  end
end


end

% NOTEST