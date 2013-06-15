function compareSolvers(Q,f,Aeq,beq,Ain,bin,lb,ub,active)

n = length(f);
if nargin<3, Aeq=zeros(0,n); end
if nargin<4, beq=[]; end
if nargin<5, Ain=zeros(0,n); end
if nargin<6, bin=[]; end
if nargin<7, lb=-inf + 0*f; end
if nargin<8, ub=inf + 0*f; end
if nargin<9, active=[]; end

% todo: take out infs
Ain_b = [Ain; -eye(length(lb)); eye(length(ub))];
bin_b = [bin; -lb; ub];

% todo:  call fastQP here?
[x_fastqp,info_fastqp,active_fastqp] = fastQPmex(Q,f,Aeq,beq,Ain_b,bin_b,active);
[x_gurobimex,info_gurobimex,active_gurobimex] = gurobiQPmex(Q,f,Aeq,beq,Ain,bin,lb,ub,active);
[x_gurobi,info_gurobi,active_gurobi] = mygurobi(Q,f,Aeq,beq,Ain,bin,lb,ub,active);

valuecheck(x_fastqp,x_gurobi);
valuecheck(x_gurobimex,x_gurobi);

end

