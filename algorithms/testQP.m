active = [];

profile on;
for i=3:995
    
    %strNum = [repmat('0',1,5-length(num2str(i))),num2str(i)];
    %filename = ['qp_data/qp_problem_',strNum,'.mat'];
    strNum = sprintf( '%.3f',i/200);
    filename = ['QPController_data/QPController_data_t=',strNum,'.mat'];

    load(filename);

    AinL = -eye(length(lb));
    AinL = AinL( ~isinf(lb),:);
    binL = -lb(~isinf(lb));
    
    AinU = eye(length(ub));
    AinU = AinU(~isinf(ub),:);
    binU = ub(~isinf(ub));
     
    Ain = [AinL;AinU];
    bin = [binL;binU];
    
    Meq = size(Aeq,1); 
    Min = size(Ain,1); 
    Q = Hqp;
    
    %[x,active]=fastQP(diag(Q)*2,f',Aeq,beq,Ain,bin,active);
    [x,active]=fastQPBlkDiag(Q*2,f',Aeq,beq,Ain,bin,active);
    
    model.sense = char(['='*ones(Meq,1);]);
    params.outputflag = 0;
    
    model.obj = f;
    model.A = sparse([Aeq]);
    model.rhs = [beq];
    model.Q = sparse(Q);
    model.lb = lb;
    model.ub = ub;
    x2 = gurobi(model,params);
    
    cG = x2.x'*Q*x2.x+f*x2.x;
    cMe = x'*Q*x+f*x;
    if (any(Ain*x-bin > 10^-6))
       error('infeasible') 
    end
    
    if (norm(cMe-cG) > 10^-1)
       error('not optimal') 
    end
    
end

profile off;

profile viewer