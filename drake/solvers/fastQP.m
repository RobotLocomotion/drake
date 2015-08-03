function [x,active,fail] = fastQP(Qdiag,f,Ain,bin,Aeq,beq,active)
    %min 1/2 * x'diag(Qdiag)'x + f'x s.t A x = b, Ain x <= bin 
    %using active set method.  Iterative solve a linearly constrained
    %quadratic minimization problem where linear constraints include
    %Ain(active,:)x == bin(active).  Quit if all dual variables associated
    %with these equations are positive (i.e. they satisfy KKT conditions).
    
    %Note: 
    % fails if QP is infeasible.  
    % active == initial rows of Ain to treat as equations.
    % Only supports diagonal Q.  Can support more general matrices
    % at cost of more expensive linear algebra.
    % Frank Permenter - June 6th 2013
    
    fail = 0;
    iterCnt = 0;
    MAX_ITER = 1000;
    
    M = size(Aeq,1);
    N = size(Aeq,2);
    
    if size(Ain,1) == 0
       Ain = zeros(0,N);
       bin = zeros(0,0);
    end
    
    Aact = Ain(active,:);
    bact = bin(active,1);

    %calculate a bunch of stuff that is constant during each iteration
    QinvDiag = 1./(10^-8+Qdiag);
    Qinv = diag(QinvDiag);
    QinvAteq = Qinv*Aeq';
    minusQinvf = -QinvDiag.*f; 

    while(1)

        A = [Aeq;Aact];
        b = [beq;bact];

        %Solve  optimality conditions H * [x;lam] = [-f;b] for equality 
        %constrained problem using Schur complements.
        % H = [[Q , A'];
        % [A,zeros(M+Mact,M+Mact)]];
        
        QinvAt= [QinvAteq,Qinv*Aact'];
        lam = -(A*QinvAt)\[b+(f'*QinvAt)'];
        x = minusQinvf - QinvAt*lam;   
        lamIneq = lam(M+1:end);
        
        if size(Ain,1) == 0
            active = [];
            break; 
        end
 
        violated = find( Ain*x-bin >= 10^-6);   
        if isempty(violated) && all(lamIneq >= 0)
            break; 
        end
        
        active = active(lamIneq >= 0,:);
        active = [active;violated];
        
        Aact = [Ain(active,:)];
        bact = [bin(active,:)];
        
        iterCnt = iterCnt + 1;
        
        if (iterCnt > MAX_ITER)
           warning('Max iter reached. Problem is likely infeasible'); 
           fail = 1;
           break;
        end
        
    end



end