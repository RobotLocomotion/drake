function [x_opt, f_opt] = solveQPCC(Q,R,f_Q,f_R,H,C,B,J,Jf,phi0,Dphi,psi0,Dpsi,mu,u_lb,u_ub,q0,qd0,dt)
% Q - cell array of quadratic cost on state
% R - cell array of quadratic cost on input
% f_Q - cell array of linear cost on state
% f_R - cell array of linear cost on input
% H,C,B - Cell arrays of manipualtor equations
% J, Jf - Cell arrays of jacobians
% phi0 - cell array of nominal phi
% Dphi - probably the same as J?
% psi0 - nominal tangential velocity
% Dpsi - probably just [0 Jf]?
% mu - coefficient of friction
% u_lb - input bounds
% u_ub - input bounds
% q0 - current state
% qd0
% dt - timestep

  nT = length(Q);
  nC = length(phi0{1});
  nQ = size(H{1},1);
  nU = size(B{1},2);
  maxNumVisitedQP = 100;
  
  % modes: 0 - no contact
  %        1 - sticking
  %        2 - sliding+, psi > 0
  %        3 - sliding-, psi < 0

  iter = 1;
  f_opt = inf;
  x_opt = [];
  priority_queue = [];
  visited_set = [];
  
  CCSpec = generateInitialGuess();
  QP = generateQP(CCSpec);
  
  [QP_x, QP_f] = quadprog(QP.H, QP.f, QP.A, QP.b, QP.B, QP.c, QP.l, QP.u, QP.x0);
  QP.f_opt = QP_f;
  QP.x_opt = QP_x;

  priority_queue = [priority_queue; QP];
  
  while (length(visited_set) < maxNumVisitedQP) && ~isempty(priority_queue)
    QP = priority_queue(end);
    priority_queue = priority_queue(1:end-1);
    visited_set = [visited_set; QP];
    if QP.f_opt < f_opt,
      f_opt = QP.f_opt;
      x_opt = QP.x_opt;
    end
    if ~isempty(QP.x_opt)
      childQPs = generateChildQP(QP);
      
      
      for i=1:length(childQPs),
        QP = childQPs(i);
        if ~isVisited(QP.CCSpec)
          [QP_x, QP_f] = quadprog(QP.H, QP.f, QP.A, QP.b, QP.B, QP.c, QP.l, QP.u, QP.x0);
          QP.f_opt = QP_f;
          QP.x_opt = QP_x;
          if ~isempty(QP.x_opt)
            priority_queue = [priority_queue; QP];
          end
        end
      end
    end
    iter = iter + 1;
  end
  
  if iter > 2
    display(sprintf('QPCC solved in %d iterations',iter-1));
  end
  function CCSpec = generateInitialGuess()
    CCSpec = cell(nT,1);
    for j=1:nT,
      CCSpec{j} = zeros(nC,1);
      CCSpec{j}(phi0{j} < 1e-6 & abs(psi0{j}) <= 1e-6) = 1;
      CCSpec{j}(phi0{j} < 1e-6 & psi0{j} > 1e-6) = 2;
      CCSpec{j}(phi0{j} < 1e-6 & psi0{j} < -1e-6) = 3;
    end
  end

  function QP = generateQP(CCSpec)
    for i=1:length(visited_set)
      if isequal(visited_set(i).CCSpec,CCSpec)
        QP = [];
        return;
      end
    end
    nVarsi = 2*nQ + nU + 4*nC;
    nVars = nT*nVarsi;
    nEqnsi = 2*nQ + 4*nC;
    nEqns = nT*nEqnsi;
    
    nIneqsi = 4*nC;
    nIneqs = nT*nIneqsi;
    
    QP.H = zeros(nVars);
    QP.f = zeros(nVars,1);
    for j=1:nT,
      QP.H((j-1)*nVarsi + (1:2*nQ),(j-1)*nVarsi + (1:2*nQ)) = Q{j};
      QP.H((j-1)*nVarsi + 2*nQ + (1:nU),(j-1)*nVarsi + 2*nQ + (1:nU)) = R{j};
      QP.f((j-1)*nVarsi + (1:2*nQ)) = f_Q{j};
      QP.f((j-1)*nVarsi + 2*nQ + (1:nU)) = f_R{j};
    end
    
    
    QP.A = zeros(nIneqs,nVars);
    QP.b = zeros(nIneqs,1);
    
    QP.B = zeros(nEqns,nVars);
    QP.c = zeros(nEqns,1);
    
    lb = repmat([-inf(2*nQ,1);u_lb;zeros(4*nC,1)],1,nT);
    ub = repmat([inf(2*nQ,1);u_ub;inf(4*nC,1)],1,nT);
    x0 = repmat([q0;qd0;zeros(nU+4*nC,1)],1,nT);
      
    QP.l = lb(:);
    QP.u = ub(:);
    QP.x0 = x0(:);
    QP.CCSpec = CCSpec;
    
    for j=1:nT,
      %equations:
      % dynamics
      % contact_lz(1)
      % contact_lz(2)
      % ...
      % contact_lx+(1)
      % contact_lx+(2)
      % ...
      % contact_lx-(1)
      % contact_lx-(2)
      % ...
      % contact_gamma(1)
      % contact_gamma(2)
      % ...
      
      %variables
      % q,qd,u,lz(1),lz(2),...,lx+(1),lx+(2),...
        
      if j == 1,
        % dynamics
        QP.B((j-1)*nEqnsi + (1:2*nQ), (j-1)*nVarsi + (1:nVarsi)) = [eye(nQ) -dt*eye(nQ) zeros(nQ, 4*nC+nU);...
                                                                    zeros(nQ) H{j} -dt*B{j} -dt*J{j}' -dt*Jf{j}' dt*Jf{j}' zeros(nQ, nC)];
        QP.c((j-1)*nEqnsi + (1:2*nQ)) = [q0;H{j}*qd0 - dt*C{j}];        
      else
        % implement this!
        error('not implemented');
      end
      
      
      %contact equations and inequalities
      % no contact:
      i_free = find(CCSpec{j} == 0);
      QP.B((j-1)*nEqnsi + 2*nQ + i_free,(j-1)*nVarsi + 2*nQ + nU + i_free) = eye(length(i_free));  % lambda_z = 0
      QP.B((j-1)*nEqnsi + 2*nQ + nC + i_free,(j-1)*nVarsi + 2*nQ + nU + nC + i_free) = eye(length(i_free));  % lambda_x+ = 0
      QP.B((j-1)*nEqnsi + 2*nQ + 2*nC + i_free,(j-1)*nVarsi + 2*nQ + nU + 2*nC + i_free) = eye(length(i_free));  % lambda_x- = 0
      QP.B((j-1)*nEqnsi + 2*nQ + 3*nC + i_free,(j-1)*nVarsi + 2*nQ + nU + 3*nC + i_free) = eye(length(i_free));  % gamma = 0
      
      QP.A((j-1)*nIneqsi + i_free,(j-1)*nVarsi + nQ + (1:nQ)) = -dt*Dphi{j}(i_free,:);  % phi0 + Dphi*q >= 0
      QP.b((j-1)*nIneqsi + i_free) = phi0{j}(i_free);
      
      % sticking contact
      i_stick = find(CCSpec{j} == 1);
      QP.B((j-1)*nEqnsi + 2*nQ + i_stick,(j-1)*nVarsi + nQ + (1:nQ)) = dt*Dphi{j}(i_stick,:);  % phi0 + h*Dphi*qd = 0
      QP.c((j-1)*nEqnsi + 2*nQ + i_stick) = -phi0{j}(i_stick);
      
      QP.B((j-1)*nEqnsi + 2*nQ + nC + i_stick,(j-1)*nVarsi + (1:2*nQ)) = Dpsi{j}(i_stick,:);  % gamma + psi0 + Dpsi*[q;qd] = 0
      QP.B((j-1)*nEqnsi + 2*nQ + nC + i_stick,(j-1)*nVarsi + 2*nQ + nU + 3*nC + i_stick) = eye(length(i_stick));
      QP.c((j-1)*nEqnsi + 2*nQ + nC + i_stick) = -psi0{j}(i_stick);
      
      QP.B((j-1)*nEqnsi + 2*nQ + 2*nC + i_stick,(j-1)*nVarsi + (1:2*nQ)) = -Dpsi{j}(i_stick,:);  % gamma - psi0 - Dpsi*[q;qd] = 0
      QP.B((j-1)*nEqnsi + 2*nQ + 2*nC + i_stick,(j-1)*nVarsi + 2*nQ + nU + 3*nC + i_stick) = eye(length(i_stick));
      QP.c((j-1)*nEqnsi + 2*nQ + 2*nC + i_stick) = psi0{j}(i_stick);
      
      QP.B((j-1)*nEqnsi + 2*nQ + 3*nC + i_stick,(j-1)*nVarsi + 2*nQ + nU + 3*nC + i_stick) = eye(length(i_stick));  % gamma = 0
      
      QP.A((j-1)*nIneqsi + 3*nC + i_stick,(j-1)*nVarsi + 2*nQ + nU + i_stick) = -mu*eye(length(i_stick));  % mulz - lx+ - lx- = 0;
      QP.A((j-1)*nIneqsi + 3*nC + i_stick,(j-1)*nVarsi + 2*nQ + nU + nC + i_stick) = eye(length(i_stick));
      QP.A((j-1)*nIneqsi + 3*nC + i_stick,(j-1)*nVarsi + 2*nQ + nU + 2*nC + i_stick) = eye(length(i_stick));
      
      % sliding psi > 0
      i_slidingp = find(CCSpec{j} ==  2);
      QP.B((j-1)*nEqnsi + 2*nQ + i_slidingp,(j-1)*nVarsi + nQ + (1:nQ)) = dt*Dphi{j}(i_slidingp,:);  % phi0 + Dphi*q = 0
      QP.c((j-1)*nEqnsi + 2*nQ + i_slidingp) = -phi0{j}(i_slidingp);
      
      QP.B((j-1)*nEqnsi + 2*nQ + nC + i_slidingp,(j-1)*nVarsi + 2*nQ + nU + nC + i_slidingp) = eye(length(i_slidingp));  % lambda_x+ = 0
      
      QP.B((j-1)*nEqnsi + 2*nQ + 2*nC + i_slidingp,(j-1)*nVarsi + (1:2*nQ)) = -Dpsi{j}(i_slidingp,:);  % gamma - psi0 - Dpsi*[q;qd] = 0
      QP.B((j-1)*nEqnsi + 2*nQ + 2*nC + i_slidingp,(j-1)*nVarsi + 2*nQ + nU + 3*nC + i_slidingp) = eye(length(i_slidingp));
      QP.c((j-1)*nEqnsi + 2*nQ + 2*nC + i_slidingp) = psi0{j}(i_slidingp);
      
      QP.B((j-1)*nEqnsi + 2*nQ + 3*nC + i_slidingp,(j-1)*nVarsi + 2*nQ + nU + i_slidingp) = mu*eye(length(i_slidingp));  % mulz - lx+ - lx- = 0;
      QP.B((j-1)*nEqnsi + 2*nQ + 3*nC + i_slidingp,(j-1)*nVarsi + 2*nQ + nU + nC + i_slidingp) = -eye(length(i_slidingp));
      QP.B((j-1)*nEqnsi + 2*nQ + 3*nC + i_slidingp,(j-1)*nVarsi + 2*nQ + nU + 2*nC + i_slidingp) = -eye(length(i_slidingp));
      
      % sliding psi < 0
      i_slidingm = find(CCSpec{j} ==  3);
      QP.B((j-1)*nEqnsi + 2*nQ + i_slidingm,(j-1)*nVarsi + nQ +(1:nQ)) = dt*Dphi{j}(i_slidingm,:);  % phi0 + Dphi*q = 0
      QP.c((j-1)*nEqnsi + 2*nQ + i_slidingm) = -phi0{j}(i_slidingm);
      
      QP.B((j-1)*nEqnsi + 2*nQ + nC + i_slidingm,(j-1)*nVarsi + (1:2*nQ)) = Dpsi{j}(i_slidingm,:);  % gamma + psi0 + Dpsi*[q;qd] = 0
      QP.B((j-1)*nEqnsi + 2*nQ + nC + i_slidingm,(j-1)*nVarsi + 2*nQ + nU + 3*nC + i_slidingm) = eye(length(i_slidingm));
      QP.c((j-1)*nEqnsi + 2*nQ + nC + i_slidingm) = -psi0{j}(i_slidingm);
      
      QP.B((j-1)*nEqnsi + 2*nQ + 2*nC + i_slidingm,(j-1)*nVarsi + 2*nQ + nU + 2*nC + i_slidingm) = eye(length(i_slidingm));  % lambda_x- = 0
      
      QP.B((j-1)*nEqnsi + 2*nQ + 3*nC + i_slidingm,(j-1)*nVarsi + 2*nQ + nU + i_slidingm) = mu*eye(length(i_slidingm));  % mulz - lx+ - lx- = 0;
      QP.B((j-1)*nEqnsi + 2*nQ + 3*nC + i_slidingm,(j-1)*nVarsi + 2*nQ + nU + nC + i_slidingm) = -eye(length(i_slidingm));
      QP.B((j-1)*nEqnsi + 2*nQ + 3*nC + i_slidingm,(j-1)*nVarsi + 2*nQ + nU + 2*nC + i_slidingm) = -eye(length(i_slidingm));
      
      
      % contact inequalities
      % no contact: phi >= 0
      % sticking: mulz - lxp - lxm >= 0
    end
  end

  function childQPs = generateChildQP(parentQP)
    childQPs = [];
    x = reshape(parentQP.x_opt,[],nT);
    for j=1:nT,
      for k=1:nC,
        switch(parentQP.CCSpec{j}(k))
          case 0
            phi = phi0{j}(k) + dt*Dphi{j}(k,:)*x(nQ+1:2*nQ,j);
            if phi < 1e-6
              CCSpec_new = parentQP.CCSpec;
              CCSpec_new{j}(k) = 1;
              
              if ~isVisited(CCSpec_new)
                childQPs = [childQPs; generateQP(CCSpec_new)];
              end
              
              psi = psi0{j}(k) + Dpsi{j}(k,:)*x(1:2*nQ,j);
              if psi > 1e-6
                CCSpec_new{j}(k) = 2;
                if ~isVisited(CCSpec_new)
                  childQPs = [childQPs; generateQP(CCSpec_new)];
                end
              elseif psi < 1e-6
                CCSpec_new{j}(k) = 3;
                if ~isVisited(CCSpec_new)
                  childQPs = [childQPs; generateQP(CCSpec_new)];
                end
              end
            end
          case 1
            lambdaz = x(2*nQ+nU+k,j);
            lambdaxp = x(2*nQ+nU+nC+k,j);
            lambdaxm = x(2*nQ+nU+2*nC+k,j);
            if lambdaz < 1e-6
              CCSpec_new = parentQP.CCSpec;
              CCSpec_new{j}(k) = 0;
              
              if ~isVisited(CCSpec_new)
                childQPs = [childQPs; generateQP(CCSpec_new)];
              end
            elseif mu*lambdaz - lambdaxp - lambdaxm < 1e-6
              if lambdaxm < 1e-6
                CCSpec_new = parentQP.CCSpec;
                CCSpec_new{j}(k) = 3;
                
                if ~isVisited(CCSpec_new)
                  childQPs = [childQPs; generateQP(CCSpec_new)];
                end
              elseif lambdaxp < 1e-6
                CCSpec_new = parentQP.CCSpec;
                CCSpec_new{j}(k) = 2;
                
                if ~isVisited(CCSpec_new)
                  childQPs = [childQPs; generateQP(CCSpec_new)];
                end
              end
            end
          case 2
            lambdaz = x(2*nQ+nU+k,j);
            gamma = x(2*nQ+nU+3*nC+k,j);
            if lambdaz < 1e-6
              CCSpec_new = parentQP.CCSpec;
              CCSpec_new{j}(k) = 0;
              
              if ~isVisited(CCSpec_new)
                childQPs = [childQPs; generateQP(CCSpec_new)];
              end
            elseif gamma < 1e-6
              CCSpec_new = parentQP.CCSpec;
              CCSpec_new{j}(k) = 1;
              
              if ~isVisited(CCSpec_new)
                childQPs = [childQPs; generateQP(CCSpec_new)];
              end            
            end
          case 3
            lambdaz = x(2*nQ+nU+k,j);
            gamma = x(2*nQ+nU+3*nC+k,j);
            if lambdaz < 1e-6
              CCSpec_new = parentQP.CCSpec;
              CCSpec_new{j}(k) = 0;
              
              if ~isVisited(CCSpec_new)
                childQPs = [childQPs; generateQP(CCSpec_new)];
              end
            elseif gamma < 1e-6
              CCSpec_new = parentQP.CCSpec;
              CCSpec_new{j}(k) = 1;
              
              if ~isVisited(CCSpec_new)
                childQPs = [childQPs; generateQP(CCSpec_new)];
              end            
            end
        end
      end      
    end
  end

  function ret = isVisited(CCSpec)
    ret = any(cellfun(@isequal,visited_set.CCSpec,CCSpec));
%     ret = ~isempty(ismember([visited_set.CCSpec]',CCSpec','rows'));
  end
end
