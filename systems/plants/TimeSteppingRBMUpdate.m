classdef TimeSteppingRBMUpdate < TimeSteppingRigidBodyManipulator
  
methods
  function obj = TimeSteppingRBMUpdate(file,dt)
    obj@TimeSteppingRigidBodyManipulator(file,dt);
  end
  
    function [obj,z,Mqdn,wqdn,dz,dMqdn,dwqdn] = solveLCP(obj,t,x,u)
      % do LCP time-stepping

      % todo: implement some basic caching here
      if cacheHit(obj,t,x,u,nargout)
        z = obj.LCP_cache.z;
        Mqdn = obj.LCP_cache.Mqdn;
        wqdn = obj.LCP_cache.wqdn;
        if nargout > 4
          dz = obj.LCP_cache.dz;
          dMqdn = obj.LCP_cache.dMqdn;
          dwqdn = obj.LCP_cache.dwqdn;
        end
      else

        obj.LCP_cache.t = t;
        obj.LCP_cache.x = x;
        obj.LCP_cache.u = u;
        obj.LCP_cache.nargout = nargout;

        num_q = obj.manip.num_positions;
        q=x(1:num_q); qd=x(num_q+(1:num_q));
        h = obj.timestep;

        if (nargout<5)
          [H,C,B] = manipulatorDynamics(obj.manip,q,qd);
          if (obj.num_u>0 && ~obj.position_control) tau = B*u - C; else tau = -C; end
        else
          [H,C,B,dH,dC,dB] = manipulatorDynamics(obj.manip,q,qd);
          if (obj.num_u>0 && ~obj.position_control)
            tau = B*u - C;
            dtau = [zeros(num_q,1), matGradMult(dB,u) - dC, B];
          else
            tau = -C;
            dtau = [zeros(num_q,1), -dC, zeros(size(B))];
          end
        end

        if (obj.position_control)
          pos_control_index = getActuatedJoints(obj.manip);
          nL = 2*length(pos_control_index);
        else
          nL = sum([obj.manip.joint_limit_min~=-inf;obj.manip.joint_limit_max~=inf]); % number of joint limits
        end
        nContactPairs = obj.manip.getNumContactPairs;
        nP = obj.manip.num_position_constraints;  % number of position constraints
        nV = obj.manip.num_velocity_constraints;

        if (nContactPairs+nL+nP+nV==0)
          z = [];
          Mqdn = [];
          wqdn = qd + h*(H\tau);
          if (nargout>4) error('need to implement this case'); end
          return;
        end

        % Set up the LCP:
        % z >= 0, Mz + w >= 0, z'*(Mz + w) = 0
        % for documentation below, use slack vars: s = Mz + w >= 0
        %
        % use qn = q + h*qdn
        % where H(q)*(qdn - qd)/h = B*u - C(q) + J(q)'*z
        %  or qdn = qd + H\(h*tau + J'*z)
        %  with z = [h*cL; h*cP; h*cN; h*beta{1}; ...; h*beta{mC}; lambda]
        %
        % and implement equation (7) from Anitescu97, by collecting
        %   J = [JL; JP; n; D{1}; ...; D{mC}; zeros(nC,num_q)]

        if (nContactPairs > 0)
          if (nargout>4)
            [phiC,normal,d,xA,xB,idxA,idxB,mu,n,D,dn,dD] = obj.manip.contactConstraints(q,true);
            nC = length(phiC);
            mC = length(D);
            dJ = zeros(nL+nP+(mC+2)*nC,num_q^2);  % was sparse, but reshape trick for the transpose below didn't work
            dJ(nL+nP+(1:nC),:) = reshape(dn,nC,[]);
            dD = cellfun(@(A)reshape(A,size(D{1},1),size(D{1},2)*size(dD{1},2)),dD,'UniformOutput',false);
            dD = vertcat(dD{:});
            dJ(nL+nP+nC+(1:mC*nC),:) = dD;
          else
            [phiC,normal,d,xA,xB,idxA,idxB,mu,n,D] = obj.manip.contactConstraints(q,true);
            nC = length(phiC);
            mC = length(D);
          end
          J = zeros(nL + nP + (mC+2)*nC,num_q)*q(1); % *q(1) is for taylorvar
          lb = zeros(nL+nP+(mC+2)*nC,1);
          D = vertcat(D{:});
          J(nL+nP+(1:nC),:) = n;
          J(nL+nP+nC+(1:mC*nC),:) = D;

          contact_data.normal = normal;
          contact_data.d = d;
          contact_data.xA = xA;
          contact_data.xB = xB;
          contact_data.idxA = idxA;
          contact_data.idxB = idxB;
        else
          mC=0;
          nC=0;
          J = zeros(nL+nP,num_q);
          lb = zeros(nL+nP,1);
          if (nargout>4)
            dJ = zeros(nL+nP,num_q^2);
          end
          contact_data = struct();
        end
        obj.LCP_cache.contact_data = contact_data;
        if (nL > 0)
          if (obj.position_control)
            phiL = q(pos_control_index) - u;
            JL = sparse(1:obj.manip.num_u,pos_control_index,1,obj.manip.num_u,obj.manip.num_positions);
            phiL = [phiL;-phiL]; JL = [JL;-JL];
            % dJ = 0 by default, which is correct here
            dJL = zeros(length(phiL),num_q^2);
          else
            if (nargout<5)
              [phiL,JL] = obj.manip.jointLimitConstraints(q);
            else
              [phiL,JL,dJL] = obj.manip.jointLimitConstraints(q);
              dJ(1:nL,:) = dJL;
            end
          end
          J(1:nL,:) = JL;
        end

        %% Bilateral position constraints
        if nP > 0
          % write as
          %   phiP + h*JP*qdn >= 0 && -phiP - h*JP*qdn >= 0
          if (nargout<5)
            [phiP,JP] = obj.manip.positionConstraints(q);
          else
            [phiP,JP,dJP] = obj.manip.positionConstraints(q);
          end
          J(nL+(1:nP),:) = JP;
          lb(nL+(1:nP),1) = -inf;
        end

        %% Bilateral velocity constraints
        if nV > 0
          error('not implemented yet');  % but shouldn't be hard
        end

        M = zeros(nL+nP+(mC+2)*nC)*q(1);
        w = zeros(nL+nP+(mC+2)*nC,1)*q(1);
        active = true(nL+nP+(mC+2)*nC,1);
        active_tol = .01;

        Hinv = inv(H);
        wqdn = qd + h*Hinv*tau;
        Mqdn = Hinv*J';

        if (nargout>4)
          dM = zeros(size(M,1),size(M,2),1+2*num_q+obj.num_u);
          dw = zeros(size(w,1),1+2*num_q+obj.num_u);
          dwqdn = [zeros(num_q,1+num_q),eye(num_q),zeros(num_q,obj.num_u)] + ...
            h*Hinv*dtau - [zeros(num_q,1),h*Hinv*matGradMult(dH(:,1:num_q),Hinv*tau),zeros(num_q,num_q),zeros(num_q,obj.num_u)];
          dJtranspose = reshape(permute(reshape(dJ,size(J,1),size(J,2),[]),[2,1,3]),prod(size(J)),[]);
          dMqdn = [zeros(numel(Mqdn),1),reshape(Hinv*reshape(dJtranspose - matGradMult(dH(:,1:num_q),Hinv*J'),num_q,[]),numel(Mqdn),[]),zeros(numel(Mqdn),num_q+obj.num_u)];
        end

        % check gradients
        %      xdn = Mqdn;
        %      if (nargout>1)
        %        df = dMqdn;
        %        df = [zeros(prod(size(xdn)),1),reshape(dJ,prod(size(xdn)),[]),zeros(prod(size(xdn)),num_q+obj.num_u)];
        %      end
        %      return;

        %% Joint Limits:
        % phiL(qn) is distance from each limit (in joint space)
        % phiL_i(qn) >= 0, cL_i >=0, phiL_i(qn) * cL_I = 0
        % z(1:nL) = cL (nL includes min AND max; 0<=nL<=2*num_q)
        % s(1:nL) = phiL(qn) approx phiL + h*JL*qdn
        if (nL > 0)
          w(1:nL) = phiL + h*JL*wqdn;
          M(1:nL,:) = h*JL*Mqdn;
          active(1:nL) = (phiL + h*JL*qd) < active_tol;
          if (nargout>4)
            dJL = [zeros(prod(size(JL)),1),reshape(dJL,numel(JL),[]),zeros(numel(JL),num_q+obj.num_u)];
            if (obj.position_control)
              dw(1:nL,:) = [zeros(size(JL,1),1),JL,zeros(size(JL,1),num_q),...
                [-1*ones(length(pos_control_index),obj.num_u);1*ones(length(pos_control_index),obj.num_u)]] + h*matGradMultMat(JL,wqdn,dJL,dwqdn);
            else
              dw(1:nL,:) = [zeros(size(JL,1),1),JL,zeros(size(JL,1),num_q+obj.num_u)] + h*matGradMultMat(JL,wqdn,dJL,dwqdn);
            end
            dM(1:nL,1:size(Mqdn,2),:) = reshape(h*matGradMultMat(JL,Mqdn,dJL,dMqdn),nL,size(Mqdn,2),[]);
          end
        end

        %% Bilateral Position Constraints:
        % enforcing eq7, line 2
        if (nP > 0)
          w(nL+(1:nP)) = phiP + h*JP*wqdn;
          M(nL+(1:nP),:) = h*JP*Mqdn;
          active(nL+(1:nP)) = true;
          if (nargout>4)
            dJP = [zeros(numel(JP),1),reshape(dJP,numel(JP),[]),zeros(numel(JP),num_q+obj.num_u)];
            dw(nL+(1:nP),:) = [zeros(size(JP,1),1),JP,zeros(size(JP,1),num_q+obj.num_u)] + h*matGradMultMat(JP,wqdn,dJP,dwqdn);
            dM(nL+(1:nP),1:size(Mqdn,2),:) = reshape(h*matGradMultMat(JP,Mqdn,dJP,qMqdn),nP,size(Mqdn,2),[]);
          end
        end

        %% Contact Forces:
        % s(nL+nP+(1:nC)) = phiC+h*n*qdn  (modified (fixed?) from eq7, line 3)
        % z(nL+nP+(1:nC)) = cN
        % s(nL+nP+nC+(1:mC*nC)) = repmat(lambda,mC,1) + D*qdn  (eq7, line 4)
        % z(nL+nP+nC+(1:mC*nC)) = [beta_1;...;beta_mC]
        % s(nL+nP+(mC+1)*nC+(1:nC)) = mu*cn - sum_mC beta_mC (eq7, line 5)
        % z(nL+nP+(mC+1)*nC+(1:nC)) = lambda
        %
        % The second set of conditions gives:
        %   lambda_i >= the largest projection of the velocity vector
        %   onto the d vectors (since lambda_i >= -(D*qdn)_i for all i,
        % and by construction of d always having the mirror vectors,
        %   lambda_i >= (D_qdn)_i
        %
        % The last eqs give
        %  lambda_i > 0 iff (sum beta)_i = mu_i*cn_i
        % where i is for the ith contact.
        % Assume for a moment that mu_i*cn_i = 1, then (sum_beta)_i = 1
        % is like a constraint ensuring that sum_beta_j D_j is like a convex
        % combination of the D vectors (since beta_j is also > 0)
        % So lambda_i >0 if forces for the ith contact are on the boundary of
        % the friction cone (lambda_i could also be > 0 if some of the beta_j
        % D_j's are internally canceling each other out)
        %
        % So one solution is
        %  v_i = 0,
        %  beta_i >= 0
        %  lambda_i = 0,
        %  sum_d beta_i < mu*cn_i
        % and another solution is
        %  v_i > 0  (sliding)
        %  lambda_i = max_d (v_i)
        %  beta_i = mu*cn_i only in the direction of the largest negative velocity
        %  beta_i = 0 otherwise
        % By virtue of the eqs of motion connecting v_i and beta_i, only one
        % of these two can exist. (the first is actually many solutions, with
        % beta_i in opposite directions canceling each other out).
        if (nC > 0)
          w(nL+nP+(1:nC)) = phiC+h*n*wqdn;
          M(nL+nP+(1:nC),:) = h*n*Mqdn;

          w(nL+nP+nC+(1:mC*nC)) = D*wqdn;
          M(nL+nP+nC+(1:mC*nC),:) = D*Mqdn;
          M(nL+nP+nC+(1:mC*nC),nL+nP+(1+mC)*nC+(1:nC)) = repmat(eye(nC),mC,1);

          M(nL+nP+(mC+1)*nC+(1:nC),nL+nP+(1:(mC+1)*nC)) = [diag(mu), repmat(-eye(nC),1,mC)];

          if (nargout>4)
            % n, dn, and dD were only w/ respect to q.  filled them out for [t,x,u]
            dn = [zeros(size(dn,1),1),dn,zeros(size(dn,1),num_q+obj.num_u)];
            dD = [zeros(numel(D),1),reshape(dD,numel(D),[]),zeros(numel(D),num_q+obj.num_u)];

            dw(nL+nP+(1:nC),:) = [zeros(size(n,1),1),n,zeros(size(n,1),num_q+obj.num_u)]+h*matGradMultMat(n,wqdn,dn,dwqdn);
            dM(nL+nP+(1:nC),1:size(Mqdn,2),:) = reshape(h*matGradMultMat(n,Mqdn,dn,dMqdn),nC,size(Mqdn,2),[]);

            dw(nL+nP+nC+(1:mC*nC),:) = matGradMultMat(D,wqdn,dD,dwqdn);
            dM(nL+nP+nC+(1:mC*nC),1:size(Mqdn,2),:) = reshape(matGradMultMat(D,Mqdn,dD,dMqdn),mC*nC,size(Mqdn,2),[]);
          end

          a = (phiC+h*n*qd) < active_tol;
          active(nL+nP+(1:(mC+2)*nC),:) = repmat(a,mC+2,1);
        end

        % check gradients
        %      xdn = M;
        %      if (nargout>1)
        %        df = reshape(dM,prod(size(M)),[]);
        %      end
        %      return;


        while (1)
          z = zeros(nL+nP+(mC+2)*nC,1);
          if any(active)
            z(active) = pathlcp(M(active,active),w(active),lb(active));
            if all(active), break; end
            inactive = ~active(1:(nL+nP+nC));  % only worry about the constraints that really matter.
            missed = (M(inactive,active)*z(active)+w(inactive) < 0);
          else
            inactive=true(nL+nP+nC,1);
            missed = (w(inactive)<0);
          end
          if ~any(missed), break; end
          
          % otherwise add the missed indices to the active set and repeat
          warning('Drake:TimeSteppingRigidBodyManipulator:ResolvingLCP',['t=',num2str(t),': missed ',num2str(sum(missed)),' constraints.  resolving lcp.']);
          ind = find(inactive);
          inactive(ind(missed)) = false;
          % add back in the related contact terms:
          inactive = [inactive; repmat(inactive(nL+nP+(1:nC)),mC+1,1)];
          active = ~inactive;
        end

        % for debugging
        %cN = z(nL+nP+(1:nC))
        %beta1 = z(nL+nP+nC+(1:nC))
        %beta2 = z(nL+nP+2*nC+(1:nC))
        %lambda = z(nL+nP+3*nC+(1:nC))
        % end debugging
        % more debugging
%        path_convergence_tolerance = 1e-6; % default according to http://pages.cs.wisc.edu/~ferris/path/options.pdf
%        assert(all(z>=0));
%        assert(all(M*z+w>=-path_convergence_tolerance));
%        valuecheck(z'*(M*z+w),0,path_convergence_tolerance);
        % end more debugging

        obj.LCP_cache.t = t;
        obj.LCP_cache.x = x;
        obj.LCP_cache.u = u;
        obj.LCP_cache.nargout = nargout;
        obj.LCP_cache.z = z;
        obj.LCP_cache.Mqdn = Mqdn;
        obj.LCP_cache.wqdn = wqdn;
        if (nargout>4)
          % Quick derivation:
          % The LCP solves for z given that:
          % M(a)*z + q(a) >= 0
          % z >= 0
          % z'*(M(a)*z + q(a)) = 0
          % where the vector inequalities are element-wise, and 'a' is a vector of  parameters (here the state x and control input u).
          %
          % Our goal is to solve for the gradients dz/da.
          %
          % First we solve the LCP to obtain z.
          %
          % Then, for all i where z_i = 0, then dz_i / da = 0.
          % Call the remaining active constraints (where z_i >0)  Mbar(a), zbar, and  qbar(a).  then we have
          % Mbar(a) * zbar + qbar(a) = 0
          %
          % and the remaining gradients are given by
          % for all j, dMbar/da_j * zbar + Mbar * dzbar / da_j + dqbar / da_j = 0
          % or
          %
          % dzbar / da_j =  -pinv(Mbar)*(dMbar/da_j * zbar + dqbar / da_j)
          %
          % Note that there may be multiple solutions to the above equation
          %    so we use the pseudoinverse to select the min norm solution

          dz = zeros(size(z,1),1+obj.num_x+obj.num_u);
          zposind = find(z>0);
          if ~isempty(zposind)
            Mbar = M(zposind,zposind);
            dMbar = reshape(dM(zposind,zposind,:),numel(Mbar),[]);
            zbar = z(zposind);
            dwbar = dw(zposind,:);
            dz(zposind,:) = -pinv(Mbar)*(matGradMult(dMbar,zbar) + dwbar);
          end
          obj.LCP_cache.dz = dz;
          obj.LCP_cache.dMqdn = dMqdn;
          obj.LCP_cache.dwqdn = dwqdn;
        else
          obj.LCP_cache.dz = [];
          obj.LCP_cache.dMqdn = [];
          obj.LCP_cache.dwqdn = [];
        end
      end
    end
    
end

end