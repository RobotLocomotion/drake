    function [xdot,dxdot] = constrainedDynamics(obj,t,x,u)
      nq = obj.getNumPositions;
      nv = obj.getNumVelocities;
      assert(nq == nv) % not ready yet if it isn't true
      nu = obj.getNumInputs;
      q=x(1:nq);
      qd=x((nq+1):end);
      
      nargout = 2;
      
            
      if nargout > 1
        [H,C,B,dH,dC,dB] = manipulatorDynamics(obj,q,qd);
        Hinv = inv(H);
        dHinv = invMatGrad(H,dH);
      else
        [H,C,B] = manipulatorDynamics(obj,q,qd);
        Hinv = inv(H);
      end
      
      
      if nargout > 1
        [phi,J,dJ,Jdotqd,dJdotqd] = obj.positionConstraintsWithJdot(q,qd);
      else
        [phi,J,dJ,Jdotqd] = obj.positionConstraintsWithJdot(q,qd);
      end
      
      dJ = [reshape(dJ,prod(size(J)),[]) zeros(prod(size(J)),nv)];
      dJ_transpose = transposeGrad(dJ,size(J));

      if ~isempty(phi)
        if isempty(u)
          constraint_force = -J'*(pinv(J*Hinv*J')*(J*Hinv*(-C) + Jdotqd));
        else
          constraint_force = -J'*(pinv(J*Hinv*J')*(J*Hinv*(B*u-C) + Jdotqd));
        end

        if nargout > 1
          % Compute gradient of Y=-J^T*inv(J'inv(H)J)
          % first, R=J*inv(H)*J'
          R = J*Hinv*J';
          dR = matGradMultMat(J*Hinv,J',matGradMultMat(J,Hinv,dJ,dHinv),dJ_transpose);
          
          Y = -J'*pinv(R);
          dY = matGradMultMat(-J',pinv(R),-dJ_transpose,invMatGrad(R,dR));

          % gradient of Z = J*inv(H)*(Bu - C) + Jdotqd
          ZZ = J*Hinv;
          dZZ = matGradMultMat(J,Hinv,dJ,dHinv);
          if isempty(u)
            Z = -ZZ*C;
            dZ = -matGradMultMat(ZZ,C,dZZ,dC);
          else
            Z = ZZ*(B*u - C) + Jdotqd;
            dZ = [matGradMultMat(ZZ,B*u-C,dZZ,matGradMult(dB,u)-dC) + dJdotqd, ZZ*B];
          end
          dconstraint_force = matGradMultMat(Y,Z,[dY zeros(size(dY,1),nu)],dZ);
        end
      else
        constraint_force = 0;
        dconstraint_force = zeros(nv,nq+nv+nu);
      end
      
      if isempty(u)
        qdd = Hinv*(constraint_force - C);
      else
        qdd = Hinv*(B*u + constraint_force - C);
      end
      xdot = [qd;qdd];

      if nargout > 1
        if isempty(u)
          tau = constraint_force - C;
          dtau = dconstraint_force - [dC zeros(nq,nu)];
          
        else
          tau = B*u + constraint_force - C;
          dtau = dconstraint_force + [-dC+matGradMult(dB,u), B];          
        end
        dqdd = matGradMultMat(Hinv,tau,[dHinv zeros(nq^2,nu)],dtau);
        dxdot = [zeros(nq) eye(nq) zeros(nq,nu);dqdd];
        dxdot = [zeros(nq+nv,1) dxdot];
      end      
    end