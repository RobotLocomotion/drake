function [Hqddot, dHqddot, phi, psi, dPhi, dPsi, H, dH] = implicitDynamics(obj,t,x,u,lambda,cLambda)
% Compute the implicit dynamics for trajectory optimization.  This
% function is fully implemented for joint limits and contact contraints
% @param obj The Manipulator object
% @param t Time
% @param x State
% @param u Input
% @param lambda Non-contact constraint forces
% @param cLambda Contact constraint forces
% @return Hqddot H(q)*qddot from the dynamics equations
% @return dHqddot d/dq and d/qdot of H*qddot
% @return phi phi(q) the contact point positions
% @return psi psi(q) the contact point velocities
% @return dPhi dphi/dq
% @return dPsi [dpsi/dq dpsi/dqdot]

phi = [];
psi = [];
J = [];
dPsi = [];
% Compute H*qddot, one form of the implicit dynamics

q=x(1:obj.num_positions); qd=x((obj.num_positions+1):end);
nContactConst = obj.num_contacts;
%       nonContactPConst = obj.num_bilateral_constraints - nContactConst;
nonContactPConst = obj.getNumJointLimitConstraints;
nUniPConstraints = nonContactPConst;
dconstraint = 0;
constraint = 0;
nClutch = 0;

[H,C,B,dH,dC,dB] = manipulatorDynamics(obj,q,qd);
%       doKinematicsAndVelocities(obj.model,q,qd);
%kinsol = doKinematics(obj,q,true);  

if (nUniPConstraints > 0)
  [phi,J,dJ] = obj.jointLimitConstraints(q);
  %         dJdqlambda = zeros(obj.num_positions^2,1);
  %         for i=1:obj.num_positions^2,
  %           dJdqlambda(i,1) = dJ(:,i)'*lambda(1:nUniPConstraints);
  %         end
  dJdqlambda = dJ'*lambda(1:nUniPConstraints);
  dconstraint = [zeros(obj.num_positions,1) reshape(dJdqlambda,obj.num_positions,obj.num_positions)...
    zeros(obj.num_positions) zeros(obj.num_positions,obj.num_u) J' ...
    zeros(obj.num_positions,2*nContactConst + nonContactPConst - nUniPConstraints+nClutch)] + dconstraint;
  constraint = constraint + J'*lambda(1:nUniPConstraints);
end

%       if (nonContactPConst - nUniPConstraints>0)
%         error('Not yet implemented in trunk version');
%         [phi,J,dJ] = obj.positionConstraints(q);
%         dJdqlambda = zeros(obj.num_positions^2,1);
%         for i=1:obj.num_positions^2,
%           dJdqlambda(i,1) = dJ(:,i)'*lambda(nUniPConstraints+1:end);
%         end
%         dconstraint = [zeros(obj.num_positions,1) reshape(dJdqlambda,obj.num_positions,obj.num_positions)...
%           zeros(obj.num_positions) zeros(obj.num_positions,obj.num_u + nUniPConstraints) J' zeros(obj.num_positions,2*nContactConst+nClutch)] + dconstraint;
%         constraint = constraint + J'*lambda(nUniPConstraints+1:end);
%       end

if (nContactConst > 0)
  [phi_n,n,D,mu,dn,dD,psi,dPsi,phi_f,dPhi] = contactConstraints(obj,q,qd);
  %         [phi, psi, dPhi, dPsi, J, dJ]  = contactPositionsAndVelocities(obj,q,qd);
  
  %TODO: clean this up instead of just cramming into previous format
  psi = psi(1,:)';
  phi = zeros(2*length(phi_n),1);
  phi(1:2:end) = phi_f;
  phi(2:2:end) = phi_n;
  J = zeros(length(phi), obj.num_positions);
  J(1:2:end,:) = D{1};
  J(2:2:end,:) = n;
  dJ = zeros(length(phi), obj.num_positions^2);
  dJ(2:2:end,:) = reshape(dn,length(phi_n),[]);
  dJ(1:2:end,:) = reshape(dD{1},length(phi_n),[]);
  dPsi = dPsi(1:2:end,:);
  
  %         dJdqlambda = zeros(obj.num_positions^2,1)*x(1);
  %         for i=1:obj.num_positions^2,
  %           dJdqlambda(i,1) = dJ(:,i)'*cLambda;
  %         end
  dJdqlambda = dJ'*cLambda;
  dconstraint = [zeros(obj.num_positions,1) reshape(dJdqlambda,obj.num_positions,obj.num_positions)...
    zeros(obj.num_positions, obj.num_positions + obj.num_u + nonContactPConst) J' zeros(obj.num_positions, nClutch)] + dconstraint;
  constraint = constraint + J'*cLambda;
end

dHqddot = [zeros(obj.num_positions,1) -dC B ...
  zeros(obj.num_positions,nonContactPConst+2*nContactConst+nClutch)] + dconstraint;

%       if (~isempty(obj.model.passive_K) && ~isempty(obj.model.passive_B))
%         % determine which passive joints are active
%         active_ind = (q < obj.model.passive_max+1e-6).*(q > obj.model.passive_min-1e-6);
%         dHqddot(:,2:(1+2*obj.num_positions)) = dHqddot(:,2:(1+2*obj.num_positions)) - ...
%           [obj.model.passive_K*diag(active_ind) obj.model.passive_B*diag(active_ind)];
%
%         passive_force = (-obj.model.passive_K*(q - obj.model.passive_nom) -...
%           obj.model.passive_B*qd).*active_ind + obj.model.passive_offset;
%
%         Hqddot = B*u - C + constraint + passive_force;
%       else
Hqddot = B*u - C + constraint;
%         end
end
