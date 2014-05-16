function [xA_in_world,xB_in_world,J,dJ_or_Jdot] = ...
  contactPositionsFromCollisionData(obj,kinsol,xA,xB,idxA,idxB,compute_Jdot_instead_of_dJ)

  if nargin < 7
    compute_Jdot_instead_of_dJ = false;
  end
  compute_second_derivative = nargout > 3 && ~compute_Jdot_instead_of_dJ;
  compute_Jdot = nargout > 3 && compute_Jdot_instead_of_dJ;

  idxA = idxA';
  idxB = idxB';

  nq = obj.getNumPositions;  
  nC = length(idxA);
  
  J = zeros(3*nC,nq)*kinsol.q(1);
  xA_in_world = zeros(size(xA));
  xB_in_world = zeros(size(xB));
  if compute_second_derivative,
    dJ = zeros(3*nC,nq*nq)*kinsol.q(1);
  end
  if compute_Jdot,
    Jdot = zeros(3*nC,nq)*kinsol.q(1);
  end
  
%   assert(isequal(idxA,sort(idxA)))
  
  body_inds = unique([idxA(idxA>1);idxB(idxB>1)]);
  
  % Cache the results of a kron and repmat, since this was taking a ton of
  % time
  tmp_vec = repmat([-2;-1;0],nC,1);
  tmp_kron = kron(speye(nC),3*ones(3,1));
  
  
  % loop over the bodies with contact points, instead of number of contacts
  % reduces the calls to forwardKin from n^2 to n
  for i=1:length(body_inds),
    % The contact indices related to this body, on idxA and idxB
    cindA = find(idxA == body_inds(i));
    cindB = find(idxB == body_inds(i));
    
    % Vectorized calculation of Jacobian indices related to this contact
    % for contact i, this is (1:3) + (i-1)*3
    if ~isempty(cindA)
      JindA = tmp_kron(1:3*length(cindA),1:length(cindA))*cindA + tmp_vec(1:3*length(cindA));
    else
      JindA = [];
    end
    if ~isempty(cindB)
      JindB = tmp_kron(1:3*length(cindB),1:length(cindB))*cindB + tmp_vec(1:3*length(cindB));
    else
      JindB = [];
    end
    if compute_second_derivative,
      [x_in_world,J_tmp,dJ_tmp] = obj.forwardKin(kinsol,body_inds(i),[xA(:,cindA) xB(:,cindB)]);
      dJ(JindA,:) = dJ(JindA,:) + dJ_tmp(1:3*length(cindA),:);
      dJ(JindB,:) = dJ(JindB,:) - dJ_tmp(3*length(cindA)+1:end,:);
    else
      [x_in_world,J_tmp] = obj.forwardKin(kinsol,body_inds(i),[xA(:,cindA) xB(:,cindB)]);
      if compute_Jdot
        [x_in_world,J_tmp] = obj.forwardKin(kinsol,body_inds(i),[xA(:,cindA) xB(:,cindB)]);
        Jdot_tmp = forwardJacDot(obj,kinsol,body_inds(i),[xA(:,cindA) xB(:,cindB)]);
        Jdot(JindA,:) = Jdot(JindA,:) + Jdot_tmp(1:3*length(cindA),:);
        Jdot(JindB,:) = Jdot(JindB,:) - Jdot_tmp(3*length(cindA)+1:end,:);
      end
    end
    xA_in_world(:,cindA) = x_in_world(:,1:numel(cindA));
    xB_in_world(:,cindB) = x_in_world(:,1:numel(cindB));
    J(JindA,:) = J(JindA,:) + J_tmp(1:3*length(cindA),:);
    J(JindB,:) = J(JindB,:) - J_tmp(3*length(cindA)+1:end,:);
  end
  if compute_second_derivative
    dJ_or_Jdot = dJ;
  elseif compute_Jdot_instead_of_dJ
    dJ_or_Jdot = Jdot;
  end
  
%   for i=1:nC,
% %     imax = find(idxA == idxA(i),1,'last');
%     I = (1:3) + (i-1)*3;
% %     I = (i-1)*3 + 1: 3*imax;
%     
%     % For each of the two bodies, if it is a real body (i.e. idx != 0),
%     % then add compute the relevent jacobians in joint coordinates
%     if idxA(i) ~= 1,
%       if compute_second_derivative,
%         [~,J(I,:),dJ(I,:)] = obj.forwardKin(kinsol,idxA(i),xA(:,i));
%       else
%         [~,J(I,:)] = obj.forwardKin(kinsol,idxA(i),xA(:,i));
%       end
%     end
%     
%     if idxB(i) ~= 1,
%       if compute_second_derivative,
%         [~,Jtmp,dJtmp] = obj.forwardKin(kinsol,idxB(i),xB(:,i));
%         J(I,:) = J(I,:) - Jtmp;
%         dJ(I,:) = dJ(I,:) - dJtmp;
%       else
%         [~,Jtmp] = obj.forwardKin(kinsol,idxB(i),xB(:,i));
%         J(I,:) = J(I,:) - Jtmp;
%       end
%     end
%   end
