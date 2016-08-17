classdef CubicPostureError < Constraint
  % approximate the posture as a cubic spline, and penalize the cost
  % sum_i (q(:,i)-q_nom(:,i))'*Q*(q(:,i)-q_nom(:,i))+
  % sum_i qdot(:,i)'*Qv*qdot(:,i) +
  % sum_i qddot(:,i)'*Qa*qddot(:,i)
  % [qdot(:,2);qdot(:,3);...;qdot(:,obj.nT-1)] =
  % velocity_mat*[q(:,1);q(:,2);...;q(:,obj.nT)]+velocity_mat_qd0*qdot(:,1)+velocity_mat_qdf*qdot(:,obj.nT)
  % @param Qv      -- A nq x nq PSD matrix
  % [qddot(:,1);qddot(:,2);...;qddot(:,obj.nT)] = accel_mat*[q(:,1);q(:,2);...;q(:,obj.nT)]+
  % accel_mat_qd0*qdot(:,1)+accel_mat_qdf*qdot(:,obj.nT)
  % @param velocity_mat    --A nq*(obj.nT-2) x nq*obj.nT matrix.
  % @param velocity_mat_qd0  -- A nq*(obj.nT-2) x nq matrix
  % @param velocity_mat_qdf  -- A nq*(obj.nT-2) x nq matrix
  % @param accel_mat       -- A nq*obj.nT x nq*obj.nT matrix
  % @param accel_mat_qd0     -- A nq*obj.nT x nq matrix
  % @param accel_mat_qdf     -- A nq*obj.nT x nq matrix
  properties(SetAccess = protected)
    Q
    q_nom
    Qv
    velocity_mat
    velocity_mat_qd0
    velocity_mat_qdf
    Qa
    accel_mat
    accel_mat_qd0
    accel_mat_qdf
  end

  properties(Access = private)
    dt
    dt_ratio;
    nq
    nT
  end

  methods
    function obj = CubicPostureError(t,Q,q_nom,Qv,Qa)
      t = unique(t(:))';
      obj = obj@Constraint(-inf,inf,size(q_nom,1)*(length(t)+2),1);
      if(~isnumeric(t))
        error('Drake:CubicPostureError:t should be numeric');
      end
      obj.nT = length(t);
      obj.dt = diff(t);
      obj.dt_ratio = obj.dt(1:end-1)./obj.dt(2:end);
      if(obj.nT<=1)
        error('Drake:CubicPostureError: t should have at least two distince elemeobj.nTs');
      end
      obj.nq = size(q_nom,1);
      valuecheck(obj.nT,size(q_nom,2));
      if(~isnumeric(q_nom))
        error('Drake:CubicPostureError:q_nom should be numeric');
      end
      obj.q_nom = q_nom;
      if(~isnumeric(Q) || size(Q,1) ~= obj.nq || size(Q,2) ~= obj.nq || any(eig(Q))<0)
        error('Drake:CubicPostureError:Q should be a nq x nq PSD matrix');
      end
      obj.Q = Q;
      if(~isnumeric(Qv) || size(Qv,1) ~= obj.nq || size(Qv,2) ~= obj.nq || any(eig(Qv))<0)
        error('Drake:CubicPostureError:Qv should be a nq x nq PSD matrix');
      end
      obj.Qv = Qv;
      if(~isnumeric(Qa) || size(Qa,1) ~= obj.nq || size(Qa,2) ~= obj.nq || any(eig(Qa))<0)
        error('Drake:CubicPostureError:Qa should be a nq x nq PSD matrix');
      end
      obj.Qa = Qa;
      velocity_mat1_diag1 = reshape([ones(obj.nq,1) repmat(obj.dt(1:end-1).*(2+2*obj.dt_ratio),obj.nq,1) ones(obj.nq,1)],[],1);
      velocity_mat1_diag2 = reshape([zeros(obj.nq,1) repmat(obj.dt(1:end-1).*obj.dt_ratio,obj.nq,1)],[],1);
      velocity_mat1_diag3 = [reshape(repmat(obj.dt(1:end-1),obj.nq,1),[],1);zeros(obj.nq,1)];
      velocity_mat1 = sparse((1:obj.nq*obj.nT)',(1:obj.nq*obj.nT)',velocity_mat1_diag1)...
          +sparse((1:obj.nq*(obj.nT-1))',obj.nq+(1:obj.nq*(obj.nT-1))',velocity_mat1_diag2,obj.nq*obj.nT,obj.nq*obj.nT)...
          +sparse(obj.nq+(1:obj.nq*(obj.nT-1))',(1:obj.nq*(obj.nT-1))',velocity_mat1_diag3,obj.nq*obj.nT,obj.nq*obj.nT);


      velocity_mat2_diag1 = reshape([zeros(obj.nq,1) bsxfun(@times,3*ones(1,obj.nT-2)-3*obj.dt_ratio.^2,ones(obj.nq,1)) zeros(obj.nq,1)],[],1);
      velocity_mat2_diag2 = reshape([zeros(obj.nq,1) bsxfun(@times,3*obj.dt_ratio.^2,ones(obj.nq,1))],[],1);
      velocity_mat2_diag3 = [-3*ones(obj.nq*(obj.nT-2),1);zeros(obj.nq,1)];
      velocity_mat2 = sparse((1:obj.nq*obj.nT)',(1:obj.nq*obj.nT)',velocity_mat2_diag1)...
          +sparse((1:obj.nq*(obj.nT-1))',obj.nq+(1:obj.nq*(obj.nT-1))',velocity_mat2_diag2,obj.nq*obj.nT,obj.nq*obj.nT)...
          +sparse(obj.nq+(1:obj.nq*(obj.nT-1))',(1:obj.nq*(obj.nT-1))',velocity_mat2_diag3,obj.nq*obj.nT,obj.nq*obj.nT);
      obj.velocity_mat = velocity_mat1\velocity_mat2;
      obj.velocity_mat = obj.velocity_mat(obj.nq+1:end-obj.nq,:);
      obj.velocity_mat_qd0 = -velocity_mat1(obj.nq+1:obj.nq*(obj.nT-1),obj.nq+1:obj.nq*(obj.nT-1))\velocity_mat1(obj.nq+(1:obj.nq*(obj.nT-2)),1:obj.nq);
      obj.velocity_mat_qdf = -velocity_mat1(obj.nq+1:obj.nq*(obj.nT-1),obj.nq+1:obj.nq*(obj.nT-1))\velocity_mat1(obj.nq+(1:obj.nq*(obj.nT-2)),obj.nq*(obj.nT-1)+(1:obj.nq));

      accel_mat1_diag1 = reshape(bsxfun(@times,[-6./(obj.dt.^2) -6/(obj.dt(end)^2)],ones(obj.nq,1)),[],1);
      accel_mat1_diag2 = reshape(bsxfun(@times,6./(obj.dt.^2),ones(obj.nq,1)),[],1);
      accel_mat1_diag3 = 6/(obj.dt(end)^2)*ones(obj.nq,1);
      accel_mat1 = sparse((1:obj.nq*obj.nT)',(1:obj.nq*obj.nT)',accel_mat1_diag1)...
          +sparse((1:obj.nq*(obj.nT-1))',obj.nq+(1:obj.nq*(obj.nT-1))',accel_mat1_diag2,obj.nq*obj.nT,obj.nq*obj.nT)...
          +sparse(obj.nq*(obj.nT-1)+(1:obj.nq)',obj.nq*(obj.nT-2)+(1:obj.nq)',accel_mat1_diag3,obj.nq*obj.nT,obj.nq*obj.nT);
      accel_mat2_diag1 = reshape(bsxfun(@times,[-4./obj.dt 4/obj.dt(end)],ones(obj.nq,1)),[],1);
      accel_mat2_diag2 = reshape(bsxfun(@times,-2./obj.dt,ones(obj.nq,1)),[],1);
      accel_mat2_diag3 = 2/obj.dt(end)*ones(obj.nq,1);
      accel_mat2 = sparse((1:obj.nq*obj.nT)',(1:obj.nq*obj.nT)',accel_mat2_diag1)...
          +sparse((1:obj.nq*(obj.nT-1))',obj.nq+(1:obj.nq*(obj.nT-1))',accel_mat2_diag2,obj.nq*obj.nT,obj.nq*obj.nT)...
          +sparse(obj.nq*(obj.nT-1)+(1:obj.nq)',obj.nq*(obj.nT-2)+(1:obj.nq)',accel_mat2_diag3,obj.nq*obj.nT,obj.nq*obj.nT);
      obj.accel_mat = accel_mat1+accel_mat2(:,obj.nq+1:end-obj.nq)*obj.velocity_mat;
      obj.accel_mat_qd0 = accel_mat2(:,1:obj.nq)+accel_mat2(:,obj.nq+1:obj.nq*(obj.nT-1))*obj.velocity_mat_qd0;
      obj.accel_mat_qdf = accel_mat2(:,end-obj.nq+1:end)+accel_mat2(:,obj.nq+1:obj.nq*(obj.nT-1))*obj.velocity_mat_qdf;
    end

    function [q,qdot,qddot] = cubicSpline(obj,x)
      % return the cubic spline at knot point
      q = x(1:obj.nq*obj.nT);
      qdot0 = x(obj.nq*obj.nT+(1:obj.nq));
      qdotf = x(obj.nq*obj.nT+obj.nq+(1:obj.nq));
      qdot = [qdot0 reshape(obj.velocity_mat*q+obj.velocity_mat_qd0*qdot0+obj.velocity_mat_qdf*qdotf,obj.nq,obj.nT-2) qdotf];
      qddot = reshape(obj.accel_mat*q+obj.accel_mat_qd0*qdot0+obj.accel_mat_qdf*qdotf,obj.nq,obj.nT);
      q = reshape(q,obj.nq,obj.nT);
    end
  end

  methods (Access = protected)
    function [c,dc] = constraintEval(obj,x)
      % @param x    -- A double array of size nq*(nT+2). x =
      % [q(:,1);q(:,2);...;q(:,nT);qdot(:,1);qdot(:,nT)]
      q = x(1:obj.nq*obj.nT);
      qdot0 = x(obj.nq*obj.nT+(1:obj.nq));
      qdotf = x(obj.nq*obj.nT+obj.nq+(1:obj.nq));
      qdot = [qdot0 reshape(obj.velocity_mat*q+obj.velocity_mat_qd0*qdot0+obj.velocity_mat_qdf*qdotf,obj.nq,obj.nT-2) qdotf];
      qddot = reshape(obj.accel_mat*q+obj.accel_mat_qd0*qdot0+obj.accel_mat_qdf*qdotf,obj.nq,obj.nT);
      q = reshape(q,obj.nq,obj.nT);
      qerr = q-obj.q_nom;
      Qqerr = obj.Q*qerr;
      Qqdot = obj.Qv*qdot;
      Qqddot = obj.Qa*qddot;
      c = sum(sum(qerr.*Qqerr))+sum(sum(qdot.*Qqdot))+sum(sum(qddot.*Qqddot));
      if nargout>1
        dc = zeros(1,obj.nq*(obj.nT+2));
        dc(1:obj.nq*obj.nT) = reshape(2*Qqerr,1,[])+reshape(2*Qqdot(:,2:obj.nT-1),1,[])*obj.velocity_mat+reshape(2*Qqddot,1,[])*obj.accel_mat;
        dc(obj.nq*obj.nT+(1:obj.nq)) = reshape(2*Qqdot(:,2:obj.nT-1),1,[])*obj.velocity_mat_qd0+reshape(2*Qqdot(:,1),1,[])+reshape(2*Qqddot,1,[])*obj.accel_mat_qd0;
        dc(obj.nq*obj.nT+obj.nq+(1:obj.nq)) = reshape(2*Qqdot(:,2:obj.nT-1),1,[])*obj.velocity_mat_qdf+reshape(2*Qqdot(:,obj.nT),1,[])+reshape(2*Qqddot,1,[])*obj.accel_mat_qdf;
      end
    end
  end
end
