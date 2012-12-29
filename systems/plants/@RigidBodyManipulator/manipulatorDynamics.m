function [H,C,B,dH,dC,dB] = manipulatorDynamics(obj,q,qd)

m = obj.featherstone;

if (nargout>3)
  if (obj.mex_model_ptr && isnumeric(q) && isnumeric(qd))
    [H,C,dH,dC] = HandCmex(obj.mex_model_ptr,q,qd);
  else
    % featherstone's HandC with analytic gradients
    a_grav = [0;0;0;obj.gravity];
    
    S = cell(m.NB,1);
    Xup = cell(m.NB,1);
    
    v = cell(m.NB,1);
    avp = cell(m.NB,1);
    
    %Derivatives
    dXupdq = cell(m.NB,1);
    dvdq = cell(m.NB,1);  %dvdq{i,j} is d/dq(j) v{i}
    dvdqd = cell(m.NB,1);
    davpdq = cell(m.NB,1);
    davpdqd = cell(m.NB,1);
    fvp = cell(m.NB,1);
    dfvpdq = cell(m.NB,1);
    dfvpdqd = cell(m.NB,1);
    
    
    for i = 1:m.NB
      dvdq{i} = zeros(6,m.NB)*q(1);
      dvdqd{i} = zeros(6,m.NB)*q(1);
      davpdq{i} = zeros(6,m.NB)*q(1);
      davpdqd{i} = zeros(6,m.NB)*q(1);
      dfvpdq{i} = zeros(6,m.NB)*q(1);
      dfvpdqd{i} = zeros(6,m.NB)*q(1);
      
      [ XJ, S{i} ] = jcalc( m.pitch(i), q(i) );
      dXJdq = djcalc(m.pitch(i), q(i));
      
      vJ = S{i}*qd(i);
      dvJdqd = S{i};
      
      Xup{i} = XJ * m.Xtree{i};
      dXupdq{i} = dXJdq * m.Xtree{i};
      
      if m.parent(i) == 0
        v{i} = vJ;
        dvdqd{i}(:,i) = dvJdqd;
        
        avp{i} = Xup{i} * -a_grav;
        davpdq{i}(:,i) = dXupdq{i} * -a_grav;
      else
        j = m.parent(i);
        v{i} = Xup{i}*v{j} + vJ;
        
        dvdq{i} = Xup{i}*dvdq{j};
        dvdq{i}(:,i) = dvdq{i}(:,i) + dXupdq{i}*v{j};
        
        dvdqd{i} = Xup{i}*dvdqd{j};
        dvdqd{i}(:,i) = dvdqd{i}(:,i) + dvJdqd;
        
        avp{i} = Xup{i}*avp{j} + crm(v{i})*vJ;
        
        davpdq{i} = Xup{i}*davpdq{j};
        davpdq{i}(:,i) = davpdq{i}(:,i) + dXupdq{i}*avp{j};
        for k=1:m.NB,
          davpdq{i}(:,k) = davpdq{i}(:,k) + ...
            dcrm(v{i},vJ,dvdq{i}(:,k),zeros(6,1));
        end
        
        dvJdqd_mat = zeros(6,m.NB);
        dvJdqd_mat(:,i) = dvJdqd;
        davpdqd{i} = Xup{i}*davpdqd{j} + dcrm(v{i},vJ,dvdqd{i},dvJdqd_mat);
      end
      fvp{i} = m.I{i}*avp{i} + crf(v{i})*m.I{i}*v{i};
      dfvpdq{i} = m.I{i}*davpdq{i} + dcrf(v{i},m.I{i}*v{i},dvdq{i},m.I{i}*dvdq{i});
      dfvpdqd{i} = m.I{i}*davpdqd{i} + dcrf(v{i},m.I{i}*v{i},dvdqd{i},m.I{i}*dvdqd{i});
    end
    
    dC = zeros(m.NB,2*m.NB)*q(1);
    IC = m.I;				% composite inertia calculation
    dIC = cell(m.NB, m.NB);
    dIC = cellfun(@(a) zeros(6), dIC,'UniformOutput',false);
    
    for i = m.NB:-1:1
      C(i,1) = S{i}' * fvp{i};
      dC(i,:) = S{i}'*[dfvpdq{i} dfvpdqd{i}];
      if m.parent(i) ~= 0
        fvp{m.parent(i)} = fvp{m.parent(i)} + Xup{i}'*fvp{i};
        dfvpdq{m.parent(i)} = dfvpdq{m.parent(i)} + Xup{i}'*dfvpdq{i};
        dfvpdq{m.parent(i)}(:,i) = dfvpdq{m.parent(i)}(:,i) + dXupdq{i}'*fvp{i};
        dfvpdqd{m.parent(i)} = dfvpdqd{m.parent(i)} + Xup{i}'*dfvpdqd{i};
        
        IC{m.parent(i)} = IC{m.parent(i)} + Xup{i}'*IC{i}*Xup{i};
        for k=1:m.NB,
          dIC{m.parent(i),k} = dIC{m.parent(i),k} + Xup{i}'*dIC{i,k}*Xup{i};
        end
        dIC{m.parent(i),i} = dIC{m.parent(i),i} + ...
          dXupdq{i}'*IC{i}*Xup{i} + Xup{i}'*IC{i}*dXupdq{i};
      end
    end
    
    % minor adjustment to make TaylorVar work better.
    %H = zeros(m.NB);
    H=zeros(m.NB)*q(1);
    
    %Derivatives wrt q(k)
    dH = zeros(m.NB^2,2*m.NB)*q(1);
    for k = 1:m.NB
      for i = 1:m.NB
        fh = IC{i} * S{i};
        dfh = dIC{i,k} * S{i};  %dfh/dqk
        H(i,i) = S{i}' * fh;
        dH(i + (i-1)*m.NB,k) = S{i}' * dfh;
        j = i;
        while m.parent(j) > 0
          if j==k,
            dfh = Xup{j}' * dfh + dXupdq{k}' * fh;
          else
            dfh = Xup{j}' * dfh;
          end
          fh = Xup{j}' * fh;
          
          j = m.parent(j);
          H(i,j) = S{j}' * fh;
          H(j,i) = H(i,j);
          dH(i + (j-1)*m.NB,k) = S{j}' * dfh;
          dH(j + (i-1)*m.NB,k) = dH(i + (j-1)*m.NB,k);
        end
      end
    end
    
  end
  dH = dH(:,1:m.NB)*[eye(m.NB) zeros(m.NB)];
  C=C+m.damping'.*qd;
  dC(:,m.NB+1:end) = dC(:,m.NB+1:end) + diag(m.damping);
  B = obj.B;
  dB = zeros(m.NB*obj.num_u,2*m.NB);
else
  if (obj.mex_model_ptr && isnumeric(q) && isnumeric(qd))
    [H,C] = HandCmex(obj.mex_model_ptr,q,qd);
  else
    [H,C] = HandC(m,q,qd,{},obj.gravity);
  end
  
  C=C+m.damping'.*qd;
  B = obj.B;
end

end