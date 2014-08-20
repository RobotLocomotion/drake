function [H,C,B,dH,dC,dB] = manipulatorDynamics(obj,q,qd,use_mex)
% note that you can also get C(q,qdot)*qdot + G(q) separately, because 
% C = G when qdot=0

checkDirty(obj);

if (nargin<4) use_mex = true; end

m = obj.featherstone;
B = obj.B;
if (nargout>3)
  dB = zeros(m.NB*obj.num_u,2*m.NB);
end

if length(obj.force)>0
  f_ext = zeros(6,m.NB);
  if (nargout>3)
      df_ext = zeros(6*m.NB,size(q,1)+size(qd,1));
  end
  for i=1:length(obj.force)
    % compute spatial force should return something that is the same length
    % as the number of bodies in the manipulator
    if (obj.force{i}.direct_feedthrough_flag)
      if (nargout>3)
        [force,B_force,dforce,dB_force] = computeSpatialForce(obj.force{i},obj,q,qd);
        dB = dB + dB_force;
      else
        [force,B_force] = computeSpatialForce(obj.force{i},obj,q,qd);
      end
      B = B+B_force;
    else
      if (nargout>3)
          [force,dforce] = computeSpatialForce(obj.force{i},obj,q,qd);
          dforce = reshape(dforce,numel(force),[]);
      else
          force = computeSpatialForce(obj.force{i},obj,q,qd);
      end
    end
    f_ext(:,m.f_ext_map_to) = f_ext(:,m.f_ext_map_to)+force(:,m.f_ext_map_from);
    if (nargout>3)
      for j=1:size(m.f_ext_map_from,2)
        i_from = m.f_ext_map_from(j);
        i_to = m.f_ext_map_to(j);
        df_ext((i_to-1)*size(f_ext,1)+1:i_to*size(f_ext,1),1:size(q,1)+size(qd,1)) = df_ext((i_to-1)*size(f_ext,1)+1:i_to*size(f_ext,1),1:size(q,1)+size(qd,1)) + dforce((i_from-1)*size(force,1)+1:i_from*size(force,1),1:size(q,1)+size(qd,1));
      end
    end
  end
else
  f_ext=[];
  if (nargout>3)
    df_ext=[]; 
  end
end

if (use_mex && obj.mex_model_ptr~=0 && isnumeric(q) && isnumeric(qd))
  f_ext = full(f_ext);  % makes the mex implementation simpler (for now)
  if (nargout>3)
    df_ext = full(df_ext);
    [H,C,dH,dC] = HandCmex(obj.mex_model_ptr,q,qd,f_ext,df_ext);
    dH = [dH, zeros(m.NB*m.NB,m.NB)];
  else
    [H,C] = HandCmex(obj.mex_model_ptr,q,qd,f_ext);
  end
else  
  if (nargout>3)
    % featherstone's HandC with analytic gradients
    a_grav = [0;0;0;obj.gravity];
    
    S = cell(m.NB,1);
    Xup = cell(m.NB,1);
    
    v = cell(m.NB,1);
    avp = cell(m.NB,1);
    
    %Derivatives
    dXupdq = cell(m.NB,1);
    dvdq = cell(m.NB,1);  %dvdq{i}(:,j) is d/dq(j) v{i}
    dvdqd = cell(m.NB,1);
    davpdq = cell(m.NB,1);
    davpdqd = cell(m.NB,1);
    fvp = cell(m.NB,1);
    dfvpdq = cell(m.NB,1);
    dfvpdqd = cell(m.NB,1);
    
    
    for i = 1:m.NB
      n = m.position_num(i);
      
      dvdq{i} = zeros(6,m.NB)*q(1);
      dvdqd{i} = zeros(6,m.NB)*q(1);
      davpdq{i} = zeros(6,m.NB)*q(1);
      davpdqd{i} = zeros(6,m.NB)*q(1);
      dfvpdq{i} = zeros(6,m.NB)*q(1);
      dfvpdqd{i} = zeros(6,m.NB)*q(1);
      
      [ XJ, S{i} ] = jcalc( m.pitch(i), q(n) );
      dXJdq = djcalc(m.pitch(i), q(n));
      
      vJ = S{i}*qd(n);
      dvJdqd = S{i};
      
      Xup{i} = XJ * m.Xtree{i};
      dXupdq{i} = dXJdq * m.Xtree{i};
      
      if m.parent(i) == 0
        v{i} = vJ;
        dvdqd{i}(:,n) = dvJdqd;
        
        avp{i} = Xup{i} * -a_grav;
        davpdq{i}(:,n) = dXupdq{i} * -a_grav;
      else
        j = m.parent(i);

        v{i} = Xup{i}*v{j} + vJ;
        
        dvdq{i} = Xup{i}*dvdq{j};
        dvdq{i}(:,n) = dvdq{i}(:,n) + dXupdq{i}*v{j};
        
        dvdqd{i} = Xup{i}*dvdqd{j};
        dvdqd{i}(:,n) = dvdqd{i}(:,n) + dvJdqd;
        
        avp{i} = Xup{i}*avp{j} + crm(v{i})*vJ;
        
        davpdq{i} = Xup{i}*davpdq{j};
        davpdq{i}(:,n) = davpdq{i}(:,n) + dXupdq{i}*avp{j};
        for k=1:m.NB,
          davpdq{i}(:,k) = davpdq{i}(:,k) + ...
            dcrm(v{i},vJ,dvdq{i}(:,k),zeros(6,1));
        end
        
        dvJdqd_mat = zeros(6,m.NB);
        dvJdqd_mat(:,n) = dvJdqd;
        davpdqd{i} = Xup{i}*davpdqd{j} + dcrm(v{i},vJ,dvdqd{i},dvJdqd_mat);
      end
      fvp{i} = m.I{i}*avp{i} + crf(v{i})*m.I{i}*v{i};
      dfvpdq{i} = m.I{i}*davpdq{i} + dcrf(v{i},m.I{i}*v{i},dvdq{i},m.I{i}*dvdq{i});
      dfvpdqd{i} = m.I{i}*davpdqd{i} + dcrf(v{i},m.I{i}*v{i},dvdqd{i},m.I{i}*dvdqd{i});
      
      if ~isempty(f_ext)
        fvp{i} = fvp{i} - f_ext(:,i);
        dfvpdq{i} = dfvpdq{i} - df_ext((i-1)*size(f_ext,1)+1:i*size(f_ext,1),1:size(q,1));
        dfvpdqd{i} = dfvpdqd{i} - df_ext((i-1)*size(f_ext,1)+1:i*size(f_ext,1),size(q,1)+1:end);
      end
      
    end
    
    C = zeros(m.NB,1)*q(1);
    dC = zeros(m.NB,2*m.NB)*q(1);
    IC = m.I;				% composite inertia calculation
    dIC = cell(m.NB, m.NB);
    dIC = cellfun(@(a) zeros(6), dIC,'UniformOutput',false);
    
    for i = m.NB:-1:1
      n = m.position_num(i);
      C(n,1) = S{i}' * fvp{i};
      dC(n,:) = S{i}'*[dfvpdq{i} dfvpdqd{i}];
      if m.parent(i) ~= 0
        fvp{m.parent(i)} = fvp{m.parent(i)} + Xup{i}'*fvp{i};
        dfvpdq{m.parent(i)} = dfvpdq{m.parent(i)} + Xup{i}'*dfvpdq{i};
        dfvpdq{m.parent(i)}(:,n) = dfvpdq{m.parent(i)}(:,n) + dXupdq{i}'*fvp{i};
        dfvpdqd{m.parent(i)} = dfvpdqd{m.parent(i)} + Xup{i}'*dfvpdqd{i};
        
        IC{m.parent(i)} = IC{m.parent(i)} + Xup{i}'*IC{i}*Xup{i};
        for k=1:m.NB,
          dIC{m.parent(i),k} = dIC{m.parent(i),k} + Xup{i}'*dIC{i,k}*Xup{i};
        end
        dIC{m.parent(i),n} = dIC{m.parent(i),n} + ...
          dXupdq{i}'*IC{i}*Xup{i} + Xup{i}'*IC{i}*dXupdq{i};
      end
    end
    
    % minor adjustment to make TaylorVar work better.
    %H = zeros(m.NB);
    H=zeros(m.NB)*q(1);
    
    %Derivatives wrt q(k)
    dH = zeros(m.NB^2,2*m.NB)*q(1);
    for k = 1:m.NB
      nk = m.position_num(k);
      for i = 1:m.NB
        n = m.position_num(i);
        fh = IC{i} * S{i};
        dfh = dIC{i,nk} * S{i};  %dfh/dqk
        H(n,n) = S{i}' * fh;
        dH(n + (n-1)*m.NB,nk) = S{i}' * dfh;
        j = i;
        while m.parent(j) > 0
          if j==k,
            dfh = Xup{j}' * dfh + dXupdq{j}' * fh;
          else
            dfh = Xup{j}' * dfh;
          end
          fh = Xup{j}' * fh;
          
          j = m.parent(j);
          np = m.position_num(j);
          
          H(n,np) = S{j}' * fh;
          H(np,n) = H(n,np);
          dH(n + (np-1)*m.NB,nk) = S{j}' * dfh;
          dH(np + (n-1)*m.NB,nk) = dH(n + (np-1)*m.NB,nk);
        end
      end
    end
    
    dH = dH(:,1:m.NB)*[eye(m.NB) zeros(m.NB)];
    dC(:,m.NB+1:end) = dC(:,m.NB+1:end) + diag(m.damping);
    
    ind = find(abs(qd)<m.coulomb_window');
    dind = sign(qd(ind))./m.coulomb_window(ind)' .* m.coulomb_friction(ind)';
    fc_drv = zeros(m.NB,1);
    fc_drv(ind) =dind;
    dC(:,m.NB+1:end) = dC(:,m.NB+1:end)+ diag(fc_drv);
  else
    [H,C] = HandC(m,q,qd,f_ext,obj.gravity);
  end
  
  C=C + computeFrictionForce(obj,qd);  
end

end