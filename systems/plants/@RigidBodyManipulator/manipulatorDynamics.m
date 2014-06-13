function [H,C,B,dH,dC,dB] = manipulatorDynamics(obj,q,v,use_mex)
% note that you can also get C(q,qdot)*qdot + G(q) separately, because 
% C = G when qdot=0

checkDirty(obj);
compute_gradients = nargout > 3;

if (nargin<4) use_mex = true; end

m = obj.featherstone;
B = obj.B;
NB = obj.getNumBodies() - 1;
if compute_gradients
  dB = zeros(NB*obj.num_u,2*NB);
end

if length(obj.force)>0
  f_ext = zeros(6,NB);
  if compute_gradients
      df_ext = zeros(6*NB,size(q,1)+size(v,1));
  end
  for i=1:length(obj.force)
    % compute spatial force should return something that is the same length
    % as the number of bodies in the manipulator
    if (obj.force{i}.direct_feedthrough_flag)
      if compute_gradients
        [force,B_force,dforce,dB_force] = computeSpatialForce(obj.force{i},obj,q,v);
        dB = dB + dB_force;
      else
        [force,B_force] = computeSpatialForce(obj.force{i},obj,q,v);
      end
      B = B+B_force;
    else
      if compute_gradients
          [force,dforce] = computeSpatialForce(obj.force{i},obj,q,v);
          dforce = reshape(dforce,numel(force),[]);
      else
          force = computeSpatialForce(obj.force{i},obj,q,v);
      end
    end
    f_ext(:,m.f_ext_map_to) = f_ext(:,m.f_ext_map_to)+force(:,m.f_ext_map_from);
    if compute_gradients
      for j=1:size(m.f_ext_map_from,2)
        i_from = m.f_ext_map_from(j);
        i_to = m.f_ext_map_to(j);
        df_ext((i_to-1)*size(f_ext,1)+1:i_to*size(f_ext,1),1:size(q,1)+size(v,1)) = df_ext((i_to-1)*size(f_ext,1)+1:i_to*size(f_ext,1),1:size(q,1)+size(v,1)) + dforce((i_from-1)*size(force,1)+1:i_from*size(force,1),1:size(q,1)+size(v,1));
      end
    end
  end
else
  f_ext=[];
  if compute_gradients
    df_ext=[]; 
  end
end

if (use_mex && obj.mex_model_ptr~=0 && isnumeric(q) && isnumeric(v))
  f_ext = full(f_ext);  % makes the mex implementation simpler (for now)
  if compute_gradients
    df_ext = full(df_ext);
    [H,C,dH,dC] = HandCmex(obj.mex_model_ptr,q,v,f_ext,df_ext);
    dH = [dH, zeros(NB*NB,NB)];
  else
    [H,C] = HandCmex(obj.mex_model_ptr,q,v,f_ext);
  end
else  
  if compute_gradients
    [H,C,dH,dC] = HandC(obj,q,v,f_ext,obj.gravity);
  else
    [H,C] = HandC(obj,q,v,f_ext,obj.gravity);
  end
  
end

end