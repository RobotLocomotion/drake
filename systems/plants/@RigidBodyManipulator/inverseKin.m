function [q,info] = inverseKin(obj,q0,varargin)
%
% inverseKin(obj,q0,body1,bodypos1,worldpos1,body2,bodypos2,worldpos2...,options)
%
% attempts to solve the optimization problem
%   min_q (q-q0)'*(q-q0)
%   subject to 
%       body1 is at pos1
%       body2 is at pos2
%       ....
%   using q0 as the initial guess
% 
% @param q0 the initial pos
% @param body1...bodyN the index to the body that should be constrained.
%   *You may use zero to indicate the center of mass*.
% @param bodypos1...bodyposN  bodyposi is a 3xmi list of points on 
%   bodyi, or a scalar index of a collision_group, that will be constrained by the solver (in body coordinates)
%   *If you specified the center of mass for bodyi, then you should skip 
%   the bodyposi and go straight to worldposi*
% @param worldpos1...worldposN worldposi is a 3xmi or 6xmi list of
%   position constraints in world coordinates for bodyposi.
%   Note that:
%   * if worldposi is pos is 6 rows, then it also constrains the orientation in Euler angle
%   * if worldposi is pos in 7 rows, then it also constraints the
%   orientation in quaternion.
%   * it does not make sense to specify an orientation for the COM.
%   * worldposi can be a structure with worldposi.min and worldposi.max 
%     both 3xmi or 6xmi which sets lower and upper bounds for bodyposi
%   * elements with NAN are treated as don't care (min=-inf, max=inf)
% @option q_nom  replaces the cost function with (q-q_nom)'*(q-q_nom).
%     @default q_nom = q0
% @option Q  puts a weight on the cost function qtilde'*Q*qtilde
%   

% If we use quaternion representation, we would require add a slack
% variable to represent the desired quaternion, which is in the [quat.min
% quat.max], also we constraint that (quat'*quat_des)^2 = 1,
% so as to make the current quaternion (quat) matches with either the
% quat_des or -quat_des

% todo: support state constraints, even contact constraints, but put an
% option in to enable/disable (presumably those will be harder to push into mex)

if isstruct(varargin{end}) 
  options = varargin{end};
  varargin=varargin(1:end-1);
else
  options = struct();
end

% note: keeping typecheck/sizecheck to a minimum because this might have to
% run inside a dynamical system (so should be fast)

if isfield(options,'q_nom') q_nom = options.q_nom; else q_nom = q0; end
if isfield(options,'Q') Q = options.Q; else Q = eye(obj.num_q); end
if ~isfield(options,'use_mex') options.use_mex = exist('inverseKinmex')==3; end

nq = obj.num_q;
Fmin=0; 
Fmax=inf;
i=1;n=1;
cum_quat_des = 0;
nF = 1;
iGfun = ones(nq,1);
jGvar = (1:nq)';
wmax = obj.joint_limit_max;
wmin = obj.joint_limit_min;
w0 = q0;
while i<=length(varargin)
  % support input as bodies instead of body inds
  if (isa(varargin{i},'RigidBody')) varargin{i} = find(obj.body==varargin{i},1); end
  body_ind(n)=varargin{i}; 
  if (body_ind(n)==0)
    bodyposi = [0;0;0];
    worldposi = varargin{i+1};
    i=i+2;
    mi=1;
  else
    bodyposi = varargin{i+1};
    worldposi = varargin{i+2};
    if ischar(bodyposi) || numel(bodyposi)==1 % then it's the name of a collision group
      b = obj.body(body_ind(n));
      bodyposi = mean(getContactPoints(b,bodyposi),2);
%      bodyposi = getContactPoints(b,bodyposi);
%      worldposi = repmat(worldposi,1,size(bodyposi,2));
    end
    i=i+3;
    [rows,mi] = size(bodyposi);
    if (rows ~=3) error('bodypos must be 3xmi'); end
  end
  if isstruct(worldposi)
    if ~isfield(worldposi,'min') || ~isfield(worldposi,'max')
      error('if worldpos is a struct, it must have fields .min and .max');
    end
    minpos=[worldposi.min];  maxpos=[worldposi.max];
  else
    minpos=worldposi; maxpos=worldposi;
  end
  [rows,cols]=size(minpos);
  if (rows ~= 3 && rows ~= 6 && rows ~=7) error('worldpos must have 3, 6 or 7 rows'); end
  if (body_ind(n)==0 && rows ~= 3) error('com pos must have only 3 rows (there is no orientation)'); end
  if (cols~=mi) error('worldpos must have the same number of elements as bodypos'); end
  sizecheck(maxpos,[rows,mi]);
  
  minpos(isnan(minpos))=-inf;
  maxpos(isnan(maxpos))=inf;
  
  body_pos{n} = bodyposi;
  rotation_type(n)= (rows==6)+2*(rows ==7);
  if(rows == 3 ||rows == 6)
    Fmin=[Fmin;minpos(:)];
    Fmax=[Fmax;maxpos(:)];
    iGfun = [iGfun;nF+repmat((1:rows*cols)',nq,1)];
    jGvar = [jGvar;reshape(bsxfun(@times,ones(rows*cols,1),(1:nq)),[],1)];
    nF = nF+rows*cols;
  elseif(rows == 7)
      % The constraints are that the position matchs, and
      % (quat_des'*quat)^2 = 1. And quat_des'*quat_des = 1
      Fmin = [Fmin;reshape(minpos(1:3,:),[],1);ones(2,1)];
      Fmax = [Fmax;reshape(maxpos(1:3,:),[],1);ones(2,1)];
      iGfun = [iGfun;nF+repmat((1:3*cols)',nq,1);nF+3*cols+ones(nq+4,1);...
          nF+3*cols+1+ones(4,1)];
      jGvar = [jGvar;reshape(bsxfun(@times,1:nq,ones(3*cols,1)),[],1);...
          (1:nq)';...
          nq+cum_quat_des+(1:4)';...
          nq+cum_quat_des+(1:4)'];
      maxpos(4:7,:) = min(maxpos(4:7,:),1+eps); % quaternion cannot be larger than 1
      minpos(4:7,:) = max(minpos(4:7,:),-1-eps);
      quat_max = min([maxpos(4:7,:) [1;1;1;1]],[],2);
      quat_min = max([minpos(4:7,:) -[1;1;1;1]],[],2);
      wmax = [wmax;reshape(quat_max,[],1)];
      wmin = [wmin;reshape(quat_min,[],1)];
      quat_des0 = (quat_max+quat_min)/2;
      quat_des0_norm = sqrt(sum(quat_des0.*quat_des0,1));
      if(quat_des0_norm~=0)
          quat_des0 = quat_des0./quat_des0_norm;
      else
          quat_des0 = [1/2;1/2;1/2;1/2];
      end
      w0 = [w0;quat_des0];
      nF = nF+3*cols+2;
      cum_quat_des = cum_quat_des+4;
  end
  n=n+1;
end

if 0 %options.use_mex
  [q,info] = inverseKinmex(obj.mex_model_ptr.getData,q0,q_nom,Q,varargin{:});
else
  N = length(varargin);
  nF = length(Fmin);
  nG = length(iGfun);

% k=1;  
% for i=1:nF
%   for j=1:obj.num_q
%     iGfun(k)=i;
%     jGvar(k)=j;
%     k=k+1;
%   end
% end
% [q,fmex,Gmex]=inverseKinmex(obj.mex_model_ptr.getData,q0,q_nom,Q,varargin{:});
% Gmex = full(sparse(iGfun,jGvar,Gmex));
% [f,G] = ik(q0);
% G = reshape(G,nF,obj.num_q);
% valuecheck(fmex,f);
% valuecheck(Gmex,G);
% return;
snsetr('Major optimality tolerance',5e-3);
snseti('Major Iterations Limit',300);
  global SNOPT_USERFUN;
  SNOPT_USERFUN = @(w) ik(w,obj,nq,nF,nG,q_nom,Q,body_ind,body_pos,rotation_type);
  
%   [iGfun,jGvar] = ind2sub([nF,obj.num_q],1:(nF*obj.num_q));
  [w_sol,F,info] = snopt(w0,wmin,wmax,Fmin,Fmax,'snoptUserfun',[],[],[],iGfun,jGvar);
end
q = w_sol(1:nq);
if (info~=1)
  [str,cat] = snoptInfo(info);
  warning('SNOPT:InfoNotOne',['SNOPT exited w/ info = ',num2str(info),'.\n',cat,': ',str,'\n  Check p19 of Gill06 for more information.']);
end
  
  
% keyboard
end

function [f,G] = ik(w,obj,nq,nF,nG,q_nom,Q,body_ind,body_pos,rotation_type)
f = zeros(nF,1); G = zeros(nG,1);
q = w(1:nq);
quat_des_all = w(nq+1:end);
cum_quat_des = 0;
f(1) = (q-q_nom)'*Q*(q-q_nom);
G(1:nq) = (2*(q-q_nom)'*Q)';
ng = nq;
if (nF<2) return; end
kinsol = doKinematics(obj,q,false);
nf = 1;
for i=1:length(body_ind)
  if (body_ind(i)==0)
    [x,J] = getCOM(obj,kinsol);
  else
    [x,J] = forwardKin(obj,kinsol,body_ind(i),body_pos{i},rotation_type(i));
  end
  [rows,cols] = size(x);
  n = rows*cols;
  if(rotation_type(i) == 0|| rotation_type(i) == 1)
      f(nf+(1:n))=x(:);
      G(ng+(1:numel(J))) = J(:);
      nf=nf+n;
      ng = ng+numel(J);
  elseif(rotation_type(i) == 2)
      f(nf+(1:3*cols)) = reshape(x(1:3,:),[],1);
      quat = x(4:7,1);
      quat_des = quat_des_all(cum_quat_des+(1:4));
      f(nf+3*cols+1) = sum(quat.*quat_des,1)^2;
      f(nf+3*cols+1+1) = sum(quat_des.*quat_des,1);
      J_rows = bsxfun(@plus,(1:7)',(0:(cols-1))*7);
      pos_rows = reshape(J_rows(1:3,:),[],1);
      dquatdq = J(4:7,:);
      G(ng+(1:3*cols*nq)) = reshape(J(pos_rows,:),[],1);
      G(ng+3*cols*nq+(1:(nq+4))) = (quat'*quat_des)*[2*quat_des'*dquatdq 2*quat']';
      G(ng+3*cols*nq+(nq+4)+(1:4)) = 2*quat_des;
      nf = nf+3*cols+2;
      ng = ng+3*cols*nq+(nq+4)+4;
      cum_quat_des = cum_quat_des+4;
  end
end
end