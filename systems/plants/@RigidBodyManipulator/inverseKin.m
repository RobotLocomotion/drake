function q = inverseKin(obj,q0,varargin)
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
%   * if worldposi is pos is 6 rows, then it also constrains the orientation 
%   * it does not make sense to specify an orientation for the COM.
%   * worldposi can be a structure with worldposi.min and worldposi.max 
%     both 3xmi or 6xmi which sets lower and upper bounds for bodyposi
%   * elements with NAN are treated as don't care (min=-inf, max=inf)
% @option q_nom  replaces the cost function with (q-q_nom)'*(q-q_nom).
%     @default q_nom = q0
% @option Q  puts a weight on the cost function qtilde'*Q*qtilde
%   

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

Fmin=0; 
Fmax=inf;
i=1;n=1;
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
  if (rows ~= 3 && rows ~= 6) error('worldpos must have 3 or 6 rows'); end
  if (body_ind(n)==0 && rows ~= 3) error('com pos must have only 3 rows (there is no orientation)'); end
  if (cols~=mi) error('worldpos must have the same number of elements as bodypos'); end
  sizecheck(maxpos,[rows,mi]);
  
  minpos(isnan(minpos))=-inf;
  maxpos(isnan(maxpos))=inf;
  
  body_pos{n} = bodyposi;
  do_rot(n)=(rows==6);
  Fmin=[Fmin;minpos(:)];
  Fmax=[Fmax;maxpos(:)];
  n=n+1;
end

if 0 %options.use_mex
  [q,info] = inverseKinmex(obj.mex_model_ptr.getData,q0,q_nom,Q,varargin{:});
else
  N = length(varargin);
  nF = length(Fmin);

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

  global SNOPT_USERFUN;
  SNOPT_USERFUN = @ik;
  
  [iGfun,jGvar] = ind2sub([nF,obj.num_q],1:(nF*obj.num_q));
  [q,F,info] = snopt(q0,obj.joint_limit_min,obj.joint_limit_max,Fmin,Fmax,'snoptUserfun',[],[],[],iGfun',jGvar');
end

if (info~=1)
  [str,cat] = snoptInfo(info);
  warning('SNOPT:InfoNotOne',['SNOPT exited w/ info = ',num2str(info),'.\n',cat,': ',str,'\n  Check p19 of Gill06 for more information.']);
end
  
  function [f,G] = ik(q)
    f = zeros(nF,1); G = zeros(nF,obj.num_q);
    f(1) = (q-q_nom)'*Q*(q-q_nom);
    G(1,:) = 2*(q-q_nom)'*Q;
    if (nF<2) return; end
    kinsol = doKinematics(obj,q,false);
    j=2;
    for i=1:length(body_ind)
      if (body_ind(i)==0)
        [x,J] = getCOM(obj,kinsol);
      else
        [x,J] = forwardKin(obj,kinsol,body_ind(i),body_pos{i},do_rot(i));
      end
      n=numel(x);
      f([j:j+n-1])=x(:);
      G([j:j+n-1],:)=J;
      j=j+n;
    end
    G=G(:);
  end

end