function q = inverseKin(obj,q0,varargin)
%
% inverseKin(obj,q0,body1,pos1,body2,pos2,...,options)
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
% *You may use zero to indicate the center of mass*.
% @param pos1...posN the desired position of the [0;0;0] point in the corresponding body
%   is pos is 6 elements, then it also constrains the orientation 
%   (note: it does not make sense to specify an orientation for the COM).
%   (note 2: if you specify pos with two columns then it is interpreted as
%   [lower bound, upper bound])
%   (note 3: if you specify pos with a signle column, then elements are set
%   to NAN are treated as "don't care")
% @option q_nom  replaces the cost function with (q-q_nom)'*(q-q_nom).
% @default q_nom = q0
% @option Q  puts a weight on the cost function qtilde'*Q*qtilde
%   

% todo: support orientation
% todo: if mex is enabled, I could move this entire loop (includins snopt)
% down into C.  It would get rid of a lot of overhead.

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

np = length(varargin)/2;
if (np-floor(np)~=0) error('bad input.  must be body,pos,body,pos,...'); end
body_ind=zeros(1,np);
do_rot=logical(zeros(1,np));
Fmin=0; 
Fmax=inf;
for i=1:2:length(varargin)
  % support input as bodies instead of body inds
  if (isa(varargin{i},'RigidBody')) varargin{i} = find(obj.body==varargin{i},1); end
  [rows,cols] = size(varargin{i+1});
  if (rows ~= 3 && rows ~= 6) error('pos must have 3 or 6 rows'); end
  if (varargin{i}==0 && rows ~= 3) error('com pos must have only 3 rows (there is no orientation)'); end
  if (cols==1)
    nind = isnan(varargin{i+1});
    varargin{i+1} = repmat(varargin{i+1},1,2);
    varargin{i+1}(nind,1) = -inf;
    varargin{i+1}(nind,2) = inf;
  elseif cols==2
    if any(isnan(varargin{i+1}(:))), error('nan not allowed when pos is two columns'); end
  else
    error('pos must be one or two cols');
  end
  body_ind((i+1)/2)=varargin{i};
  do_rot((i+1)/2)=(rows==6);
  Fmin=[Fmin;varargin{i+1}(:,1)];
  Fmax=[Fmax;varargin{i+1}(:,2)];
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
        [x,J] = forwardKin(obj,kinsol,body_ind(i),[0;0;0],do_rot(i));
      end
      n=length(x);
      f([j:j+n-1])=x;
      G([j:j+n-1],:)=J;
      j=j+n;
    end
    G=G(:);
  end

end