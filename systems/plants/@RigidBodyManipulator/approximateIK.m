function q = approximateIK(obj,q0,varargin)
%
% approximateIK(obj,q0,body1,bodypos1,worldpos1,body2,bodypos2,worldpos2...,options)
%
% attempts to solve the optimization problem
%   min_q (q-q0)'*(q-q0)
%   subject to 
%       body1 is at pos1
%       body2 is at pos2
%       ....
%   using q0 as the initial guess
%   where constraints are linear: 
%    e.g., com_desired - com_0 = J*(q-q0);
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
%   * elements with NAN are treated as don't care (min=-inf, max=inf)
% @option q_nom  replaces the cost function with (q-q_nom)'*(q-q_nom).
%     @default q_nom = q0
% @option Q  puts a weight on the cost function qtilde'*Q*qtilde
%   

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

kinsol = doKinematics(obj,q0,false);

i=1;n=1;
while i<=length(varargin)
  % support input as bodies instead of body inds
  if (isa(varargin{i},'RigidBody')) varargin{i} = find(obj.body==varargin{i},1); end
  body_ind=varargin{i}; 
  
  if (body_ind==0)
    body_pos = [0;0;0];
    world_pos = varargin{i+1};
    i=i+2;
    rows=3;
  else
    body_pos = varargin{i+1};
    world_pos = varargin{i+2};
    if ischar(body_pos) || numel(body_pos)==1 % then it's the name of a collision group
      b = obj.body(body_ind(n));
      body_pos = mean(getContactPoints(b,body_pos),2);
    end
    i=i+3;
    [rows,~] = size(body_pos);
    if (rows ~=3) error('bodypos must be 3xmi'); end
    [rows,~] = size(world_pos);
  end
  
  if (rows ~= 3 && rows ~= 6) error('worldpos must have 3 or 6 rows'); end
  if (body_ind==0 && rows ~= 3) error('com pos must have only 3 rows (there is no orientation)'); end
  
  if (body_ind==0)
    [x,J] = getCOM(obj,kinsol);
  else
    [x,J] = forwardKin(obj,kinsol,body_ind,body_pos,(rows==6));
  end
  
  % build equality constraints
  idx = ~isnan(world_pos);
  A{n} = J(idx,:);
  b{n} = world_pos(idx) - x(idx) + J(idx,:)*q0;
  
  n=n+1;
end

Aeq = sparse(vertcat(A{:}));
beq = vertcat(b{:});


f = -2*q_nom'*Q;

% NOTE: solving analytically ignoring joint limits doesn't work very well.
%A = [Q Aeq'; Aeq zeros(size(Aeq,1))];
%b = [-f'; beq];
%y = A\b;
%q = y(1:obj.num_q);

% solve QP
model.Q = sparse(Q);
model.obj = f;
model.A = Aeq;
model.rhs = beq;
model.sense = repmat('=',length(beq),1);
model.lb = obj.joint_limit_min;
model.ub = obj.joint_limit_max;

params.outputflag = 0; % not verbose
params.method = 2; % -1=automatic, 0=primal simplex, 1=dual simplex, 2=barrier

result = gurobi(model,params);
q = result.x;  

end