function [q,info] = approximateIK(obj,q0,varargin)
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
%   * worldposi can be a structure with worldposi.min and worldposi.max 
%     both 3xmi or 6xmi which sets lower and upper bounds for bodyposi
% @option q_nom  replaces the cost function with (q-q_nom)'*(q-q_nom).
%     @default q_nom = q0
% @option Q  puts a weight on the cost function qtilde'*Q*qtilde
%   
% @result info is 0 on success, 1 on failure

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
if ~isfield(options,'use_mex') options.use_mex = true; end

i=1;
while i<=length(varargin)
  if (isa(varargin{i},'RigidBody')) varargin{i} = find(obj.body==varargin{i},1); end
  body_ind=varargin{i}; 
  if (body_ind==0)
    i = i+2;
  else
    i = i+3;
  end
end

if options.use_mex
% Frank's fast QP (mex'd by Mike)
  %[q,info] = approximateIKEIQPmex(obj.getMexModelPtr.getData,q0,q_nom,Q,varargin{:});

% Mex'd gurobi version  
  %if info
    %warning('approximateIKEIQP failed.  trying gurobi version.');
    [q,info] = approximateIKmex(obj.getMexModelPtr.getData,q0,q_nom,Q,varargin{:});
    info = (info~=2);  % gurobi "OPTIMAL" http://www.gurobi.com/documentation/5.5/reference-manual/node896#sec:StatusCodes
  %end
  return;
end

kinsol = obj.doKinematics(q0);

A={};b={};Ai={};bi={};
i=1;neq=1;nin=1;
while i<=length(varargin)
  % support input as bodies instead of body inds
  if (isa(varargin{i},'RigidBody')) varargin{i} = find(obj.body==varargin{i},1); end
  body_ind=varargin{i}; 
  
  if (body_ind==0)
    body_pos = [0;0;0];
    world_pos = varargin{i+1};
    i=i+2;
  else
    body_pos = varargin{i+1};
    world_pos = varargin{i+2};
    if ischar(body_pos) || numel(body_pos)==1 % then it's the name of a collision group
      b = obj.body(body_ind);
      body_pos = mean(getContactPoints(b,body_pos),2);
    end
    i=i+3;
    [rows,~] = size(body_pos);
    if (rows ~=3) error('bodypos must be 3xmi'); end
  end
  
  if isstruct(world_pos)
    if ~isfield(world_pos,'min') || ~isfield(world_pos,'max')
      error('if world_pos is a struct, it must have fields .min and .max');
    end
    minpos=[world_pos.min];  maxpos=[world_pos.max];
    
    rows = size(minpos,1);
    maxrows = size(maxpos,1);
    if (rows ~= 3 && rows ~= 6) error('world_pos.min must have 3 or 6 rows'); end
    if (maxrows ~= rows) error('world_pos.max must have the same number of rows as world_pos.min'); end
  else
    minpos=[]; maxpos=[];
    [rows,~] = size(world_pos);
    if (rows ~= 3 && rows ~= 6 && rows ~= 7) error('worldpos must have 3, 6 or 7 rows'); end
  end
  if (body_ind==0 && rows ~= 3) error('com pos must have only 3 rows (there is no orientation)'); end
  
  if (body_ind==0)
    [x,J] = getCOM(obj,kinsol);
  else
    [x,J] = forwardKin(obj,kinsol,body_ind,body_pos,(rows==6));
    if rows==6 && isempty(minpos) && isempty(maxpos) 
      % make sure desired/current angles are unwrapped
      delta = angleDiff(x(4:6),world_pos(4:6));
      world_pos(4:6) = x(4:6)+delta;
    end
      % TODO: implement angle wrap protection for inequality constraints (or convert to quaternions)
%     if rows==6 && ~isempty(minpos)
%       delta = angleDiff(x(4:6),minpos(4:6));
%       minpos(4:6) = x(4:6)+delta;
%       delta = angleDiff(x(4:6),maxpos(4:6));
%       maxpos(4:6) = x(4:6)+delta;
%       swapidx = minpos>maxpos;
%       tmp = minpos;
%       minpos(swapidx) = maxpos(swapidx);
%       maxpos(swapidx) = tmp(swapidx);
%     end
  end
  
  if ~isempty(maxpos) 
    % add inequality constraint
    idx = ~isnan(maxpos);
    Ai{nin} = J(idx,:);
    bi{nin} = maxpos(idx) - x(idx) + J(idx,:)*q0;
    nin=nin+1;
  end
    
  if ~isempty(minpos) 
    % add inequality constraint
    idx = ~isnan(minpos);
    Ai{nin} = -J(idx,:);
    bi{nin} = -(minpos(idx) - x(idx) + J(idx,:)*q0);
    nin=nin+1;
  end
    
  if isempty(minpos) && isempty(maxpos) 
    % add equality constraint
    idx = ~isnan(world_pos);
    A{neq} = J(idx,:);
    b{neq} = world_pos(idx) - x(idx) + J(idx,:)*q0;
    neq=neq+1;
  end
    
end

Aeq = sparse(vertcat(A{:}));
beq = vertcat(b{:});

Ain = sparse(vertcat(Ai{:}));
bin = vertcat(bi{:});

f = -2*q_nom'*Q;

% NOTE: solving analytically ignoring joint limits doesn't work very well.
%A = [Q Aeq'; Aeq zeros(size(Aeq,1))];
%b = [-f'; beq];
%y = A\b;
%q = y(1:obj.num_q);

% solve QP
model.Q = sparse(Q);
model.obj = f;
model.A = [Aeq; Ain];
model.rhs = [beq; bin];
model.sense = [repmat('=',length(beq),1);repmat('<',length(bin),1)]; % using repmat is inefficient 

if isfield(options,'jointLimitMin')
  model.lb = options.jointLimitMin;
else
  model.lb = obj.joint_limit_min;
end

if isfield(options,'jointLimitMax')
  model.ub = options.jointLimitMax;
else
  model.ub = obj.joint_limit_max;
end

params.outputflag = 0; % not verbose
params.method = 2; % -1=automatic, 0=primal simplex, 1=dual simplex, 2=barrier
params.bariterlimit = 20; % iteration limit
params.barhomogeneous = 0; % 0 off, 1 on
params.barconvtol = 1e-4;

result = gurobi(model,params);
info = ~strcmp(result.status,'OPTIMAL');
if (info)
  q = q0;
else
  q = result.x;
end

end
