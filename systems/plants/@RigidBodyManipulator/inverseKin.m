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
%   * elements with NAN are treated as don't care (min=-inf, max=inf)
%   * worldposi can be a structure with worldposi.min and worldposi.max 
%     both 3xmi or 6xmi which sets lower and upper bounds for bodyposi
%     - It can also contain the following optional fields
%       * contact_affs - [n_aff x 1] cell array, where n_aff is the number of
%         affordances with respect to which bodyi is constrained
%       * contact_state - [n_aff x 1] cell array of [1 x mi] arrays, where each
%         element of the arrays is one of the following: 
%         - ActionKinematicConstraint.UNDEFINED_CONTACT
%         - ActionKinematicConstraint.NOT_IN_CONTACT
%         - ActionKinematicConstraint.MAKE_CONTACT
%         - ActionKinematicConstraint.BREAK_CONTACT
%         - ActionKinematicConstraint.STATIC_PLANAR_CONTACT
%         - ActionKinematicConstraint.STATIC_GRIP_CONTACT
%       * contact_dist - [n_aff x 1] cell array of structures, where the j-th
%         structure has the following fields
%         - min - [1 x mi] array giving the minimum allowable distance from
%           each point in bodyposi to the affordance specified by
%           worldposi.contact_affs{j}
%         - max - [1 x mi] array giving the maximum allowable distance from
%           each point in bodyposi to the affordance specified by
%           worldposi.contact_affs{j}
%    *Lastly, worldposi can be a general purpose struct--this currently
%     only implements the gaze constraint type, with the following
%     fields used:
%       * type (must be 'gaze')
%       * gaze_orientation (3x1 (rpy) or 4x1(quaternion))
%         * __XOR__ gaze_target (3x1 (xyz) position in space to look at)
%         * __XOR__ gaze_dir (3x1 (xyz) unit vector direction to look down)
%       * gaze_axis (3x1 unit vector)
%       * (gaze_threshold))  optional, default INF
%            determine the max angular rotation about the gaze axis
%       * (gaze_conethreshold) optional, defualt to 0
%          allows the gaze_axis to be within a cone of the specified angle
%          from the nominal
%        
% @option q_nom  replaces the cost function with (q-q_nom)'*(q-q_nom).
%     @default q_nom = q0
% @option Q  puts a weight on the cost function qtilde'*Q*qtilde
% @options quastiStaticFlag adds the constraint that the CoM is within the
% support polygon
% @option shrinkFactor is the factor to shrink the size of the support
% polygon to get an conservative inner approximation

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
if(~isfield(options,'jointLimitMin')) options.jointLimitMin = obj.joint_limit_min; end
if(~isfield(options,'jointLimitMax')) options.jointLimitMax = obj.joint_limit_max; end

joint_limit_min = options.jointLimitMin; 
joint_limit_max = options.jointLimitMax; 

%options.use_mex = false;
if(options.use_mex)
	mex_varargin = {};
end
if(isfield(options,'quasiStaticFlag'))
    quasiStaticFlag = options.quasiStaticFlag;
else
    quasiStaticFlag = false;
end
if(isfield(options,'shrinkFactor'))
    shrinkFactor = options.shrinkFactor;
else
    shrinkFactor = 0.95;
end
nq = obj.num_q;
  sizecheck(q0,[nq,1]);
  sizecheck(q_nom,[nq,1]);
  sizecheck(Q,[nq,nq]);
  sizecheck(joint_limit_max,[nq,1]);
  sizecheck(joint_limit_min,[nq,1]);
Fmin=0; 
Fmax=inf;
i=1;n=1;
cum_quat_des = 0;
nF = 1;
iGfun = ones(nq,1);
jGvar = (1:nq)';
iAfun = [];
jAvar = [];
A = [];
wmax = joint_limit_max;
wmin = joint_limit_min;
w0 = q0;
total_num_support_vert = 0;
support_polygon_flag = {};
body_num_support_vert = [];
gaze_con = {};

while i<=length(varargin)
  % support input as bodies instead of body inds
  if (isa(varargin{i},'RigidBody')) varargin{i} = find(obj.body==varargin{i},1); end
  body_ind(n)=varargin{i};
  if (body_ind(n)==0)
    bodyposi = [0;0;0];
    worldposi = varargin{i+1};
    support_polygon_flag{n} = false;
    body_num_support_vert(n) = 0;
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
  end
  if isstruct(worldposi) && isfield(worldposi,'type')
    if options.use_mex
      error('Gaze constraints and mex not currently supported.  Please disable use_mex');
    end
    
    % this case added by mposa for gaze constraints
    if strcmp(worldposi.type,'gaze')
      if isfield(worldposi,'gaze_orientation') + isfield(worldposi,'gaze_target') + isfield(worldposi,'gaze_dir') ~= 1
        error('Must constrain exactly one of gaze orientation, target, or direction')
      end
      
      if ~isfield(worldposi,'gaze_axis')
        error('Must supply a gaze axis')
      end
      
      gaze_i = struct();
      gaze_i.axis = worldposi.gaze_axis;
      gaze_i.axis = gaze_i.axis/norm(gaze_i.axis);
      sizecheck(gaze_i.axis,3);
      
      if isfield(worldposi,'gaze_orientation')
        gaze_i.type = 1;  %for orientation
        if length(worldposi.gaze_orientation) == 3,
          gaze_i.orientation = rpy2quat(worldposi.gaze_orientation);
        else
          gaze_i.orientation = worldposi.gaze_orientation;
          gaze_i.orientation = gaze_i.orientation/norm(gaze_i.orientation);
        end
        sizecheck(gaze_i.orientation,4);
      elseif isfield(worldposi,'gaze_target')
        gaze_i.type = 2;  %for target
        gaze_i.target = worldposi.gaze_target;
        sizecheck(gaze_i.target,3);
      elseif isfield(worldposi, 'gaze_dir')
        gaze_i.type = 1;
        %Construct a quaternion from the given direction
        dir = worldposi.gaze_dir;
        dir = dir/norm(dir);
        if abs(dir'*gaze_i.axis - 1) < 1e-6
          gaze_i.orientation = [1;0;0;0];
        elseif abs(dir'*gaze_i.axis + 1) < 1e-6
          %then they're pointed in opposite directions
          % get a random vector orthogonal to our axis
          gaze_i.orientation = [1;0;0;0];
          rand_vec = randn(3,1);
          rand_vec = rand_vec - gaze_i.axis*(gaze_i.axis'*rand_vec);
          rand_vec = rand_vec/norm(rand_vec);
          gaze_i.orientation = axis2quat([rand_vec;pi]);
        else
          quat_axis = cross(gaze_i.axis,dir);
          quat_ang = acos(dir'*gaze_i.axis);          

          gaze_i.orientation = axis2quat([quat_axis;quat_ang]);
        end
      else
        error('Unknown gaze constraint.  This error should be impossible')
      end
      
      % setup constraint here
      if isfield(worldposi,'gaze_conethreshold')
        gaze_bound = cos(worldposi.gaze_conethreshold) - 1;
      else
        gaze_bound = 0;
      end
      
      Fmin = [Fmin;gaze_bound];
      Fmax = [Fmax;0];
      iGfun = [iGfun;repmat(nF+1,nq,1)];
      jGvar = [jGvar;(1:nq)'];
      nF = nF + 1;
      
      if isfield(worldposi,'gaze_threshold')
				if ~isfield(worldposi,'gaze_orientation')
					error('Cannot supply a gaze_threshold if the orientation was not fully specified (as gaze_orientation).')
				end
        gaze_i.check_angle = 1;
        %Add in an angle constraint here
        Fmin = [Fmin; cos(worldposi.gaze_threshold/2)];
        Fmax = [Fmax;inf];
        iGfun = [iGfun;repmat(nF+1,nq,1)];
        jGvar = [jGvar;(1:nq)'];
        nF = nF + 1; 
      else
        gaze_i.check_angle = 0;
      end
      
      gaze_con{n} = gaze_i;
      
    else
      error(strcat('Unknown worldpos.type=',worldposi.type));
    end
  else  
    [rows,mi] = size(bodyposi);
    if (rows ~=3) error('bodypos must be 3xmi'); end
    
    if isstruct(worldposi)
      if ~isfield(worldposi,'min') || ~isfield(worldposi,'max')
        error('if worldpos is a struct, it must have fields .min and .max');
      end
      minpos=[worldposi.min];  maxpos=[worldposi.max];

      % The remaining fields in worldposi are optional
      if isfield(worldposi,'contact_state')
          if iscell(worldposi.contact_state)
              contact_state{n} = worldposi.contact_state;
          else
              contact_state{n} = {worldposi.contact_state};
          end
      elseif(abs(minpos(3))<eps && abs(maxpos(3)<eps))
        contact_state{n} = {ActionKinematicConstraint.STATIC_PLANAR_CONTACT ...
                              *ones(1,size(bodyposi,2))};
      else
        contact_state{n} = {ActionKinematicConstraint.UNDEFINED_CONTACT ...
                              *ones(1,size(bodyposi,2))};
      end

      if isfield(worldposi,'contact_affs')
        contact_affs{n} = worldposi.contact_affs;
      else
        contact_affs{n} = {ContactAffordance()};
      end

      if isfield(worldposi,'contact_dist')
        contact_dist{n} = worldposi.contact_dist;
      else
        contact_dist{n} = {struct('max',inf,'min',0)};
      end
    else
      minpos=worldposi; maxpos=worldposi;
      if(abs(minpos(3))<eps)
        contact_state{n} = {ActionKinematicConstraint.STATIC_PLANAR_CONTACT ...
          *ones(1,size(bodyposi,2))};
      else
        contact_state{n} = {ActionKinematicConstraint.UNDEFINED_CONTACT ...
          *ones(1,size(bodyposi,2))};
      end
      contact_affs{n} = {ContactAffordance()};
      contact_dist{n} = {struct('max',inf,'min',0)};
    end
    if(quasiStaticFlag)
      support_polygon_flag{n} = any(cell2mat(contact_state{n}') == ActionKinematicConstraint.STATIC_PLANAR_CONTACT,1)|...
                                any(cell2mat(contact_state{n}') == ActionKinematicConstraint.STATIC_GRIP_CONTACT,1)|...
                                (any(cell2mat(contact_state{n}') == ActionKinematicConstraint.MAKE_CONTACT,1)&...
                                 any(cell2mat(contact_state{n}') == ActionKinematicConstraint.BREAK_CONTACT,1));
      body_num_support_vert(n) = sum(support_polygon_flag{n});
      total_num_support_vert  = total_num_support_vert+body_num_support_vert(n);
    end
    [rows,cols]=size(minpos);

    if (rows == 6)
      % convert RPY to quaternions internally
      maxpos(7,:) = 0;
      minpos(7,:) = 0;
      for j = 1:cols
        minpos(4:7,j) = rpy2quat(minpos(4:6,j));
        maxpos(4:7,j) = rpy2quat(maxpos(4:6,j));
      end
      rows = 7;
    end

    if (rows ~= 3 && rows ~= 6 && rows ~=7) error('worldpos must have 3, 6 or 7 rows'); end
    if (body_ind(n)==0 && rows ~= 3) error('com pos must have only 3 rows (there is no orientation)'); end
    if (cols~=mi) error('worldpos must have the same number of elements as bodypos'); end
    sizecheck(maxpos,[rows,mi]);

    minpos(isnan(minpos))=-inf;
    maxpos(isnan(maxpos))=inf;

    if(all(all(isinf(minpos)))&&all(all(isinf(maxpos))))
        poscon_type(n) = -1; % No constraint on the worldpos
      else
        poscon_type(n) = 0*(rows==3)+(rows==6)+2*(rows==7);
    end
    body_pos{n} = bodyposi;
      if(poscon_type(n) == 0 ||poscon_type(n) == 1)
      Fmin=[Fmin;minpos(:)];
      Fmax=[Fmax;maxpos(:)];
      iGfun = [iGfun;nF+repmat((1:rows*cols)',nq,1)];
      jGvar = [jGvar;reshape(bsxfun(@times,ones(rows*cols,1),(1:nq)),[],1)];
      nF = nF+rows*cols;
      elseif(poscon_type(n) == 2)
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

      % Currently deal with the collision avoidance only, will handle affordance contact
      % later
      if(~isempty(contact_state{n}))
        if(contact_state{n}{1} == ActionKinematicConstraint.COLLISION_AVOIDANCE)
          num_rigid_bodies = length(contact_affs{n});
          Fmin = [Fmin; ones(num_rigid_bodies,1)];
          Fmax = [Fmax; ones(num_rigid_bodies,1)];
          iGfun = [iGfun;nF+reshape(bsxfun(@times,(1:num_rigid_bodies)',ones(1,nq)),[],1)];
          jGvar = [jGvar;reshape(bsxfun(@times,ones(num_rigid_bodies,1),(1:nq)),[],1)];
          nF = nF+num_rigid_bodies;
        end
      end
  end
  if(options.use_mex)
	  mex_varargin = [mex_varargin,{[bodyposi;ones(1,size(bodyposi,2))]}];
  end
  n=n+1;
end

gaze_con{n} = []; %total hack to make it empty

if(quasiStaticFlag)
    if(total_num_support_vert == 0)
        error('inverseKin:NoContacts','No contact points')
    end
    wmax = [wmax;ones(total_num_support_vert,1)];
    wmin = [wmin;zeros(total_num_support_vert,1)];
    iGfun = [iGfun;nF+reshape(bsxfun(@times,[1;2],ones(1,nq+total_num_support_vert)),[],1)];
    jGvar = [jGvar;reshape(bsxfun(@times,[1;1],[1:nq nq+cum_quat_des+(1:total_num_support_vert)]),[],1)];
    iAfun = [iAfun;nF+3*ones(total_num_support_vert,1)];
    jAvar = [jAvar;nq+cum_quat_des+(1:total_num_support_vert)'];
    A = [A; ones(total_num_support_vert,1)];
    w0 = [w0;1/total_num_support_vert*ones(total_num_support_vert,1)];
    Fmin = [Fmin;0;0;1];
    Fmax = [Fmax;0;0;1];
    nF = nF+3;
end

  N = length(varargin);
  nF = length(Fmin);
  nG = length(iGfun);
if options.use_mex
  Fmin_nan = isnan(Fmin);
  Fmin(Fmin_nan) = -1e20*ones(sum(Fmin_nan),1);
  Fmin_inf = isinf(Fmin);
  Fmin(Fmin_inf) = -1e20*ones(sum(Fmin_inf),1);
  Fmax_nan = isnan(Fmax);
  Fmax(Fmax_nan) = 1e20*ones(sum(Fmax_nan),1);
  Fmax_inf = isinf(Fmax);
  Fmax(Fmax_inf) = 1e20*ones(sum(Fmax_inf),1);
  wmin_nan = isnan(wmin);
  wmin(wmin_nan) = -1e20*ones(sum(wmin_nan),1);
  wmin_inf = isinf(wmin);
  wmin(wmin_inf) = -1e20*ones(sum(wmin_inf),1);
  wmax_nan = isnan(wmax);
  wmax(wmax_nan) = 1e20*ones(sum(wmax_nan),1);
  wmax_inf = isinf(wmax);
  wmax(wmax_inf) = 1e20*ones(sum(wmax_inf),1);
    [w_sol,info] = inverseKinmex(obj.mex_model_ptr.getData,w0,q_nom,Q,Fmax,Fmin,wmax,wmin,iGfun,jGvar,body_ind,poscon_type,mex_varargin{:});
    %   [w_sol,f,G] = inverseKinmex(obj.mex_model_ptr.getData,w0,q_nom,Q,Fmax,Fmin,wmax,wmin,iGfun,jGvar,body_ind,poscon_type,mex_varargin{:});
% keyboard
else

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
% snprint('snopt.out')
snsetr('Major optimality tolerance',5e-3);
snseti('Major Iterations Limit',300);
% snseti('Verify Level',3);

  global SNOPT_USERFUN;
    SNOPT_USERFUN = @(w) ik(w,obj,nq,nF,nG,q_nom,Q,body_ind,body_pos,poscon_type,...
      cum_quat_des,quasiStaticFlag,total_num_support_vert,body_num_support_vert,support_polygon_flag,shrinkFactor,contact_state,contact_affs,contact_dist,gaze_con);
  
%   [iGfun,jGvar] = ind2sub([nF,obj.num_q],1:(nF*obj.num_q));
  [w_sol,F,info] = snopt(w0,wmin,wmax,Fmin,Fmax,'snoptUserfun',0,1,A,iAfun,jAvar,iGfun,jGvar);
end
q = w_sol(1:nq);
if (info~=1)
  [str,cat] = snoptInfo(info);
  warning('SNOPT:InfoNotOne',['SNOPT exited w/ info = ',num2str(info),'.\n',cat,': ',str,'\n  Check p19 of Gill06 for more information.']);
end

if(info == 13)
    ub_err = F-Fmax;
    ub_err = ub_err(~isinf(ub_err));
    max_ub_error = max(ub_err);
    max_ub_error = max_ub_error*(max_ub_error>0);
    lb_err = Fmin-F;
    lb_err = lb_err(~isinf(lb_err));
    max_lb_error = max(lb_err);
    max_lb_error = max_lb_error*(max_lb_error<0);
    if(max_ub_error+max_lb_error>1e-4)
        info = 13;
    else
        info = 4;
    end
end  
  
% keyboard
end

function [f,G] = ik(w,obj,nq,nF,nG,q_nom,Q,body_ind,body_pos,poscon_type,cum_quat_des,...
    quasiStaticFlag,total_num_support_vert,body_num_support_vert,support_polygon_flag,shrinkFactor,...
    contact_state,contact_affs,contact_dist,gaze_con)
support_vert_pos = zeros(2,total_num_support_vert);
dsupport_vert_pos = zeros(2*total_num_support_vert,nq);
f = zeros(nF,1); G = zeros(nG,1);
q = w(1:nq);
quat_des_all = w(nq+(1:cum_quat_des));
if(quasiStaticFlag)
    support_vert_weight = w(nq+cum_quat_des+(1:total_num_support_vert));
end
cum_quat_des = 0;
f(1) = (q-q_nom)'*Q*(q-q_nom);
G(1:nq) = (2*(q-q_nom)'*Q)';
ng = nq;
if (nF<2) return; end
kinsol = doKinematics(obj,q,false);
nf = 1;
support_vert_count = 0;
for i=1:length(body_ind)
  if isempty(gaze_con{i})
    if(quasiStaticFlag)
      [com,dcom] = getCOM(obj,kinsol);
    end
    if (body_ind(i)==0)
      if(quasiStaticFlag)
        x = com;
        J = dcom;
      else
        [x,J] = getCOM(obj,kinsol);
      end
    else
      [x,J] = forwardKin(obj,kinsol,body_ind(i),body_pos{i},poscon_type(i));
      [rows,cols] = size(x);
      if(quasiStaticFlag)
        support_vert_pos(:,support_vert_count+(1:body_num_support_vert(i))) = x(1:2,support_polygon_flag{i});
        J_support_polygon_row = reshape(1:rows*cols,rows,cols);
        J_support_polygon_row = reshape(J_support_polygon_row(1:2,support_polygon_flag{i}),[],1);
        dsupport_vert_pos(2*support_vert_count+(1:2*body_num_support_vert(i)),:) = ...
          J(J_support_polygon_row,:);
        support_vert_count = support_vert_count+body_num_support_vert(i);
      end
    end
    [rows,cols] = size(x);
    n = rows*cols;
    if(poscon_type(i) == 0|| poscon_type(i) == 1)
      if isa(contact_affs{i}(1),'ContactShapeAffordance')
        [x,J] = contact_affs{i}(1).inGeomFrame(x,J);
      end
      f(nf+(1:n))=x(:);
      G(ng+(1:numel(J))) = J(:);
      nf=nf+n;
      ng = ng+numel(J);
    elseif(poscon_type(i) == 2)
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
    if(contact_state{i}{1} == ActionKinematicConstraint.COLLISION_AVOIDANCE)
      num_rigid_bodies = length(contact_affs{i});
      col_dist = zeros(num_rigid_bodies,1);
      dcol_dist = zeros(num_rigid_bodies,nq);
      for j = 1:num_rigid_bodies
        [col_dist(j),dcol_dist(j,:)] = pairwiseContactDistance(obj,kinsol,body_ind(i),contact_affs{i}{j});
      end
      f(nf+(1:num_rigid_bodies)) = col_dist;
      G(ng+(1:num_rigid_bodies*nq)) = dcol_dist(:);
      nf = nf+num_rigid_bodies;
      ng = ng+num_rigid_bodies*nq;
    end
  else
    % Gaze constraint here
    [x,J] = forwardKin(obj,kinsol,body_ind(i),[0;0;0],2); %2 for quaternions
    
    gaze_axis = gaze_con{i}.axis;
    if gaze_con{i}.type == 2,
      % Desired gaze direction
      gaze_vec = gaze_con{i}.target - x(1:3);
      gaze_len = norm(gaze_vec);
      
      quat_vec = [0; -(gaze_vec/gaze_len + gaze_axis)];
      quat_len = norm(quat_vec);
      quat_des = quat_vec/norm(quat_len);
      
      dquatvecdx = eye(3)/gaze_len - gaze_vec*gaze_vec'/(gaze_len^3);
      dquat = (eye(3)/quat_len - quat_vec(2:4)*quat_vec(2:4)'/(quat_len^3))*dquatvecdx;
            
      dquat_des = [zeros(1,nq); dquat*J(1:3,:)];
    else
      quat_des = gaze_con{i}.orientation;
      dquat_des = zeros(4,nq);
    end
    
    [axis_err, daxis_err] = quatDiffAxisInvar(x(4:7),quat_des,gaze_axis);
    daxis_err_dq = daxis_err(1:4)*J(4:7,:) + daxis_err(5:8)*dquat_des;
    f(nf + 1) = axis_err;
    G(ng+(1:nq)) = daxis_err_dq';
    nf = nf + 1;
    ng = ng + nq;
    
    if gaze_con{i}.check_angle,
      [q_diff,dq_diff] = quatDiff(x(4:7),quat_des);
      dq_diff_dq = dq_diff(1:4)*J(4:7,:) + dq_diff(5:8)*dquat_des;
      f(nf + 1) = q_diff(1);

      G(ng+(1:nq)) = dq_diff_dq';
      nf = nf + 1;
      ng = ng + nq;
    end    
  end
end
if(quasiStaticFlag)
    support_vert_center = mean(support_vert_pos,2);
    dsupport_vert_center = [mean(dsupport_vert_pos(1:2:end,:),1);mean(dsupport_vert_pos(2:2:end,:),1)];
    support_vert_pos = shrinkFactor*support_vert_pos+(1-shrinkFactor)*bsxfun(@times,support_vert_center,ones(1,total_num_support_vert));
    dsupport_vert_pos = shrinkFactor*dsupport_vert_pos+(1-shrinkFactor)*repmat(dsupport_vert_center,total_num_support_vert,1);
    f(nf+(1:3)) = [com(1:2)-support_vert_pos*support_vert_weight;0];
    G(ng+(1:2*(nq+total_num_support_vert))) = reshape([dcom(1:2,:)-[support_vert_weight'*dsupport_vert_pos(1:2:end,:);...
        support_vert_weight'*dsupport_vert_pos(2:2:end,:)] -support_vert_pos],[],1);
    nf = nf+3;
    ng = ng+2*(nq+total_num_support_vert);
end
end
