function [t_breaks,q,qdot,qddot,info] = inverseKinSequence(obj,q0,qdot0,action_sequence,options)
% attempts to solve the optimization problem
% min_(q1,q2,...,qk,qdotk)
%  0.5*sum(qddot(j))'*Qa*qddot(j)+0.5*sum(qdot(i))'Qv*qdot(i)+(q(i)-q_nom)'Q*(q(i)-q_nom)
%   j = 0,1,...,k.   i = 1,2,...,k
% subject to
%    q(t) is a polynomial interpolated by (q1, q2,...,qk) and
%    action_sequence.tspan
%    q(t_i) satisfies the action_sequence at time t
%    q(t_i) satisfies that the COM is in the support polygon, this is
%    OPTIONAL

% @param qtraj The piecewise cubic interpolated trajectory, matches q and
% qdot at the knot points.
% @param q0 the posture at the starting time
% @param qdot0 the velocity at the starting time
% @action_sequence an ActionSequence object, which specifies kinematic
% constraint for the whole time interval
% options qdotf.ub the upper bound of final velocity
% options qdotf.lb the lower bound of final velocity
% options Qa penalizes the joint acceleration
% options Qv penalize the joint velocity
% options q_nom the nominal postures
% options Q penalizes the difference from the nominal postures
% options nSample number of samples for the q(t)
% options quasiStatcFlag true if we want the COM lies inside the
% support polygon
% options qtraj0 the initial guess of q;
% options optimizeSparsity true if the user want to further find out the
% nonzero entries in gradient computation

nq = obj.getNumDOF;
if ~isfield(options,'use_mex') options.use_mex = exist('inverseKinmex')==3; end
if(~isfield(options,'qdotf')) options.qdotf.ub = inf(nq,1); options.qdotf.lb = -inf(nq,1);end
if(~isfield(options,'Qa')) options.Qa = eye(nq); end
if(~isfield(options,'Qv')) options.Qv = zeros(nq); end
if(~isfield(options,'q_nom')) options.q_nom = zeros(nq,1); options.Q = zeros(nq);end
if(~isfield(options,'Q')) options.Q = eye(nq); end
if(~isfield(options,'nSample')) options.nSample = 2; end
if(~isfield(options,'quasiStaticFlag')) options.quasiStaticFlag = false; end
if(~isfield(options,'optimizeSparsity')) options.optimizeSparsity = false; end
Fmin = 0;
Fmax = inf;

t_breaks = linspace(action_sequence.tspan(1),action_sequence.tspan(end),options.nSample+1);
t_breaks = unique([t_breaks action_sequence.key_time_samples]);
nSample = length(t_breaks)-1;
dt = diff(t_breaks);
dt_ratio = dt(1:end-1)./(dt(2:end));
body_ind = cell(1,nSample);
body_pos = cell(1,nSample);
world_pos = cell(1,nSample);
do_rot = cell(1,nSample);

% Currently I ONLY handle the support polygon defined by the ground contact
support_polygon_flags = cell(1,nSample);
contact_tol = 1e-4;
num_sequence_support_vertices = cell(1,nSample); % The active contacts that determine the support polygon
num_sample_support_vertices = zeros(1,nSample);
iGfun = ones(nq*(nSample+1),1);
jGvar = (1:nq*(nSample+1))';
nF = 1;
cum_num_weights = 0;
for i = 1:nSample
    ikargs = action_sequence.getIKArguments(t_breaks(i+1));
    j = 1;
    n = 1;
    while j<length(ikargs)
        if(isa(ikargs{j},'RigidBody'))
            ikargs{j} = find(obj.body==ikargs{j},1);
        end
        body_ind{i}(n) = ikargs{j};
        if(body_ind{i}(n) == 0)
            body_pos{i}{n} = [0;0;0];
            world_pos{i}{n} = ikargs{j+1};
            support_polygon_flags{i}{n} = false;
            j = j+2;
            [rows,mi] = size(body_pos{i}{n});
            if(rows~=3) error('bodypos must be 3xmi');end
        else
            body_pos{i}{n} = ikargs{j+1};
            if(ischar(body_pos{i}{n})||numel(body_pos{i}{n})==1)
                body_pos{i}{n} = getContactPoints(obj.body(body_ind{i}(n)),body_pos{i}{n});
            end
            world_pos{i}{n} = ikargs{j+2};
            j = j+3;
            [rows,mi] = size(body_pos{i}{n});
            if(rows~=3) error('bodypos must be 3xmi');end
        end
        if(isstruct(world_pos{i}{n}))
            if(~isfield(world_pos{i}{n},'min')||~isfield(world_pos{i}{n},'max'))
                error('if worldpos is a struct, it must have fields .min and .max')
            end
            maxpos = world_pos{i}{n}.max;
            minpos = world_pos{i}{n}.min;
        else
            maxpos = world_pos{i}{n};
            minpos = world_pos{i}{n};
        end
        [rows,cols] = size(minpos);
        if(rows~=3 && rows~=6) error('world pos must have 3 or 6 rows');end
        if(body_ind{i}(n) ==0 &&rows~=3) error('com pos must have only 3 rows');end
        if(cols~=mi) error('worldpos must have the same number of elements as bodypos');end
        sizecheck(maxpos,[rows,mi]);
        
        minpos(isnan(minpos)) = -inf;
        maxpos(isnan(maxpos)) = inf;
        do_rot{i}(n) = (rows == 6);
        [col_G,row_G] = meshgrid(1:nq,1:rows*cols);
        iGfun = [iGfun;nF+row_G(:)];
        jGvar = [jGvar;nq*(i-1)+col_G(:)];
        nF = nF+rows*cols;
        support_polygon_flags{i}{n} = maxpos(3,:)<contact_tol;
        num_sequence_support_vertices{i}(n) = sum(support_polygon_flags{i}{n});
        Fmin = [Fmin;minpos(:)];
        Fmax = [Fmax;maxpos(:)];
        n = n+1;
        
    end
    if(options.quasiStaticFlag)
        num_sample_support_vertices(i) = sum(num_sequence_support_vertices{i});
        iGfun = [iGfun;nF+repmat((1:2)',nq+num_sample_support_vertices(i),1)];
        jGvar = [jGvar;reshape(bsxfun(@times,[nq*(i-1)+(1:nq) nq*(nSample+1)+cum_num_weights+(1:num_sample_support_vertices(i))],[1;1]),[],1)];
        Fmin = [Fmin;0;0];
        Fmax = [Fmax;0;0];
        cum_num_weights = cum_num_weights+num_sample_support_vertices(i);
        nF = nF+2;
    end
end
total_sequence_support_vertices = sum(cat(2,num_sequence_support_vertices{:}));
% Suppose the joint angles are interpolated using cubic splines, then the
velocity_mat1_diag1 = reshape([ones(nq,1) repmat(dt(1:end-1).*(2+2*dt_ratio),nq,1) ones(nq,1)],[],1);
velocity_mat1_diag2 = reshape([zeros(nq,1) repmat(dt(1:end-1).*dt_ratio,nq,1)],[],1);
velocity_mat1_diag3 = [reshape(repmat(dt(1:end-1),nq,1),[],1);zeros(nq,1)];
velocity_mat1 = sparse((1:nq*(nSample+1))',(1:nq*(nSample+1))',velocity_mat1_diag1)...
    +sparse((1:nq*(nSample))',nq+(1:nq*nSample)',velocity_mat1_diag2,nq*(nSample+1),nq*(nSample+1))...
    +sparse(nq+(1:nq*nSample)',(1:nq*nSample)',velocity_mat1_diag3,nq*(nSample+1),nq*(nSample+1));


velocity_mat2_diag1 = reshape([zeros(nq,1) bsxfun(@times,3*ones(1,nSample-1)-3*dt_ratio.^2,ones(nq,1)) zeros(nq,1)],[],1);
velocity_mat2_diag2 = reshape([zeros(nq,1) bsxfun(@times,3*dt_ratio.^2,ones(nq,1))],[],1);
velocity_mat2_diag3 = [-3*ones(nq*(nSample-1),1);zeros(nq,1)];
velocity_mat2 = sparse((1:nq*(nSample+1))',(1:nq*(nSample+1))',velocity_mat2_diag1)...
    +sparse((1:nq*nSample)',nq+(1:nq*nSample)',velocity_mat2_diag2,nq*(nSample+1),nq*(nSample+1))...
    +sparse(nq+(1:nq*nSample)',(1:nq*nSample)',velocity_mat2_diag3,nq*(1+nSample),nq*(1+nSample));
velocity_mat = velocity_mat1\velocity_mat2;
velocity_mat = velocity_mat(nq+1:end-nq,:);

% [qddot(0);...qddot(k)] =
% accel_mat*[q(0);...;q(k)]+accel_mat_qdot0*qdot(0)+accel_mat_qdof*qdot(k)
accel_mat1_diag1 = reshape(bsxfun(@times,[-6./(dt.^2) -6/(dt(end)^2)],ones(nq,1)),[],1);
accel_mat1_diag2 = reshape(bsxfun(@times,6./(dt.^2),ones(nq,1)),[],1);
accel_mat1_diag3 = 6/(dt(end)^2)*ones(nq,1);
accel_mat1 = sparse((1:nq*(nSample+1))',(1:nq*(nSample+1))',accel_mat1_diag1)...
    +sparse((1:nq*nSample)',nq+(1:nq*nSample)',accel_mat1_diag2,nq*(nSample+1),nq*(nSample+1))...
    +sparse(nq*nSample+(1:nq)',nq*(nSample-1)+(1:nq)',accel_mat1_diag3,nq*(nSample+1),nq*(nSample+1));
accel_mat2_diag1 = reshape(bsxfun(@times,[-4./dt 5/dt(end)],ones(nq,1)),[],1);
accel_mat2_diag2 = reshape(bsxfun(@times,-2./dt,ones(nq,1)),[],1);
accel_mat2_diag3 = 4/dt(end)*ones(nq,1);
accel_mat2 = sparse((1:nq*(nSample+1))',(1:nq*(nSample+1))',accel_mat2_diag1)...
    +sparse((1:nq*nSample)',nq+(1:nq*nSample)',accel_mat2_diag2,nq*(nSample+1),nq*(nSample+1))...
    +sparse(nq*nSample+(1:nq)',nq*(nSample-1)+(1:nq)',accel_mat2_diag3,nq*(nSample+1),nq*(nSample+1));
accel_mat = accel_mat1+accel_mat2(:,nq+1:end-nq)*velocity_mat;
accel_mat_qdot0 = accel_mat2(:,1:nq);
accel_mat_qdotf = accel_mat2(:,end-nq+1:end);
A = [];
iAfun = [];
jAvar = [];
whigh = [repmat(obj.joint_limit_max,nSample,1);options.qdotf.ub];
wlow = [repmat(obj.joint_limit_min,nSample,1);options.qdotf.lb];
if(~isfield(options,'qtraj0'))
    w0 = [repmat(q0,nSample,1);zeros(nq,1)];
else
    w0 = [reshape(options.qtraj0.eval(t_breaks(2:end)),[],1);options.qtraj0.deriv(t_breaks(end))];
end
if(options.quasiStaticFlag)
    Fmax = [Fmax;ones(nSample,1)];
    Fmin = [Fmin;ones(nSample,1)];
    whigh = [whigh;ones(total_sequence_support_vertices,1)];
    wlow = [wlow;zeros(total_sequence_support_vertices,1)];
    cum_num_weights = 0;
    for i = 1:nSample
        iAfun = [iAfun;(nF+i)*ones(num_sample_support_vertices(i),1)];
        jAvar = [jAvar;nq*(nSample+1)+cum_num_weights+(1:num_sample_support_vertices(i))'];
        A = [A;ones(num_sample_support_vertices(i),1)];
        w0 = [w0;1/num_sample_support_vertices(i)*ones(num_sample_support_vertices(i),1)];
        cum_num_weights = cum_num_weights+num_sample_support_vertices(i);
    end
end
nF = length(Fmin);
nG = length(iGfun);

snset('Major optimality tolerance=5e-3');
setSNOPTParam(options,'Iterations Limit',1e5);
setSNOPTParam(options,'Major Iterations Limit',200);
setSNOPTParam(options,'Superbasics Limit',1000);

if(options.optimizeSparsity)
    nTrials = 10;
    w_tmp = rand(length(w0),nTrials);
    G_tmp = zeros(length(iGfun),nTrials);
    for i = 1:nTrials
        [~,G_tmp(:,i)] = ik_tmp(w_tmp(:,i),obj,nq,nSample+1,nF,nG,options.Qa,options.Qv,options.Q,body_ind,body_pos,do_rot,q0,qdot0,options.q_nom,velocity_mat,accel_mat,accel_mat_qdot0,accel_mat_qdotf,num_sequence_support_vertices,num_sample_support_vertices,total_sequence_support_vertices,support_polygon_flags,options.quasiStaticFlag,true(length(iGfun),1));
    end
    nonzero_ind = any(G_tmp~=0,2);
    iGfun = iGfun(nonzero_ind);
    jGvar = jGvar(nonzero_ind);
    ik = @(w) ik_tmp(w,obj,nq,nSample+1,nF,nG,options.Qa,options.Qv,options.Q,body_ind,body_pos,do_rot,q0,qdot0,options.q_nom,velocity_mat,accel_mat,accel_mat_qdot0,accel_mat_qdotf,num_sequence_support_vertices,num_sample_support_vertices,total_sequence_support_vertices,support_polygon_flags,options.quasiStaticFlag,nonzero_ind);
else
    ik = @(w) ik_tmp(w,obj,nq,nSample+1,nF,nG,options.Qa,options.Qv,options.Q,body_ind,body_pos,do_rot,q0,qdot0,options.q_nom,velocity_mat,accel_mat,accel_mat_qdot0,accel_mat_qdotf,num_sequence_support_vertices,num_sample_support_vertices,total_sequence_support_vertices,support_polygon_flags,options.quasiStaticFlag,true(length(iGfun),1));
end
global SNOPT_USERFUN;
SNOPT_USERFUN = ik;

% keyboard;
% tic
[w,F,info] = snopt(w0,wlow,whigh,Fmin,Fmax,'snoptUserfun',0,1,A,iAfun,jAvar,iGfun,jGvar);
% toc

q = w(1:(nq*nSample));
qdotf = w(nq*nSample+(1:nq));
qdot = [qdot0 reshape(velocity_mat*[q0;q(:)],nq,nSample-1) qdotf];
qddot = reshape(accel_mat*[q0;q(:)]+accel_mat_qdot0*qdot0+accel_mat_qdotf*qdotf,nq,nSample+1);
q = [q0 reshape(q,nq,nSample)];
% qtraj = PPTrajectory(pchipDeriv(t_breaks,q,qdot));
% keyboard;
end

function [f,G] = ik_tmp(w,obj,nq,nT,nF,nG,Qa,Qv,Q,body_ind,body_pos,do_rot,q0,qdot0,q_nom,velocity_mat,accel_mat,accel_mat_qd0,accel_mat_qdf,num_sequence_support_vertices,num_sample_support_vertices,nWeights,support_vert_flags,quasiStaticFlag,nonzero_ind_G)
% profile on
% [qdot(1);...;qdot(nT-1)] = velocity_mat*[q(0);q(1);...;q(nT)];
% [qddot(0);...;qdot(nT)] =
% accel_mat*[q(0);q(1);...;q(nT)]+accel_mat_qd0*qdot(0)+accel_mat_qdf*qdot(nT);
f = zeros(nF,1);
G = zeros(nG,1);
q = reshape(w(1:nq*(nT-1)),nq,nT-1);
qdotf = w(nq*(nT-1)+(1:nq));
if(quasiStaticFlag)
    weights = mat2cell(w(nq*nT+(1:nWeights)),num_sample_support_vertices,1);
end
qdot = [reshape(velocity_mat*[q0;q(:)],nq,nT-2) qdotf]; %[qdot(1) qdot(2) ... qdot(nT)]
qddot = reshape(accel_mat*[q0;q(:)]+accel_mat_qd0*qdot0+accel_mat_qdf*qdotf,nq,nT); % [qddot(0) qddot(1) ... qddot(nT)]
q_diff = q-bsxfun(@times,ones(1,nT-1),q_nom);
f(1) = 0.5*sum(sum((Qv*qdot(:,1:end-1)).*qdot(:,1:end-1)))...
    +0.5*sum(sum((Q*q_diff).*q_diff))...
    +0.5*sum(sum((Qa*qddot).*qddot))...
    +0.5*qdotf'*Q*qdotf;
G(1:nq*(nT-1)) = reshape(reshape((qdot(:,1:end-1)'*Qv)',1,[])*velocity_mat(:,nq+1:end),[],1)...
    +reshape(Q*q_diff,[],1)...
    +reshape(reshape(Qa*qddot,1,[])*accel_mat(:,nq+1:end),[],1);
G(nq*(nT-1)+(1:nq)) = reshape(reshape(Qa*qddot,1,[])*accel_mat_qdf,[],1)...
    +reshape(qdotf'*Q,[],1);
if(nF<2) return; end
nf = 1;
ng = nq*nT;
for i = 1:nT-1
    kinsol = doKinematics(obj,q(:,i));
    if(quasiStaticFlag)
        support_vert_pos = zeros(2,num_sample_support_vertices(i));
        dsupport_vert_pos = zeros(2*num_sample_support_vertices(i),nq);
        total_body_support_vert = 0;
        [com,dcom] = getCOM(obj,kinsol);
    end
    for j = 1:length(body_ind{i})
        
        if(body_ind{i}(j) == 0)
            if(quasiStaticFlag)
                x = com;
                J = dcom;
            else
                [x,J] = getCOM(obj,kinsol);
            end
        else
           [x,J] = forwardKin(obj,kinsol,body_ind{i}(j),body_pos{i}{j},do_rot{i}(j)); 
           if(quasiStaticFlag)
               support_vert_pos(:,total_body_support_vert+(1:num_sequence_support_vertices{i}(j)))...
                   = x(1:2,support_vert_flags{i}{j});
               Jtmp = reshape(J,size(x,1),size(x,2),nq);
               dsupport_vert_pos(total_body_support_vert*2+(1:2*num_sequence_support_vertices{i}(j)),:)...
                   = reshape(Jtmp(1:2,support_vert_flags{i}{j},:),[],nq);
               total_body_support_vert = total_body_support_vert+num_sequence_support_vertices{i}(j);
           end
        end
        n = numel(x);
        f(nf+(1:n)) = x(:);
        G(ng+(1:n*nq)) = J(:);
        nf = nf+n;
        ng = ng+n*nq;
    end
    if(quasiStaticFlag)
        f(nf+(1:2)) = support_vert_pos*weights{i}-com(1:2);
        G(ng+(1:2*(nq+num_sample_support_vertices(i)))) = [reshape([weights{i}'*dsupport_vert_pos(1:2:end,:)-dcom(1,:);...
            weights{i}'*dsupport_vert_pos(2:2:end,:)-dcom(2,:)],[],1);...
            reshape(support_vert_pos,[],1)];
        nf = nf+2;
        ng = ng+2*(nq+num_sample_support_vertices(i));
    end
end
G = G(nonzero_ind_G);
% profile off
% profile viewer
end

function setSNOPTParam(options,paramstring,default)
  str=paramstring(~isspace(paramstring));
  if (isfield(options,str))
    snset([paramstring,'=',num2str(getfield(options,str))]);
  else
    snset([paramstring,'=',num2str(default)]);
  end
end