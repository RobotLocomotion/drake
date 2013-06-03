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
% options q_traj_nom the nominal postures
% options Q penalizes the difference from the nominal postures
% options nSample number of samples for the q(t)
% options quasiStaticFlag true if we want the COM lies inside the
% support polygon
% options shrinkFactor   determines how much shrinking we want to compute
%                        support polygon vertices from the contact poinst
% options qtraj0 the initial guess of q;
% options optimizeSparsity true if the user want to further find out the
% nonzero entries in gradient computation

nq = obj.getNumDOF;
if ~isfield(options,'use_mex') options.use_mex = exist('inverseKinmex')==3; end
if(~isfield(options,'qdotf')) options.qdotf.ub = inf(nq,1); options.qdotf.lb = -inf(nq,1);end
if(~isfield(options,'Qa')) options.Qa = eye(nq); end
if(~isfield(options,'Qv')) options.Qv = zeros(nq); end
if(~isfield(options,'q_traj_nom')) options.q_traj_nom = ConstantTrajectory(zeros(nq,1)); options.Q = zeros(nq);end
q_traj_nom = options.q_traj_nom;
sizecheck(q_traj_nom,[nq,1]);
if(~isfield(options,'Q')) options.Q = eye(nq); end
if(~isfield(options,'nSample')) options.nSample = 2; end
if(~isfield(options,'quasiStaticFlag')) options.quasiStaticFlag = false; end
if(~isfield(options,'shrinkFactor')) shrinkFactor = 0.95; else shrinkFactor = options.shrinkFactor;end
if(~isfield(options,'optimizeSparsity')) options.optimizeSparsity = false; end
if(~isfield(options,'jointLimitMin')) options.jointLimitMin = obj.joint_limit_min; end
if(~isfield(options,'jointLimitMax')) options.jointLimitMax = obj.joint_limit_max; end
if(isfield(options,'quasiStaticFlag'))
    quasiStaticFlag = options.quasiStaticFlag;
else
    quasiStaticFlag = false;
end

joint_limit_min = options.jointLimitMin; 
joint_limit_max = options.jointLimitMax; 

F_name = {};
Fmin = 0;
Fmax = inf;
F_name{1} = 'obj';

t_breaks = linspace(action_sequence.tspan(1),action_sequence.tspan(end),options.nSample+1);
t_breaks = unique([t_breaks action_sequence.key_time_samples]);
%fprintf('t(8) = %4.2f\n',t_breaks(8));
nSample = length(t_breaks)-1;
q_nom = q_traj_nom.eval(t_breaks(2:end));
dt = diff(t_breaks);
dt_ratio = dt(1:end-1)./(dt(2:end));
body_ind = cell(1,nSample);
body_pos = cell(1,nSample);
world_pos = cell(1,nSample);
rotation_type = cell(1,nSample);
static_pts_flag = cell(1,nSample);
static_pts_ind = cell(1,nSample);
num_static_pts = cell(1,nSample);
static_pts = cell(1,nSample); % static_pts{i}{n} records the body_pos that is in static contact for time i, body n
prev_body_ind = cell(1,nSample); % body_ind{i-1}{prev_body_ind{i}{n}} = body{i}{n} 
prev_pts_ind = cell(1,nSample); % body_pos{i-1}{prev_body_ind{i}{n}(:,prev_pts_ind{i}{n}) = static_pts{i}{n}
% Currently I ONLY handle the support polygon defined by the ground contact
support_polygon_flags = cell(1,nSample);
contact_states = cell(1,nSample);
contact_affs = cell(1,nSample);
contact_dists = cell(1,nSample);
contact_body_pts_ind = cell(1,nSample);
num_contact_body_pts = cell(1,nSample);

num_sequence_support_vertices = cell(1,nSample); % The active contacts that determine the support polygon
num_sample_support_vertices = zeros(1,nSample);
total_sequence_support_vertices = 0;
iGfun = ones(nq*(nSample+1),1);
jGvar = (1:nq*(nSample+1))';
nF = 1;
cum_quat_des = 0;
whigh = [repmat(joint_limit_max,nSample,1);options.qdotf.ub];
wlow = [repmat(joint_limit_min,nSample,1);options.qdotf.lb];
if(~isfield(options,'qtraj0'))
    w0 = [repmat(q0,nSample,1);zeros(nq,1)];
else
    w0 = [reshape(options.qtraj0.eval(t_breaks(2:end)),[],1);options.qtraj0.deriv(t_breaks(end))];
end
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
            contact_states{i}{n} = ikargs{j+2};
            j = j+5;
            [rows,mi] = size(body_pos{i}{n});
            if(rows~=3) error('bodypos must be 3xmi');end
        else
            body_pos{i}{n} = ikargs{j+1};
            if(ischar(body_pos{i}{n})||numel(body_pos{i}{n})==1)
                body_pos{i}{n} = getContactPoints(obj.body(body_ind{i}(n)),body_pos{i}{n});
            end
            world_pos{i}{n} = ikargs{j+2};
            if(quasiStaticFlag)
              support_polygon_flags{i}{n} = ...
                any(cell2mat(ikargs{j+3}') == ActionKinematicConstraint.STATIC_PLANAR_CONTACT,1) | ...
                any(cell2mat(ikargs{j+3}') == ActionKinematicConstraint.STATIC_GRIP_CONTACT,1);
            end
            contact_states{i}{n} = ikargs{j+3};
            contact_affs{i}{n} = ikargs{j+4};
            contact_dists{i}{n} = ikargs{j+5};
            j = j+6;
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
        if(rows~=3 && rows~=6 && rows ~=7) error('world pos must have 3, 6 or 7 rows');end
        if(body_ind{i}(n) ==0 &&rows~=3) error('com pos must have only 3 rows');end
        if(cols~=mi) error('worldpos must have the same number of elements as bodypos');end
        sizecheck(maxpos,[rows,mi]);
        rotation_type{i}(n) = 0*(rows == 3)+(rows == 6)+2*(rows == 7);
        
        static_pts_flag{i}{n} = any(cell2mat(contact_states{i}{n}') == 2,1) | ...
                          any(cell2mat(contact_states{i}{n}') == 3,1)|...
                          any(cell2mat(contact_states{i}{n}') == 4,1);
        static_pts_ind{i}{n} = find(static_pts_flag{i}{n});
        num_static_pts{i}(n) = sum(static_pts_flag{i}{n});
        % needs to determine which body points in the previous time
        % corresponds to the static body points in the current time
        if(false && num_static_pts{i}(n) > 0)
          if(i ~= 1)
            static_pts{i}{n} = body_pos{i}{n}(:,static_pts_flag{i}{n});
            prev_body_ind{i}(n) = find(body_ind{i-1}==body_ind{i}(n),1,'last');
            [~,~,prev_pts_ind{i}{n}] = intersect(static_pts{i}{n}',body_pos{i-1}{prev_body_ind{i}(n)}','rows','stable');
            prev_pts_ind{i}{n} = prev_pts_ind{i}{n}';
            if(length(prev_pts_ind{i}{n})~=num_static_pts{i}(n))
                error('inverseKinSequence:StaticContact','Static contact body points not found in the previous time sample');
            end
              Fmin = [Fmin;zeros(3*num_static_pts{i}(n),1)];
              Fmax = [Fmax;zeros(3*num_static_pts{i}(n),1)];
              [col_G,row_G] = meshgrid(1:2*nq,1:3*num_static_pts{i}(n));
              iGfun = [iGfun;nF+row_G(:)];
              jGvar = [jGvar;nq*(i-2)+col_G(:)];
              nF = nF+3*num_static_pts{i}(n);
              for l = 1:3*num_static_pts{i}(n)
                F_name =[F_name,{['static constraint at time ',num2str(t_breaks(i+1)),' for body ',num2str(body_ind{i}(n))]}];
              end
          else
            kinsol = doKinematics(obj,q0);
            x = forwardKin(obj,kinsol,body_ind{i}(n),body_pos{i}{n},0); 
            minpos(1:3,static_pts_ind{i}{n}) = x(:,static_pts_ind{i}{n});
            maxpos(1:3,static_pts_ind{i}{n}) = x(:,static_pts_ind{i}{n});
          end
        end
        minpos(isnan(minpos)) = -inf;
        maxpos(isnan(maxpos)) = inf;
        if(all(all(isinf(minpos)))&&all(all(isinf(maxpos))))
          continue;
        end

        if(quasiStaticFlag)
          num_sequence_support_vertices{i}(n) = sum(support_polygon_flags{i}{n});
        end
        if(rotation_type{i}(n) == 0 || rotation_type{i}(n) == 1)
          Fmin = [Fmin;minpos(:)];
          Fmax = [Fmax;maxpos(:)];
          [col_G,row_G] = meshgrid(1:nq,1:rows*cols);
          iGfun = [iGfun;nF+row_G(:)];
          jGvar = [jGvar;nq*(i-1)+col_G(:)];
          nF = nF+rows*cols;
          for l = 1:rows*cols
              if(body_ind{i}(n) == 0)
                  F_name = [F_name,{['com at time ',num2str(t_breaks(i+1))]}];
              else
                F_name = [F_name,{[obj.body(body_ind{i}(n)).linkname,' at time ',num2str(t_breaks(i+1))]}];
              end
          end
        elseif(rotation_type{i}(n) == 2)
          Fmin = [Fmin;reshape(minpos(1:3,:),[],1);ones(2,1)];
          Fmax = [Fmax;reshape(maxpos(1:3,:),[],1);ones(2,1)];
          iGfun = [iGfun;nF+repmat((1:3*cols)',nq,1);nF+3*cols+ones(nq+4,1);...
            nF+3*cols+1+ones(4,1)];
          jGvar = [jGvar;nq*(i-1)+reshape(bsxfun(@times,1:nq,ones(3*cols,1)),[],1);...
            nq*(i-1)+(1:nq)';...
            nq*(nSample+1)+cum_quat_des+(1:4)';...
            nq*(nSample+1)+cum_quat_des+(1:4)'];
          quat_max = min([maxpos(4:7,:) (1+eps)*ones(4,1)],[],2);
          quat_min = max([minpos(4:7,:) -(1+eps)*ones(4,1)],[],2);
          whigh = [whigh;quat_max];
          wlow = [wlow;quat_min];
          quat_des0 = (quat_max+quat_min)/2;
          quat_des0_norm = sqrt(sum(quat_des0.*quat_des0,1));
          if(quat_des0_norm ~=0)
            quat_des0 = quat_des0./quat_des0_norm;
          else
            quat_des0 = 1/2*ones(4,1);
          end
          w0 = [w0;quat_des0];
          nF = nF+3*cols+2;
          cum_quat_des = cum_quat_des+4;
          for l = 1:(3*cols+2)
              F_name = [F_name,{[obj.body(body_ind{i}(n)).linkname,' at time ',num2str(t_breaks(i+1))]}];
          end
        end
        % parse the affordance contact
        if(body_ind{i}(n) ~=0 )
            num_contact_aff = length(contact_affs{i}{n});
            contact_body_pts_ind{i}{n} = cell(1,num_contact_aff);
            num_contact_body_pts{i}{n} = zeros(1,num_contact_aff);
            for aff_ind = 1:num_contact_aff
                aff = contact_affs{i}{n}{aff_ind};
                contact_body_pts_ind{i}{n}{aff_ind} = find(contact_states{i}{n}{aff_ind} ~= ActionKinematicConstraint.UNDEFINED_CONTACT);
                num_contact_body_pts{i}{n}(aff_ind) = length(contact_body_pts_ind{i}{n}{aff_ind});
                if(num_contact_body_pts{i}{n}(aff_ind)>0)
                    contact_dist_ub = contact_dists{i}{n}{aff_ind}.max;
                    contact_dist_ub = contact_dist_ub(:,contact_body_pts_ind{i}{n}{aff_ind});
                    contact_dist_lb = contact_dists{i}{n}{aff_ind}.min;
                    contact_dist_lb = contact_dist_lb(:,contact_body_pts_ind{i}{n}{aff_ind});
                    sizecheck(contact_dist_ub,[1,num_contact_body_pts{i}{n}(aff_ind)]);
                    sizecheck(contact_dist_lb,[1,num_contact_body_pts{i}{n}(aff_ind)]);
                    if(isa(aff,'ContactAffordance'))
                        if(isa(aff,'PlanarContactAffordance'))
                            residual_ub = contact_dist_ub;
                            residual_lb = contact_dist_lb;
                        elseif(isa(aff,'PolygonalContactAffordance'))
                            residual_ub = [contact_dist_ub;inf(aff.n_vert,num_contact_body_pts{i}{n}(aff_ind))];
                            residual_lb = [contact_dist_lb;zeros(aff.n_vert,num_contact_body_pts{i}{n}(aff_ind))];
                        elseif(isa(aff,'CylindricalContactAffordance'))
                            residual_ub = [contact_dist_ub;aff.height/2*ones(1,num_contact_body_pts{i}{n}(aff_ind))];
                            residual_lb = [contact_dist_lb;-aff.height/2*ones(1,num_contact_body_pts{i}{n}(aff_ind))];
                        else
                            residual_ub = [];
                            residual_lb = [];
                        end
                    else
                        error('Must supply with a contact affordance');
                    end
                    Fmin = [Fmin;residual_lb(:)];
                    Fmax = [Fmax;residual_ub(:)];
                    num_residual = numel(residual_lb);
                    if(num_residual > 0)
                      iGfun = [iGfun;nF+reshape(bsxfun(@times,(1:num_residual)',ones(1,nq)),[],1)];
                      jGvar = [jGvar;nq*(i-1)+reshape(bsxfun(@times,ones(num_residual,1),(1:nq)),[],1)];
                      nF = nF+num_residual;
                      for l = 1:num_residual
                        F_name = [F_name,{[obj.body(body_ind{i}(n)).linkname,' contact with ',aff.name,' at time ',num2str(t_breaks(i+1))]}];
                      end
                    end
                end
            end
        end
      n = n+1;
        
    end
    
end
if(quasiStaticFlag)
  total_sequence_support_vertices = sum(cat(2,num_sequence_support_vertices{:}));
end
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


if(options.quasiStaticFlag)
    whigh = [whigh;ones(total_sequence_support_vertices,1)];
    wlow = [wlow;zeros(total_sequence_support_vertices,1)];
    cum_num_weights = 0;
    for i = 1:nSample
        num_sample_support_vertices(i) = sum(num_sequence_support_vertices{i});
        if(num_sample_support_vertices(i) == 0)
          error('inverseKinSequence:NoContacts','No contact points at time %4.2f', ...
            t_breaks(i+1))
        end
        iGfun = [iGfun;nF+repmat((1:2)',nq+num_sample_support_vertices(i),1)];
        jGvar = [jGvar;reshape(bsxfun(@times,[nq*(i-1)+(1:nq) nq*(nSample+1)+cum_quat_des+cum_num_weights+(1:num_sample_support_vertices(i))],[1;1]),[],1)];
        iAfun = [iAfun;nF+3*ones(num_sample_support_vertices(i),1)];
        jAvar = [jAvar;nq*(nSample+1)+cum_quat_des+cum_num_weights+(1:num_sample_support_vertices(i))'];
        A = [A;ones(num_sample_support_vertices(i),1)];
        w0 = [w0;1/num_sample_support_vertices(i)*ones(num_sample_support_vertices(i),1)];
        Fmin = [Fmin;0;0;1];
        Fmax = [Fmax;0;0;1];
        cum_num_weights = cum_num_weights+num_sample_support_vertices(i);
        nF = nF+3;
        for l = 1:3
            F_name = [F_name,{['quasi static at time ',num2str(t_breaks(i+1))]}];
        end
    end
end
nF = length(Fmin);
nG = length(iGfun);
%snprint('snopt.out');
snset('Major optimality tolerance=5e-4');
setSNOPTParam(options,'Iterations Limit',1e6);
setSNOPTParam(options,'Major Iterations Limit',5e2);
setSNOPTParam(options,'Superbasics Limit',1000);
%setSNOPTParam(options,'Hessian','full memory');

if(options.optimizeSparsity)
    nTrials = 10;
    w_tmp = rand(length(w0),nTrials);
    G_tmp = zeros(length(iGfun),nTrials);
    for i = 1:nTrials
        [~,G_tmp(:,i)] = ik_tmp(w_tmp(:,i),obj,nq,nSample+1,nF,nG,...
            options.Qa,options.Qv,options.Q,body_ind,body_pos,...
            rotation_type,q0,qdot0,q_nom,velocity_mat,accel_mat,...
            accel_mat_qdot0,accel_mat_qdotf,num_sequence_support_vertices,...
            num_sample_support_vertices,total_sequence_support_vertices,...
            cum_quat_des,support_polygon_flags,options.quasiStaticFlag,true(length(iGfun),1),...
            contact_states,static_pts_flag,static_pts_ind,num_static_pts,...
            prev_body_ind,prev_pts_ind,shrinkFactor,contact_affs,contact_dists,...
            contact_body_pts_ind,num_contact_body_pts);
    end
    nonzero_ind = any(G_tmp~=0,2);
    iGfun = iGfun(nonzero_ind);
    jGvar = jGvar(nonzero_ind);
    ik = @(w) ik_tmp(w,obj,nq,nSample+1,nF,nG,options.Qa,options.Qv,...
        options.Q,body_ind,body_pos,rotation_type,q0,qdot0,q_nom,...
        velocity_mat,accel_mat,accel_mat_qdot0,accel_mat_qdotf,...
        num_sequence_support_vertices,num_sample_support_vertices,...
        total_sequence_support_vertices,cum_quat_des,support_polygon_flags,...
        options.quasiStaticFlag,nonzero_ind,contact_states,static_pts_flag,...
        static_pts_ind,num_static_pts,prev_body_ind,prev_pts_ind,shrinkFactor,...
        contact_affs,contact_dists,contact_body_pts_ind,num_contact_body_pts);
else
    ik = @(w) ik_tmp(w,obj,nq,nSample+1,nF,nG,options.Qa,options.Qv,options.Q,...
        body_ind,body_pos,rotation_type,q0,qdot0,q_nom,velocity_mat,...
        accel_mat,accel_mat_qdot0,accel_mat_qdotf,num_sequence_support_vertices,...
        num_sample_support_vertices,total_sequence_support_vertices,cum_quat_des,...
        support_polygon_flags,options.quasiStaticFlag,true(length(iGfun),1),...
        contact_states,static_pts_flag,static_pts_ind,num_static_pts,...
        prev_body_ind,prev_pts_ind,shrinkFactor,contact_affs,contact_dists,...
        contact_body_pts_ind,num_contact_body_pts);
end
global SNOPT_USERFUN;
SNOPT_USERFUN = ik;

% keyboard;
disp('Starting SNOPT ...');
tic
[w,F,info] = snopt(w0,wlow,whigh,Fmin,Fmax,'snoptUserfun',0,1,A,iAfun,jAvar,iGfun,jGvar);
toc

if(info == 13)
    f_sol = SNOPT_USERFUN(w);
    f_sol = f_sol+sparse(iAfun,jAvar,A,nF,length(w))*w;
    max_ub_error = max(f_sol-Fmax);
    max_ub_error = max_ub_error*(max_ub_error>0);
    max_lb_error = max(f_sol-Fmin);
    max_lb_error = max_lb_error*(max_lb_error<0);
    if(max_ub_error+max_lb_error>1e-4)
        info = 13;
    else
        info = 1;
    end
end
if (info~=1)
  [str,cat_] = snoptInfo(info);
  warning('SNOPT:InfoNotOne',['SNOPT exited w/ info = ',num2str(info),'.\n',cat_,': ',str,'\n  Check p19 of Gill06 for more information.']);
end
q = w(1:(nq*nSample));
qdotf = w(nq*nSample+(1:nq));
qdot = [qdot0 reshape(velocity_mat*[q0;q(:)],nq,nSample-1) qdotf];
qddot = reshape(accel_mat*[q0;q(:)]+accel_mat_qdot0*qdot0+accel_mat_qdotf*qdotf,nq,nSample+1);
q = [q0 reshape(q,nq,nSample)];
% qtraj = PPTrajectory(pchipDeriv(t_breaks,q,qdot));
% weights = w(nq*nSample+nq+(1:total_sequence_support_vertices));

% I do not trust the info 13 error in SNOPT 


% keyboard;
end

function [f,G] = ik_tmp(w,obj,nq,nT,nF,nG,Qa,Qv,Q,body_ind,body_pos,...
    rotation_type,q0,qdot0,q_nom,velocity_mat,accel_mat,accel_mat_qd0,...
    accel_mat_qdf,num_sequence_support_vertices,num_sample_support_vertices,...
    nWeights,num_quat_des,support_vert_flags,quasiStaticFlag,nonzero_ind_G,...
    contact_states,static_pts_flag,static_pts_ind,num_static_pts,prev_body_ind,...
    prev_pts_ind,shrinkFactor,contact_affs,contact_dists,contact_body_pts_ind,...
    num_contact_body_pts)
% profile on
% [qdot(1);...;qdot(nT-1)] = velocity_mat*[q(0);q(1);...;q(nT)];
% [qddot(0);...;qdot(nT)] =
% accel_mat*[q(0);q(1);...;q(nT)]+accel_mat_qd0*qdot(0)+accel_mat_qdf*qdot(nT);
f = zeros(nF,1);
G = zeros(nG,1);
q = reshape(w(1:nq*(nT-1)),nq,nT-1);
qdotf = w(nq*(nT-1)+(1:nq));
quat_des_all = w(nq*nT+(1:num_quat_des));
if(quasiStaticFlag)
    weights = mat2cell(w(nq*nT+num_quat_des+(1:nWeights)),num_sample_support_vertices,1);
end
qdot = [reshape(velocity_mat*[q0;q(:)],nq,nT-2) qdotf]; %[qdot(1) qdot(2) ... qdot(nT)]
qddot = reshape(accel_mat*[q0;q(:)]+accel_mat_qd0*qdot0+accel_mat_qdf*qdotf,nq,nT); % [qddot(0) qddot(1) ... qddot(nT)]
q_diff = q-q_nom;
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
cum_quat_des = 0;
if(quasiStaticFlag)
    com = cell(1,nT-1);
    dcom = cell(1,nT-1);
    support_vert_pos = cell(1,nT-1);
    dsupport_vert_pos = cell(1,nT-1);
end
x_cache = cell(1,nT-1);
J_cache = cell(1,nT-1);
for i = 1:nT-1
    kinsol = doKinematics(obj,q(:,i));
    if(quasiStaticFlag)
        support_vert_pos{i} = zeros(2,num_sample_support_vertices(i));
        dsupport_vert_pos{i} = zeros(2*num_sample_support_vertices(i),nq);
        total_body_support_vert = 0;
        [com{i},dcom{i}] = getCOM(obj,kinsol);
    end
    x_cache{i} = cell(1,length(body_ind{i}));
    J_cache{i} = cell(1,length(body_ind{i}));
    for j = 1:length(body_ind{i})
        
        if(body_ind{i}(j) == 0)
            if(quasiStaticFlag)
                x = com{i};
                J = dcom{i};
            else
                [x,J] = getCOM(obj,kinsol);
            end
        else
           [x,J] = forwardKin(obj,kinsol,body_ind{i}(j),body_pos{i}{j},rotation_type{i}(j)); 
           x_cache{i}{j} = x;
           J_cache{i}{j} = J;
           if(quasiStaticFlag)
               support_vert_pos{i}(:,total_body_support_vert+(1:num_sequence_support_vertices{i}(j)))...
                   = x(1:2,support_vert_flags{i}{j});
               Jtmp = reshape(J,size(x,1),size(x,2),nq);
               dsupport_vert_pos{i}(total_body_support_vert*2+(1:2*num_sequence_support_vertices{i}(j)),:)...
                   = reshape(Jtmp(1:2,support_vert_flags{i}{j},:),[],nq);
               total_body_support_vert = total_body_support_vert+num_sequence_support_vertices{i}(j);
           end
        end
        [rows,cols] = size(x);
        n = rows*cols;
        if(i ~= 1)
          if(false&&num_static_pts{i}(j) > 0)
              static_pos = x(1:3,static_pts_flag{i}{j});
              prev_j = prev_body_ind{i}(j);
              static_pos_prev = x_cache{i-1}{prev_j}(1:3,prev_pts_ind{i}{j});
              % J is n x nq
              % J_reshaped is 3*nq x num_static_pts{i}{j}
              pos_row = reshape(bsxfun(@plus,[1;2;3],rows*(static_pts_ind{i}{j}-1)),[],1);
              J_pos = J(pos_row,:);
              
              pos_row_prev = reshape(bsxfun(@plus,[1;2;3],rows*(prev_pts_ind{i}{j}-1)),[],1);
              J_pos_prev = J_cache{i-1}{prev_j}(pos_row_prev,:);
              
              f(nf+(1:3*num_static_pts{i}(j))) = static_pos(:) - static_pos_prev(:);
              G(ng+(1:2*numel(J_pos))) = [-J_pos_prev(:); J_pos(:)];
              nf = nf+3*num_static_pts{i}(j);
              ng = ng+2*numel(J_pos);
          end
        end
        if(rotation_type{i}(j) ==0|| rotation_type{i}(j) == 1)
          f(nf+(1:n)) = x(:);
          G(ng+(1:numel(J))) = J(:);
          nf = nf+n;
          ng = ng+numel(J);
        elseif(rotation_type{i}(j) == 2)
          f(nf+(1:3*cols)) = reshape(x(1:3,:),[],1);
          quat = x(4:7,1);
          quat_des = quat_des_all(cum_quat_des+(1:4));
          f(nf+3*cols+1) = sum(quat.*quat_des,1)^2;
          f(nf+3*cols+1+1) = sum(quat_des.*quat_des,1);
          J_rows = bsxfun(@plus,(1:7)',(0:(cols-1))*7);
          pos_rows = reshape(J_rows(1:3,:),[],1);
          dquatdq = J(4:7,:);
          G(ng+(1:3*cols*nq)) = reshape(J(pos_rows,:),[],1);
          G(ng+3*cols*nq+(1:(nq+4))) = (quat'*quat_des)*[2*quat_des'*dquatdq 2*quat'];
          G(ng+3*cols*nq+(nq+4)+(1:4)) = 2*quat_des;
          nf = nf+3*cols+2;
          ng = ng+3*cols*nq+(nq+4)+4;
          cum_quat_des = cum_quat_des+4;
        end
        % contact affordance residues
        if(body_ind{i}(j) ~=0)
            for aff_ind = 1:length(contact_affs{i}{j})
                if(num_contact_body_pts{i}{j}(aff_ind)>0)
                    aff = contact_affs{i}{j}{aff_ind};
                    x_contact = x(1:3,contact_body_pts_ind{i}{j}{aff_ind});
                    J_contact_rows = reshape(bsxfun(@plus,[1;2;3],rows*(reshape(contact_body_pts_ind{i}{j}{aff_ind},1,[])-1)),[],1);
                    J_contact = J(J_contact_rows,:);
                    [residue,dRes] = aff.residualsPts2Aff(q(:,i),x_contact);
                    num_residue = numel(residue);
                    if(num_residue ~= 1 || residue ~= -1)
                      f(nf+(1:num_residue)) = residue(:);
                      dResdq = dRes(:,1:nq)+dRes(:,nq+1:end)*J_contact;
                      G(ng+(1:numel(dResdq))) = reshape(dResdq,[],1);
                      nf = nf+num_residue;
                      ng = ng+numel(dResdq);
                    end
                end
            end
        end
    end
end

% shrink the support polygon by the shrinkFactor

if(quasiStaticFlag)
    for i = 1:nT-1
        support_vert_center = mean(support_vert_pos{i},2);
        dsupport_vert_center = [mean(dsupport_vert_pos{i}(1:2:end,:),1);mean(dsupport_vert_pos{i}(2:2:end,:),1)];
        support_vert_pos{i} = shrinkFactor*support_vert_pos{i}+(1-shrinkFactor)*bsxfun(@times,support_vert_center,ones(1,size(support_vert_pos{i},2)));
        dsupport_vert_pos{i} = shrinkFactor*dsupport_vert_pos{i}+(1-shrinkFactor)*repmat(dsupport_vert_center,size(support_vert_pos{i},2),1);
        f(nf+(1:2)) = support_vert_pos{i}*weights{i}-com{i}(1:2);
        f(nf+3) = 0;
        G(ng+(1:2*(nq+num_sample_support_vertices(i)))) = [reshape([weights{i}'*dsupport_vert_pos{i}(1:2:end,:)-dcom{i}(1,:);...
            weights{i}'*dsupport_vert_pos{i}(2:2:end,:)-dcom{i}(2,:)],[],1);...
            reshape(support_vert_pos{i},[],1)];
        nf = nf+3;
        ng = ng+2*(nq+num_sample_support_vertices(i));
    end
    %figure(7);
    %plot(support_vert_pos{7}(1,:),support_vert_pos{7}(2,:),'ob',com{7}(1),com{7}(2),'sr');
    %axis equal
    %hold on
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
