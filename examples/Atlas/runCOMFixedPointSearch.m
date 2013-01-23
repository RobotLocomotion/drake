function [xstar,ustar,zstar] = runCOMFixedPointSearch()

planar = false;
runsim = true; % run 1s sim with constant input after search

s = 'urdf/atlas_minimal_contact.urdf';

options.floating = true;
if (planar)
    options.view = 'right';
    options.twoD = true;
    dim = 2;
else
    dim = 3;
end

dt = 0.001;
r = TimeSteppingRigidBodyManipulator(s,dt,options);
r = setSimulinkParam(r,'MinStep','0.0001');
v = r.constructVisualizer;
v.display_dt = dt*25;

x0 = Point(r.getStateFrame);

if (planar)
    x0.pelvis_z = 0.920000;
    x0.pelvis_p = -0.100000;
    x0.back_mby = 0.050000;
    x0.l_leg_kny = 0.300000;
    x0.l_leg_uay = -0.200000;
    x0.r_leg_kny = 0.300000;
    x0.r_leg_uay = -0.200000;
else
    x0.pelvis_z = 0.920000;
    x0.pelvis_pitch = -0.050000;
    x0.back_mby = 0.075000;
    x0.l_arm_elx = 1.400000;
    x0.l_arm_ely = 1.570000;
    x0.l_arm_shx = -1.100000;
    x0.l_arm_usy = 0.200000;
    x0.l_leg_kny = 0.400000;
    x0.l_leg_lhy = -0.150000;
    x0.l_leg_uay = -0.1750000;

    x0.r_arm_elx = -1.400000;
    x0.r_arm_ely = 1.570000;
    x0.r_arm_shx = 1.100000;
    x0.r_arm_usy = -0.200000;
    x0.r_leg_kny = 0.400000;
    x0.r_leg_lhy = -0.150000;
    x0.r_leg_uay = -0.1750000;
end
%x0 = resolveConstraints(r,double(x0));
u0 = Point(r.getInputFrame());
v.draw(0,double(x0));
[xstar,ustar,zstar] = computeFixedPoint(r,double(x0),double(u0),v);

if (runsim)
    % simulate fixed point 
    T = 1.0; % sec
    
    c = ConstOrPassthroughSystem(ustar);
    c = setOutputFrame(c,r.getInputFrame);
    sys = cascade(c,r); 
    if (0)
        traj = simulate(sys,[0 T],xstar);
        playback(v,traj);
    else    
        s = warning('off','Drake:DrakeSystem:UnsupportedSampleTime');  % we are knowingly breaking out to a simulink model with the cascade on the following line.
        sys = cascade(sys,v);
        warning(s);
        simulate(sys,[0 T],xstar);
    end
end

function [xstar,ustar,zstar] = computeFixedPoint(r,x0,u0,v)
    nx = r.getNumStates();
    nq = nx/2;
    nu = r.getNumInputs();
    nz = r.getNumContacts()*dim;
    z0 = zeros(nz,1);
    q0 = x0(1:nq);
    
    % take initial state guess as desired nominal pose
    qstar = x0(1:nq);
    
    problem.x0 = [q0;u0;z0];
    problem.objective = @(quz) myobj(quz);
    problem.nonlcon = @(quz) mycon(quz);
    problem.solver = 'fmincon';
    %problem.options=optimset('DerivativeCheck','on','GradObj','on','GradConstr','on','Algorithm','interior-point','Display','iter','OutputFcn',@drawme,'TolX',1e-12);
    problem.options=optimset('GradObj','on','GradConstr','on','Algorithm','interior-point','Display','iter','OutputFcn',@drawme,'TolX',1e-6,'MaxFunEvals',500);
    
    lb_z = -1e6*ones(nz,1);
    lb_z(dim:dim:end) = 0; % normal forces must be >=0
    ub_z = 1e6*ones(nz,1);
    [jl_min,jl_max] = r.getJointLimits();
    if (planar)
        problem.lb = [jl_min+0.01; r.umin; lb_z];
        problem.ub = [jl_max-0.01; r.umax; ub_z];
    else
        % force search to be close to starting position
        problem.lb = [max(q0-0.05,jl_min+0.01); r.umin; lb_z];
        problem.ub = [min(q0+0.05,jl_max-0.01); r.umax; ub_z];
    end
    problem.lb(dim) = 0.8; % above the ground
    
    % fix foot starting position
    foot_pos = r.contactPositions(x0(1:nq));
    foot_pos(end,:) = 0; % we want the feet to be on the ground
    min_xf = min(foot_pos(1,:));
    max_xf = max(foot_pos(1,:));
    if (planar)
        com_des = [mean([max_xf,min_xf]); 0.6];
    else
        min_yf = min(foot_pos(2,:));
        max_yf = max(foot_pos(2,:));
        com_des = [mean([max_xf,min_xf]); mean([max_yf,min_yf]); 1.0];
%         % debug
%         figure(124);
%         hold on;
%         plot([min_xf min_xf max_xf max_xf],[min_yf max_yf min_yf max_yf],'bx');
%         plot(com_des(1),com_des(2),'go');
%         hold off;
    end

    foot_pos = reshape(foot_pos,nz,1);

    [quz_sol,~,exitflag] = fmincon(problem);
    success=(exitflag==1);
    xstar = [quz_sol(1:nq); zeros(nq,1)];
    ustar = quz_sol(nq+(1:nu));
    zstar = quz_sol(nq+nu+(1:nz));
%     if (~success)
%         error('failed to find fixed point');
%     end
    
    function stop=drawme(quz,optimValues,state)
        stop=false;
        v.draw(0,[quz(1:nq); zeros(nq,1)]);
        
        if (~planar)
            figure(66);
            plot([min_xf min_xf max_xf max_xf],[min_yf max_yf min_yf max_yf],'bx','MarkerSize',10);
            hold on;
            plot(com_des(1),com_des(2),'go','MarkerSize',10);
            cm = r.getCOM(quz(1:nq));
            plot(cm(1),cm(2),'ro','MarkerSize',10);
            hold off;
        end
    end
    
    function [f,df] = myobj(quz)
        q=quz(1:nq);
        [cm,J] = r.getCOM(q);
        W = eye(dim);
        W(end,end) = 0; % ignore z-component
        w_q = 0.1;
        f = (com_des-cm)'*W*(com_des-cm) + w_q*(qstar-q)'*(qstar-q);
        df = [-2*(com_des-cm)'*W*J-2*w_q*(qstar-q)',zeros(1,nu+nz)];
    end

    function [c,ceq,GC,GCeq] = mycon(quz)
        q=quz(1:nq);
        u=quz(nq+(1:nu));
        z=quz(nq+nu+(1:nz));

        [~,C,B,~,dC,~] = r.manipulatorDynamics(q,zeros(nq,1));
        [cpos,J,dJ] = r.contactPositions(q);
        cpos = reshape(cpos,nz,1);
        
        % ignore friction constraints for now
        c = 0;
        GC = zeros(nq+nu+nz,1); 
        
        dJz = zeros(nq,nq);
        for i=1:nq
            dJz(:,i) = dJ(:,(i-1)*nq+1:i*nq)'*z;
        end
        
        ceq = [C-B*u-J'*z; cpos-foot_pos];
        GCeq = [[dC(1:nq,1:nq)-dJz,-B,-J']',[J'; zeros(nu+nz,length(cpos))]]; 
end
end

end

