function [xstar,ustar,zstar] = runFixedPointSearch()

planar = false;
runsim = true; % run 1s sim with constant input after search

%s = 'urdf/simple_atlas_minimal_contact.urdf';
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
v.display_dt = dt*10;

x0 = Point(r.getStateFrame);
x0 = resolveConstraints(r,double(x0));
u0 = Point(r.getInputFrame);

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
    
    problem.x0 = [q0;u0;z0];
    problem.objective = @(quz) 0; % feasibility problem
    problem.nonlcon = @(quz) mycon(quz);
    problem.solver = 'fmincon';
    %problem.options=optimset('DerivativeCheck','on','GradConstr','on','Algorithm','interior-point','Display','iter','OutputFcn',@drawme,'TolX',1e-14,'MaxFunEvals',15000);
    problem.options=optimset('GradConstr','on','Algorithm','interior-point','Display','iter','OutputFcn',@drawme,'TolX',1e-14,'MaxFunEvals',15000);

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
    %problem.lb(2) = 0.0; % body z

    [quz_sol,~,exitflag] = fmincon(problem);
    success=(exitflag==1);
    xstar = [quz_sol(1:nq); zeros(nq,1)];
    ustar = quz_sol(nq+(1:nu));
    zstar = quz_sol(nq+nu+(1:nz));
    if (~success)
        error('failed to find fixed point');
    end

    function stop=drawme(quz,optimValues,state)
        stop=false;
        v.draw(0,[quz(1:nq); zeros(nq,1)]);
    end

    function [c,ceq,GC,GCeq] = mycon(quz)
        q=quz(1:nq);
        u=quz(nq+(1:nu));
        z=quz(nq+nu+(1:nz));

        [~,C,B,~,dC,~] = r.manipulatorDynamics(q,zeros(nq,1));
        [phiC,JC] = r.contactConstraints(q);
        [~,J,dJ] = r.contactPositions(q);
        
        % ignore friction constraints for now
        c = 0;
        GC = zeros(nq+nu+nz,1); 
        
        dJz = zeros(nq,nq);
        for i=1:nq
            dJz(:,i) = dJ(:,(i-1)*nq+1:i*nq)'*z;
        end
        
       ceq = [C-B*u-J'*z; phiC];
       GCeq = [[dC(1:nq,1:nq)-dJz,-B,-J']',[JC'; zeros(nu+nz,length(phiC))]]; 
    end
end

end

