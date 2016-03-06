function [ctrans, TransSurf, V] = transverseLQR(p,w, xtraj, utraj,Qtraj, Rtraj, Qfinal, Qjump,periodic,TransSurf)
% Generates a hybrid solution of transverse TVLQR
%
% irm@mit.edu

if nargin<10
    generate_surfaces = 1;
else
    generate_surfaces = 0;
end

N = length(p.modes);
Qf = Qfinal;

for i = N:-1:1  %N = number of continuous phases
    
    % Extract plant and trajectories for this phase
    
    pi = p.modes{i};
    
    xtraji = xtraj.traj{i}.trajs{2};
    utraji = utraj.traj{i};
    
    tspan = xtraji.tspan;
    
    if (isa(Qtraj{i},'double'))
        Qi = PPTrajectory(zoh(tspan([1,end]),repmat(Qtraj{i},[1 1 2])));
    end
    if (isa(Rtraj{i},'double'))
        Ri = PPTrajectory(zoh(tspan([1,end]),repmat(Rtraj{i},[1 1 2])));
    end
    
    ti = tspan(1);
    tf = tspan(2);
    xdoti = pi.dynamics(ti,xtraji.eval(ti), utraji.eval(ti));
    if (norm(xdoti)<1e-5)
        ti = 1e-3;
        xdoti = pi.dynamics(ti,xtraji.eval(ti), utraji.eval(ti));
    end
    
    if ((i>1)||periodic)
        if (i>1)
            j = i-1;
        else
            j = N;
        end

        tspanprev = xtraj.traj{j}.trajs{2}.tspan;
        Tprev = tspanprev(2);
        mode = xtraj.traj{j}.trajs{1}.eval(Tprev-0.01);
        xT = xtraj.traj{j}.trajs{2}.eval(Tprev);
        uT = utraj.traj{j}.eval(Tprev);

        zcs = zeroCrossings(p,Tprev,[mode;xT],uT);
        active_id = find(zcs<1e-6);
        if (isempty(active_id)), error('no active guard at final time');end
        if (length(active_id)>1), error('multiple guards tripped at the same time.  behavior is undefined.  consider reducing the step size'); end
        [ft,modeSwitch,status,df] = geval(3, p.transition{mode}{active_id},p, mode,Tprev, xT, uT);

        F = df(:,3:(2+size(xT,1)));
        dgPrev = impactNormalVec(p,j,xtraj,utraj);
        
        %************
        testDir = dot(dgPrev, xdoti);
        if(testDir < 0)
            dgPrev = -dgPrev;
        end
        %TODO: check that the normal is oriented correctly

        init_surf_normal = F'*dgPrev;  %adjoint update for surface normal
    else
        init_surf_normal = xdoti/norm(xdoti);
    end
    
    
    if generate_surfaces
        if(i==N && ~periodic)
            final_surf_normal = pi.dynamics(ti,xtraji.eval(tf), utraji.eval(tf));
        else
            xdotf = pi.dynamics(ti,xtraji.eval(tf), utraji.eval(tf));
            final_surf_normal = impactNormalVec(p,i,xtraj,utraj);
            %TODO: check that the normal is oriented correctly
            %****************
            testDir = dot(final_surf_normal, xdotf);
            if(testDir < 0)
                final_surf_normal = -final_surf_normal;
            end
        end
        
        init_surf_normal = init_surf_normal/norm(init_surf_normal);
        final_surf_normal = final_surf_normal/norm(final_surf_normal);
        
        %TODO: This should be scaled by how long the trajectory is
        Nsteps = 50;
        
        fprintf('Creating transversal surfaces for segment %i \n',i)
        TransSurf{i} = TransversalSurface.design(pi,w, xtraji, utraji, Nsteps,1,init_surf_normal, final_surf_normal);
    end
    fprintf('Creating transverse control for segment %i \n',i)
    
    if (i<N) %If this wasn't the segement at the end then we need to do a 
        %jump to figure out how we hook up to the next one in time
        PiPlus = TransSurf{i+1}.getPi(TransSurf{i+1}.z.tspan(1));
        PiMinus = TransSurf{i}.getPi(TransSurf{i}.z.tspan(end));
        S = ctrans{i+1}.S.eval(ctrans{i+1}.S.tspan(1));
        Qf = riccatiJump(S,PiPlus*F*PiMinus',PiMinus*Qjump{i}*PiMinus'); 
    end
    
    [ctrans{i},V{i}] = transverseLQR(pi,xtraji,utraji,Qi,Ri,Qf,TransSurf{i});
end
    
end


function Pminus = riccatiJump(P,F,Q)
% Jump riccati equation update with x^+ = Fx^-, and incremental cost Q.
% Currently no discrete control.

Pminus = F'*P*F+Q;

end


function dg = impactNormalVec(p,i,xtraj,utraj)

xtraji = xtraj.traj{i}.trajs{2};
utraji = utraj.traj{i};
t = xtraji.tspan(end);
uT = utraji.eval(xtraji.tspan(end));
xT = xtraji.eval(xtraji.tspan(end));
zcs = zeroCrossings(p,t,[i;xT],uT);
active_id = find(zcs<1e-6);
if (isempty(active_id)), error('no active guard at final time');end % Just gets first guard, hack for compass gait
if (length(active_id)>1), error('multiple guards tripped at the same time.  behavior is undefined.  consider reducing the step size'); end

[g, dg] = geval(p.guard{i}{active_id},p,t,xT,uT);   %Need dg to be the normal vector to the guard. 
                                         %should be gradient of smooth guard function
                                         
dg = dg(2:end-1)'; %cut off the t and u gradients
    
end
