classdef TransversalSurface
% Transversal surfaces for orbital stabilization
% irm@mit.edu
    
    methods
        function obj=TransversalSurface(z,w)
            if (isempty(z)) error('must run designSurfaceNormal first'); end
            typecheck(z,'Trajectory');
            w = w/norm(w);
            W = null(w');
            
            obj.eta = [w W];
            obj.w = w;
            obj.z = z;
            obj.zdot = fnder(z);
        end
        function [Pi Pidot] = getPi(obj, t)
        
            z_t = obj.z.eval(t);
            dz_t = obj.zdot.eval(t);
            eta_t = obj.eta;
            n = obj.z.dim;
            
            if (n==2)
                Pi = [-z_t(2) z_t(1)];
                Pidot = [-dz_t(2) dz_t(1)];
            else
            
                xi = zeros(n,n);
                xidot =  zeros(n,n);
                
                xi(:,1) = z_t;
                xidot(:,1) = dz_t;
                
                for j = 2:n
                    xi(:,j) = eta_t(:,j)-(eta_t(:,j)'*z_t)/(1+eta_t(:,1)'*z_t).*(eta_t(:,1)+z_t);
                    xidot(:,j) = -(((eta_t(:,j)'*dz_t).*(eta_t(:,1)+z_t)+(eta_t(:,j)'*z_t).*dz_t).*(1+eta_t(:,1)'*z_t) ...
                        -(eta_t(:,j)'*z_t).*(eta_t(:,1)+z_t).*(eta_t(:,1)'*dz_t))/(1+eta_t(:,1)'*z_t)^2;
                end
                
                Pi = xi(:,2:end)';
                Pidot = xidot(:,2:end)';
            end
        end
        
    end
    properties
        z = [];
        zdot = [];
        eta = [];
        w = [];
    end
    

    methods (Static=true)

      function TransSurf = design(plant, w, xtraj, utraj, Nsteps,optimize,init_surf_normal, final_surf_normal)
        %Design transversal surfaces
        % Documentation to come...
        %
        % irm@mit.edu
        
        pnorm = 10;
        
        tspan = utraj.getBreaks();
        nX = xtraj.dim;
        ftraj = FunctionHandleTrajectory(@(t)plant.dynamics(t,xtraj.eval(t), utraj.eval(t)), nX, tspan);
        
        
        ts = linspace(min(tspan),max(tspan),Nsteps); % linspace of time samples to optimize. Could just use breaks?
        
        % fs = ftraj.eval(ts)  % -- needs vectorized dynamics evaluation
        
        fs = zeros(nX,length(ts));
        for k = 1:length(ts)
          fs(:,k) = ftraj.eval(ts(k));
        end
        
        
        fsnormalized = fs./repmat((sqrt(sum(fs.^2))),nX,1);
        
        if (optimize)
          if(nargin<7)
            %If no initial and final surface constraint, assume smooth periodic
            [zs, ~] = z_opt(fs,ts, pnorm,fsnormalized);
          else
            % Make sure surface normals are pointing the right direction (acute angle
            % with fs)
            if (fs(1)'*init_surf_normal<0)
              init_surf_normal = -init_surf_normal;
            end
            if (fs(end)'*final_surf_normal<0)
              final_surf_normal = -final_surf_normal;
            end
            
            
            [zs, ~] = TransversalSurface.z_opt(fs,ts, pnorm,fsnormalized, init_surf_normal,final_surf_normal);
          end
        else
          zs = fsnormalized;
        end
        
        zpp = spline(ts,zs);
        
        
        TransSurf = TransversalSurface(PPTrajectory(zpp),w);
      end
      
      function [zs, fval] = z_opt(fs,ts, pnorm,zguess,zinit,zfinal,smooth_reg,orthog_reg, verbose)
        % zs = z_opt(fs,pnorm,zguess,zinit,zfinal)
        %
        % Basic optimization of z(tau), as a sequence of vectors.
        % Inputs:
        %   - fs: f(x_star(tau)) as a sequence of vectors
        %   - ts: corresponding sequence of times
        %   - pnorm: which l^p norm to optimize over (Default = 10).
        %   - zguess: initial guess for zs. (Default = fs/norm(fs).)
        %   - zinit, zfinal: enpoint constraints for z(tau), for switching surfaces
        %               if not given, assume periodic: zinit=zfinal.
        %   - smooth_reg: L2 smoothness regulariser (optional positive number, default 0)
        %   - orthog_reg: orthogonality regulariser (optional positive number, default 0)
        %   - verbose: print out details of optimization (optional)
        %
        %  irm@mit.edu
        
        % Transversality constraint, should be a small positive number
        delta = 1e-1;
        
        n = size(fs,1);
        N = size(fs,2);
        
        if (nargin<9)
          verbose = 0;
        end
        
        if (verbose)
          tic;
          disp(['Optimizing transversal surfaces with ' num2str(N) ' time points and l_' num2str(pnorm) ' norm.']);
        end
        
        % If no pnorm, assume pnorm=10
        if (nargin<3)
          pnorm = 2;
        end
        
        fsnormalized = fs./repmat((sqrt(sum(fs.^2))),n,1);
        
        % If no initial guess given, take from normalised fs
        if (nargin<4)
          
          zs0 = fsnormalized;
        else
          zs0 = zguess;
        end
        
        %If no endpoint constraints, assume periodic
        if (nargin<5)
          periodic = 1;
        else
          periodic = 0;
        end
        if (nargin<7)
          smooth_reg = 1e-3;
        end
        if (nargin<8)
          orthog_reg = 0;
        end
        thet=linspace(0,pi,N);
        shape = (sin(thet)).^2;
        
        if (verbose)
          options = optimset('MaxFunEvals',2e5,'MaxIter',2e3,'TolFun',1e-3*zcost(zs0),'Display','iter','Algorithm','sqp');
        else
          options = optimset('MaxFunEvals',2e5,'MaxIter',2e3,'TolFun',1e-3*zcost(zs0),'Display','off','Algorithm','sqp');
        end
        
        
        [zs, fval] = fmincon(@zcost,zs0,[],[],[],[],[],[],@zconst,options);
        
        if (verbose)
          opt_time = toc;
          disp(['Completed in ' num2str(opt_time) ' sec.']);
        end
        
        function J = zcost(zs)
          %zdotf = sum(zs.*fs);
          
          
          dzdt = gradient(zs)./repmat(gradient(ts),n,1);
          ddzdt2 = gradient(dzdt)./repmat(gradient(ts),n,1);
          zcosts = sqrt(sum(dzdt.^2))./(sum(zs.*fs));
          J = norm(shape.*zcosts,pnorm)^2+smooth_reg*(sum(sum(ddzdt2.^2)))+orthog_reg/(sum(sum(zs.*fsnormalized)));
        end
        
        function [c ceq] = zconst(zs)
          constr = sum(zs.^2)-1;
          if (periodic)
            ceq = [constr';(zs(:,1)-zs(:,end))];
          else
            ceq = [constr';(zs(:,1)-zinit);(zs(:,end)-zfinal)];
          end
          c = -sum(zs.*fsnormalized)+delta;
        end
      end
      
      
    end
end        
            
            
            