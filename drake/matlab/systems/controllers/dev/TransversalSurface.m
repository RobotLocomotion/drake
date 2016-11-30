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
            z_t = z_t/norm(z_t);
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
        
        function h=plotSurface(obj,xtraj,rad,plotdims)
          typecheck(xtraj,'Trajectory');
          if (nargin<3) rad=.1; end
          typecheck(rad,'double');
          if (nargin<4) plotdims=[1 2]; end
          
          nX=size(xtraj,1);
          xperp=rad*ones(nX-1,1); 
          
          h=[];
          for t=xtraj.getBreaks();
            x0=xtraj.eval(t); Pi=obj.getPi(t);
            y1=x0+Pi'*xperp;
            y2=x0-Pi'*xperp;
            h=[h,line([y1(plotdims(1));y2(plotdims(1))],[y1(plotdims(2));y2(plotdims(2))],'Color',[1 0 0])];
          end
        end
        
        function h=plotFunnel(obj,Vtraj,xtraj,plotdims,options)
          typecheck(Vtraj,'PolynomialTrajectory');
          if (nargin<4) plotdims=[1 2]; end
          if (nargin<5) options=struct(); end
          if (~isfield(options,'color')) options.color=.7*[1 1 1]; end
          rho=1;
          
          ts = Vtraj.getBreaks(); V=Vtraj.eval(ts(1));
          p_xp=decomp(V);
          if (deg(V,p_xp)>2) error('only works for quadratics so far'); end
          
          hold on;
          view(0,90);
          
          function x=getLevel(t)
            x0=xtraj.eval(t);
            V=Vtraj.eval(t);
            Pi=obj.getPi(t);

            V=subss(V,p_xp,p_xp+Pi*x0);  % put it into relative frame

            if (length(p_xp)==1)
              a=double(.5*diff(diff(V,p_xp),p_xp));
              r=sqrt(1/a);
              x=x0(:,[1 1])+Pi'*[r,-r];
            else
              error('not implemented yet'); 
            end
          end
          
          for i=length(ts)-1:-1:1
            xa=getLevel(ts(i));
            xb=getLevel(ts(i+1)-eps);
            x=[xa,fliplr(xb)];
            k = convhull(x(1,:),x(2,:));
            fill3(x(1,k),x(2,k),repmat(0,1,length(k)),options.color,'LineStyle','none');
            plot3(x(1,k),x(2,k),repmat(-.1,1,length(k)),'k','LineWidth',5);
%            x=[x,x(:,1)];
%            fill3(x(1,:),x(2,:),repmat(0,1,size(x,2)),options.color,'LineStyle','none');
%            plot3(x(1,:),x(2,:),repmat(-.1,1,size(x,2)),'k','LineWidth',5);
            
            plot3(xb(1,:),xb(2,:),repmat(.1,1,size(xb,2)),'k','LineWidth',.5);
          end
          
          % draw level-set at the beginning
          plot3(xa(1,:),xa(2,:),repmat(.1,1,size(xa,2)),'k','LineWidth',2);
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
        
        pnorm = 4;
        
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
        
        w = median(zs,2)/norm(median(zs,2));
        
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
        delta = 1e-6;
        
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
          pnorm = 10;
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
          smooth_reg =0.01;
        end
        if (nargin<8)
          orthog_reg = 20;
        end
        thet=linspace(0,pi,N);
        shape = (sin(thet)).^2;
        
        if (1) %verbose)
          options = optimset('MaxFunEvals',2e5,'MaxIter',2e3,'TolFun',0.1,'Display','off','Algorithm','interior-point','UseParallel', 'always');
        else
          options = optimset('MaxFunEvals',2e5,'MaxIter',2e3,'TolFun',1e-4*zcost(zs0),'Display','off','Algorithm','interior-point');
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
            
            
            
