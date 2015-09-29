classdef PolytopicSystem < DrakeSystem
% defines a piecewise smooth system with piecewise segments defined on a
% polytope.
% Note: the current representation using Ax>=b is simple and relatively efficient at
% runtime.  the major inefficiency is that i currently require subsystems
% to be defined even for regions that cannot exist (e.g. for the case where
% the input is saturated on *both* the max and min values.  
 
  
  properties
    A        % Ax=b defines the boundaries of the polytopic regions. 
    b
    subsys  % cell array of smooth systems
  end

  methods
    function obj=PolytopicSystem(A,b,subsys)
      obj=obj@DrakeSystem(0,0,0,0,false,true);
      
      % make sure all are consistent
      [m,n]=size(A);
      sizecheck(b,[m,1]);
      typecheck(subsys,'cell');
      sizecheck(subsys,2^m);

      typecheck(subsys{1},'SmoothRobotLibSystem');
      
      % copy properties from subsystem 1
      
      if (subsys{1}.getNumStates() ~= n) error('subsystem 1 must have the same number of states as A has columns'); end

%      for p = properties('RobotLibSystem')
%        disp(['setting property ', p]);
%        obj = set(obj,p,get(subsys{1},p));
%      end
      obj = obj.setNumContStates(subsys{1}.getNumContStates());
      obj = obj.setNumDiscStates(subsys{1}.getNumDiscStates());
      obj = obj.setNumInputs(subsys{1}.getNumInputs());
      obj = obj.setNumOutputs(subsys{1}.getNumOutputs());
      
      % make sure all sub systems are smooth, and have the same number of
      % inputs, states, and outputs.
      for i=2:length(subsys)
        typecheck(subsys{i},'DrakeSystem');
        
%        for p = properties('DrakeSystem')
%          if (get(obj,p)~=get(subsys{i},p))
%            error(['subsystem property ',p,' must match subsystem 1']);
%          end
%        end

        if (getNumContStates(subsys{i}) ~= getNumContStates(obj)) error(['sub_sys ',num2str(i),' must have the same number of continuous states as sub system 1']); end
        if (getNumDiscStates(subsys{i}) ~= getNumDiscStates(obj)) error(['sub_sys ',num2str(i),' must have the same number of discrete states as sub system 1']); end
        if (getNumInputs(subsys{i}) ~= getNumInputs(obj)) error(['sub_sys ',num2str(i),' must have the same number of inputs as sub system 1']); end
        if (getNumOutputs(subsys{i}) ~= getNumOutputs(obj)) error(['sub_sys ',num2str(i),' must have the same number of outputs as sub system 1']); end
      end
      
      obj.A=A;
      obj.b=b;
      obj.subsys=subsys;
    end

    function x0 = getInitialState(obj)
      if ~isempty(obj.initial_state)
        x0 = obj.initial_state;
        return;
      end
      x0 = getInitialState(obj.subsys{1});
    end

    function zcs = zeroCrossings(obj,t,x,u)
      zcs = obj.A*x-obj.b;
    end

    function xcdot = dynamics(obj,t,x,u)
      ind = polytopeIndex(obj,x);
      xcdot = obj.subsys{ind}.dynamics(t,x,u);
    end
    
    function xdn = update(obj,t,x,u)
      ind = polytopeIndex(obj,x);
      xdn = obj.subsys{ind}.update(t,x,u);
    end
    
    function y = output(obj,t,x,u)
      ind = polytopeIndex(obj,x);
      y = obj.subsys{ind}.output(t,x,u);
    end
    
    function ind = polytopeIndex(obj,x)
      % get the index of the system which is active in the state x
      bin=obj.A*x>=obj.b;
      ind = 1+bin2dec(sprintf('%d',flipud(bin)));
    end
    
    function h = plotRegions(obj,x0,plotdims)
      % adds a slice of the polytopic regions to a current plots

      gcf;  % will make a figure if it doesn't exist
      
      A=obj.A; b=obj.b;
      if (obj.getNumStates()>2) 
        % set x(no_plot_dims)=x0
        no_plot_dims=1:length(x0);  no_plot_dims(plot_dims)=[];
        b = b-A(:,no_plot_dims)*x0(no_plot_dims);
        A = A(:,plot_dims);
      end
      
      v=axis;
      for i=1:size(A,1)  % draw a line for each half-plane
        % find the intersections of each line with the box from the current axes.
        x=[v(1:2),(b(i)-A(i,2)*v(3:4))/A(i,1)];
        y=[(b(i)-A(i,1)*v(1:2))/A(i,2),v(3:4)];
        toplot = (x>=v(1)) & (x<=v(2)) & (y>=v(3)) & (y<=v(4));
        line(x(toplot),y(toplot),'Color','k');
      end
      
      opt=optimset('Display','off');
      for i=1:length(obj.subsys)
        bb=sscanf(dec2bin(i-1),'%1d');bb=[zeros(length(b)-length(bb),1);bb];
        flip=-diag(2*flipud(bb)-1);
        [xtext,~,flag]=linprog(zeros(2,1),flip*A,flip*b,[],[],v([1;3]),v([2;4]),[],opt);
        if (flag>0)
          text(xtext(1),xtext(2),['$f_',num2str(i),'$'],'Interpreter','latex');
        end
      end
            
    end
  end

  methods (Static)
    function obj = constructFeedbackWSaturations(sys1,sys2)
      % construct feedback system, or throw error if I can't do it.

      function y=doubleSafe(x)
        y=double(x);
        if (~isa(y,'double')) error('double failed'); end
      end
      
      A=[]; b=[]; subsys={};

      umin1=sys1.umin; umax1=sys1.umax;
      sys1=setInputLimits(sys1,-inf,inf);

      umin2=sys2.umin; umax2=sys2.umax;
      sys2=setInputLimits(sys2,-inf,inf);
      
      subsys={FeedbackSystem(sys1,sys2)};
      pt = msspoly('t',1);
      px = msspoly('x',getNumStates(subsys{1}));
      [px1,px2]=subsys{1}.decodeX(px);
      
      if (any(~isinf([umin1; umax1])))
        % found saturation.  check if the saturation is a linear 
        % (or affine) function of the state
        
        try 
          if (~sys1.isDirectFeedthrough()) % have to do sys1 first
            py1=output(sys1,pt,px1);  % doesn't need u
            py2=output(sys2,pt,px2,py1);
          else % do sys2 first
            py2=output(sys2,pt,px2);  % doesn't need u
          end
        catch
          error('Drake:PolytopicSystem:NotSupported','can''t parse through with msspoly');
          % note: this will still trip if there is something non-polynomial
          % on an output which is not saturated.  
          % could also fail in sys1.output, even if py2 does not depend py1.
          % i could definitely do it better. but this is good enough for now...
        end
        if (deg(py2,pt)>0 || deg(py2,px)>1)
          error('Drake:PolytopicSystem:NotSupported','output of system 2 must be affine in x and independent of t and u');
        end
        y0=doubleSafe(subs(py2,px,double(0*px)));
        dydx=doubleSafe(diff(py2,px));
        
        % y0+dydx*x<=umin
        inds=find(umin1~=-inf);
        A = [A;-dydx(inds,:)];
        b = [b;y0(inds)-umin1(inds)];
        for i=inds
          y=nan*umin1;  y(i)=umin1(i); 
          c=ConstOrPassthroughSystem(y);
          for j=1:length(subsys)
            subsys={subsys{:},feedback(cascade(c,subsys{j}.sys1),subsys{j}.sys2)};
          end
        end
        
        % y0+dydx*x>=umax
        inds=find(umax1~=inf);
        A = [A;dydx(inds,:)];
        b = [b;umax1(inds)-y0(inds)];
        for i=inds
          y=nan*umax1;  y(i)=umax1(i); 
          c=ConstOrPassthroughSystem(y);
          for j=1:length(subsys)
            subsys={subsys{:},feedback(cascade(c,subsys{j}.sys1),subsys{j}.sys2)};
          end
        end

        % note (again) the inefficiency here, that I am making a subsystem
        % for the case when the input is saturated *both* at umin and umax,
        % which can never happen.  
        % specifically, I'm creating 2^(2*m) regions instead of 3^m
        % regions, where m is the number of saturating inputs.
        % ok for now (space is cheap).
        
      end

      if (any(~isinf([umin2; umax2])))
        try 
          if (~sys1.isDirectFeedthrough()) % have to do sys1 first
            py1=output(sys1,pt,px1);  % doesn't need u
          else % do sys2 first
            py2=output(sys2,pt,px2);  % doesn't need u
            py1=output(sys1,pt,px1,py2);
          end
        catch
          error('RobotLib:PolytopicRobotLibSystem:NotSupported','can''t parse through with msspoly');
          % note: this will still trip if there is something non-polynomial
          % on an output which is not saturated.  
          % could also fail in sys1.output, even if py2 does not depend py1.
          % i could definitely do it better. but this is good enough for now...
        end
        if (deg(py1,pt)>0 || deg(py1,px)>1)
          error('RobotLib:PolytopicRobotLibSystem:NotSupported','output of system 2 must be affine in x and independent of t and u');
        end
        y0=doubleSafe(subs(py1,px,double(0*px)));
        dydx=doubleSafe(diff(py1,px));
        
        % y0+dydx*x<=umin
        inds=find(umin2~=-inf);
        A = [A;-dydx(inds,:)];
        b = [b;y0(inds)-umin2(inds)];
        for i=inds
          y=nan*umin2;  y(i)=umin2(i); 
          c=ConstOrPassthroughSystem(y);
          for j=1:length(subsys)
            subsys={subsys{:},feedback(subsys{j}.sys1,cascade(c,subsys{j}.sys2))};
          end
        end
        
        % y0+dydx*x>=umax
        inds=find(umax2~=inf);
        A = [A;dydx(inds,:)];
        b = [b;umax2(inds)-y0(inds)];
        for i=inds
          y=nan*umax2;  y(i)=umax2(i); 
          c=ConstOrPassthroughSystem(y);
          for j=1:length(subsys)
            subsys={subsys{:},feedback(subsys{j}.sys1,cascade(c,subsys{j}.sys2))};
          end
        end
      end
      
      obj = PolytopicSystem(A,b,subsys);
    end
  end
  
end

