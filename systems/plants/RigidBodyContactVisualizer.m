classdef RigidBodyContactVisualizer < Visualizer
  
  methods
    function obj = RigidBodyContactVisualizer(manip)
      typecheck(manip,'RigidBodyManipulator');
      obj=obj@Visualizer(manip.getStateFrame);
      obj.model = manip;
    end
    
    function draw(obj,t,x)

      persistent hFig;
      
      if (isempty(hFig))
        hFig = sfigure(32);
        set(hFig,'DoubleBuffer','on');
      end

      sfigure(hFig);
      clf; hold on;
      
      nq = obj.getNumPositions();
      nv = obj.getNumVelocities();
      q = x(1 : nq); qd = x(nq + (1 : nv));
      kinsol = obj.doKinematics(q);
      
      % for debugging:
      %co = get(gca,'ColorOrder');
      %h = [];
      % end debugging

      for i=1:length(obj.model.body)
        body = obj.model.body(i);
        nC = size(body.contact_pts,2);
        if nC>0
          contact_pos = forwardKin(obj,kinsol,i,body.contact_pts);
          ind = nchoosek(1:nC,2);
          for k=1:size(ind,1)
            line(contact_pos(1,ind(k,:)),contact_pos(2,ind(k,:)),contact_pos(3,ind(k,:)));
          end
        end
      end
      
      axis equal;
      view(0,10);
      if ~isempty(obj.xlim)
        xlim(obj.xlim);
      end
      if ~isempty(obj.ylim)
        ylim(obj.ylim);
      end
      if ~isempty(obj.axis)
        axis(obj.axis);
      end
      
      title(['t = ', num2str(t,'%.2f') ' sec']);
      drawnow;
    end
  end

  properties
    model;
    xlim=[]
    ylim=[];
  end
end
