function h = addXYAxis(xlabel,ylabel)

v = axis;

axis off;
h = line([2*v(1)-v(2),v(2)],[0 0],'Color',[0 0 0],'Marker','>','MarkerFaceColor',[0 0 0]);
h = [h,line([0,0],[2*v(3)-v(4),v(4)],'Color',[0 0 0],'Marker','^','MarkerFaceColor',[0 0 0])];

if (nargin > 0)
  h = [h,text(v(2),-.05*v(4),xlabel,'FontName','Times','FontSize',20,'Interpreter','Latex')];
end
if (nargin > 1)
  h = [h,text(v(1)-.1*(v(2)-v(1)),v(4)-.1,ylabel,'FontName','Times','FontSize',20,'Interpreter','Latex')];
end

axis(v);
