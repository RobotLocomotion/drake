classdef GraspedPolyhedron < GraspedGeometry
  properties(SetAccess = protected)
    num_verts % number of vertices
    verts % A 3 x num_verts matrix, verts(:,i) is the i'th vertex of the polyhedron
    % The polyhderon can also be described as Ain*x <= bin
    Ain
    bin
    % We define a shrunk polyhedron as the convex hull of the shrunk
    % surface region
    shrink_factor  % A scalar in the range of (0,1]. The allowable grasp region on each face is the shrunk surface region
    verts_shrunk % A 3 x m matrix, verts(:,i) is the i'th vertex of the shrunk polyhedron
    % The shrunk polyhedron can also be defined as Ain_shrunk*x <=
    % bin_shrunk
    Ain_shrunk
    bin_shrunk
    
    % We also want to find the maximum sphere contained in the polyhedron
    inner_sphere_center  % The center of the inner sphere inside the polyhderon
    inner_sphere_radius  % The radius of the inner sphere inside the polyhedron
  end
  
  methods
    function obj = GraspedPolyhedron(verts,shrink_factor)
      % @param verts   A 3 x N matrix. The polyhedron is defined as the
      % convex hull of the columns of verts
      % @param shrink_factor    A scalar in the range of (0,1]. The
      % allowable grasp region in each surface is the shrunk region of each
      % face. The shrinking factor is shrink_factor. When shrink_factor =
      % 1, the allowable grasp region is the whole face (including the
      % edges of the polyhedron)
      obj = obj@GraspedGeometry(GraspedGeometry.POLYHEDRON_TYPE);
      if(size(verts,1) ~= 3 && length(size(verts)) ~= 2)
        error('verts should be a 3 x N matrix');
      end
      if(numel(shrink_factor) ~= 1 || shrink_factor > 1 || shrink_factor <=0)
        error('shrink_factor should be within (0,1]');
      end
      obj.shrink_factor = shrink_factor;
      TOL = 1e-10;
      [obj.Ain,obj.bin,Aeq,beq] = vert2lcon(verts',TOL);
      if(~isempty(Aeq))
        error('verts are on a 2D/1D subspace. We only support 3D object now');
      end
      verts_tmp = lcon2vert(obj.Ain,obj.bin,Aeq,beq,TOL);
      obj.num_verts = size(verts_tmp,1);
      obj.verts = verts_tmp';
      
      num_faces = size(obj.Ain,1);
      obj.verts_shrunk = [];
      for i = 1:num_faces
        face_verts = obj.verts(:,abs(obj.Ain(i,:)*obj.verts - obj.bin(i)) <= TOL);
        face_center = mean(face_verts,2);
        obj.verts_shrunk = [obj.verts_shrunk face_verts * obj.shrink_factor + bsxfun(@times,face_center,(1-obj.shrink_factor)*ones(1,size(face_verts,2)))];
      end
      [obj.Ain_shrunk,obj.bin_shrunk] = vert2lcon(obj.verts_shrunk',TOL);
      
%       [obj.Pobj,obj.Pobj_shrunk] = shrinkPolyhedron(verts,shrink_factor);
      [obj.inner_sphere_center,obj.inner_sphere_radius] = maxInnerSphere(obj.Ain,obj.bin);
    end
    
    function plotGeometry(obj,use_lcmgl)
      if(use_lcmgl)
        drawPolyhedron(obj.verts,'polyhderon');
      else
        hold on
        plotregion(-obj.Ain,-obj.bin,[],[],[0,0,1],0.5);
%         plot(obj.Pobj,'Color','b'); alpha(0.5);
        axis equal
        grid off
        drawnow;
        hold off
      end
    end
  end
end