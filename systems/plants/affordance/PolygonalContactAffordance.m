classdef PolygonalContactAffordance < ContactAffordance
  properties 
      % when is_fixed = false, the vertices, edges, normal and edge_normals
      % are all in the body frame.
      % when is_fixed = true, the vertices, edges, normal and edge_normals
      % are all in the world frame
    vertices % [3 x n_vert] position of surface vertices:
    n_vert
    edges % The edges, normal, and edge_normals are all in the body frame.
    normal 
    edge_normals
  end
  methods
    function obj = PolygonalContactAffordance(robot,body_ind,name,vertices,is_fixed,mu)
        if(nargin<6)
            mu = 1;
        end
      obj = obj@ContactAffordance(robot,body_ind,name,is_fixed,mu);
      obj = setVertices(obj,vertices);
    end
    
    function obj = setVertices(obj,vertices)
        % we rearrange the vertices so that it is in counter-clockwise
        % order, and a convex hull.
      obj.vertices = vertices;
      if(length(unique(obj.vertices(1:2)','rows'))>2)
        k = convhull(obj.vertices(1:2,:)','simplify',true);
      else
        k = convhull(obj.vertices(2:3,:)','simplify',true);
      end
      obj.vertices = obj.vertices(:,k(1:end-1));
      obj.n_vert = size(obj.vertices,2);
      obj.edges = circshift(obj.vertices,[0 -1]) - obj.vertices; 
      obj.normal = cross(obj.edges(:,1),obj.edges(:,2));
      obj.normal = obj.normal/norm(obj.normal);
      obj.edge_normals = zeros(3,obj.n_vert);
      for i = 1:obj.n_vert
        m = cross(obj.normal,obj.edges(:,i));
        obj.edge_normals(:,i) = m/norm(m);
      end
    end
    
    function [dist, dDist] = distancePts2Aff(obj,q,pts)
%       [n_pts,n_q] = size(dPts,2);
      n_pts = size(pts,2);
      nq = obj.robot.getNumPositions();
      sizecheck(q,[nq,1]);
      if(obj.is_fixed)
          normal_abs = obj.normal;
          vert_abs = obj.vertices(:,1);
          dnormal_absdq = zeros(3,nq);
          dvert_absdq = zeros(3,nq);
      else
          kinsol = doKinematics(obj.robot,q);
          [pts_abs,dpts_absdq] = forwardKin(obj.robot,kinsol,obj.body_ind,[obj.vertices(:,1) obj.normal zeros(3,1)],0);
          normal_abs = pts_abs(:,2)-pts_abs(:,3);
          dnormal_absdq = dpts_absdq(4:6,:)-dpts_absdq(7:9,:);
          vert_abs = pts_abs(:,1);
          dvert_absdq = dpts_absdq(1:3,:);
      end
      dist = (pts-bsxfun(@times,vert_abs,ones(1,n_pts)))'*normal_abs;
      if(nargout>1)
          dDistdq = repmat(normal_abs'*(-dvert_absdq),n_pts,1)+(pts-bsxfun(@times,vert_abs,ones(1,n_pts)))'*dnormal_absdq;
          dDistdpts = sparse(reshape(bsxfun(@times,[1;1;1],(1:n_pts)),[],1),...
              (1:n_pts*3)',reshape(bsxfun(@times,normal_abs,ones(1,n_pts)),[],1));
          dDist = [dDistdq dDistdpts];
      end
      
    end
    
    function [res, dRes] = additionalResPts2Aff(obj,q,pts)
      % res =
      % [pts(:,1)'*edge_normal(:,1);pts(:,1)'*edge_normal(:,2);...pts(:,1)'*edge_normal(:,N);...]
      % res >= 0 means that the projection of the pts are within the
      % polygon
      n_pts = size(pts,2);
      nq = obj.robot.getNumPositions();
      sizecheck(q,[nq,1]);
      if(obj.is_fixed)
          vert_abs = obj.vertices;
          edge_normals_abs = obj.edge_normals;
          dvert_abs = zeros(3*obj.n_vert,nq);
          dedge_normals_abs = zeros(3*obj.n_vert,nq);
      else
          kinsol = doKinematics(obj.robot,q);
          [pts_abs,dpts_absdq] = forwardKin(obj.robot,kinsol,obj.body_ind,[obj.vertices obj.edge_normals zeros(3,1)],0);
          vert_abs = pts_abs(:,1:obj.n_vert);
          dvert_abs = dpts_absdq(1:3*obj.n_vert,:);
          edge_normals_abs = pts_abs(:,obj.n_vert+(1:obj.n_vert))-bsxfun(@times,pts_abs(:,end),ones(1,obj.n_vert));
          dedge_normals_abs = dpts_absdq(3*obj.n_vert+(1:3*obj.n_vert),:)-repmat(dpts_absdq(end-2:end,:),obj.n_vert,1);
      end
      % res = [(p1-v1)*e1;(p1-v2)*e2;...;(p1-vn)*en;(p2-v1)*e1;...];
      res = sum((reshape(repmat(pts,obj.n_vert,1),3,[])-repmat(vert_abs,1,n_pts)).*repmat(edge_normals_abs,1,n_pts),1)';
      if(nargout>1)
          dResdq = sparse(reshape(bsxfun(@times,ones(3,1),(1:obj.n_vert*n_pts)),[],1),...
              (1:3*obj.n_vert*n_pts)',reshape((reshape(repmat(pts,obj.n_vert,1),3,[])-repmat(vert_abs,1,n_pts)),[],1))*...
              repmat(dedge_normals_abs,n_pts,1)+...
              repmat(sparse(reshape(bsxfun(@times,ones(3,1),(1:obj.n_vert)),[],1),...
              (1:3*obj.n_vert),-edge_normals_abs(:))*dvert_abs,n_pts,1);
          dResdpts = sparse(reshape(bsxfun(@plus,[(1:obj.n_vert)';(1:obj.n_vert)';(1:obj.n_vert)'],(0:n_pts-1)*obj.n_vert),[],1),...
              reshape(bsxfun(@times,(1:3*n_pts),ones(obj.n_vert,1)),[],1),...
              reshape(bsxfun(@times,reshape(edge_normals_abs',[],1),ones(1,n_pts)),[],1));
          dRes = [dResdq dResdpts];
      end
    end
  end
end
