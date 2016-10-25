classdef ForceClosureContactsFixedFaces < BMIspotless
  % Find the contact points that can achieve force closure on a polyhedron
  properties(SetAccess = protected)
    Pobj         % A GraspedPolyhedron object
    num_contacts % The number of contact points
    mu_face      % The friction coefficient
    face_idx     % the i'th point is on facet obj.Pobj.getFacet(face_idx(i))
    % The contact points are parameterized by A_xc*obj.xc(:,i)+b_xc, where
    % A_xc represents the scaling, and b_xc represents the shifting
    A_xc % A 3 x 3 matrix
    b_xc % A 3 x 1 matrix
    xc % A 3 x obj.num_contacts matrix, obj.xc(:,i) is the decision variable for the i'th contact point
    contact_pos % A 3 x obj.num_contacts matrix. contact_pos(:,i) is the contact point for the i'th contact
    
    sphere_center
    sphere_radius
    
    face_Ae   % face_Ae is a num_contacts x 3 matrix, face_Ae(i,:) is the normal vector on face_idx(i)
    
    f             % A 3 x num_contacts matrix. f(:,i) is the contact force at i'th contact point
  end
  
  methods
    function obj = ForceClosureContactsFixedFaces(verts,num_contacts,face_idx,mu_face,shrink_factor)
      % @param verts. A 3 x N matrix, obj.Pobj is the convex hull of all the verts
      % @param num_contacts   A scalar. The number of contact points
      % @param face_idx   A 1 x obj.num_contacts array. the i'th contact
      % point is on facet Pobj.Ain(i,:)*x=Pobj.bin(i) 
      % @param mu_face    A scalar. The friction coefficient
      % @param shrink_factor   The factor to shrink each face of the polyhedron. The
      % shrunk region is the allowable region for contact
      obj = obj@BMIspotless();
      if(size(verts,1)~=3)
        error('verts should be a 3 x N matrix');
      end
      obj.Pobj = GraspedPolyhedron(verts,shrink_factor);
      [obj.sphere_center,obj.sphere_radius] = maxInnerSphere(obj.Pobj.Ain,obj.Pobj.bin);
      obj.A_xc = obj.sphere_radius*eye(3);
      obj.b_xc = obj.sphere_center;
      if(numel(num_contacts) ~= 1)
        error('num_contacts should be a scalar');
      end
      obj.num_contacts = num_contacts;
      if(any(size(face_idx)~=[1,obj.num_contacts]))
        error('face_idx should be a 1 x %d array',obj.num_contacts);
      end
      obj.face_idx = face_idx;
      if(numel(mu_face) ~= 1 || mu_face<0)
        error('mu_face should be a non-negative scalar');
      end
      obj.mu_face = mu_face;
      if(numel(shrink_factor) ~= 1 || shrink_factor<=0 || shrink_factor>1)
        error('shrink_factor should be a scalar, in the range of (0,1]');
      end
      [obj,obj.xc] = obj.newFree(3,obj.num_contacts);
      obj.contact_pos = obj.A_xc*obj.xc+bsxfun(@times,obj.b_xc,ones(1,obj.num_contacts));
      
      obj.face_Ae = zeros(obj.num_contacts,3);
      for j = 1:obj.num_contacts
        % contact_pos lies on the shrunk facet
        face_shrunk_vert = obj.Pobj.verts_shrunk(:,abs(obj.Pobj.Ain(obj.face_idx(j),:)*obj.Pobj.verts_shrunk - obj.Pobj.bin(obj.face_idx(j)))< 1e-10);
        [Ain_shrunk_j,bin_shrunk_j,Aeq_shrunk_j,beq_shrunk_j] = vert2lcon(face_shrunk_vert');
        obj = obj.withPos(bin_shrunk_j - Ain_shrunk_j*obj.contact_pos(:,j));
        obj = obj.withEqs(obj.Pobj.bin(obj.face_idx(j)) - obj.Pobj.Ain(obj.face_idx(j),:)*obj.contact_pos(:,j));        
        obj.face_Ae(j,:) = obj.Pobj.Ain(obj.face_idx(j),:)/norm(obj.Pobj.Ain(obj.face_idx(j),:));
      end
      [obj,obj.f] = obj.newFree(3,obj.num_contacts);
    end
    
    function [sol,sol_bilinear] = retrieveSolution(obj,solver_sol)
      [sol,sol_bilinear] = retrieveSolution@BMIspotless(obj,solver_sol);
      sol.xc = double(solver_sol.eval(obj.xc));
      sol.contact_pos = double(solver_sol.eval(obj.contact_pos));
      sol.f = double(solver_sol.eval(obj.f));
    end
    
    function plotSolution(obj,sol,sol_bilinear)
      plotSolution@BMIspotless(obj,sol,sol_bilinear);
      obj.Pobj.plotGeometry(obj.use_lcmgl);
      force_scalar = det(obj.A_xc'*obj.A_xc)^(1/6)*0.5/min(sqrt(sum(sol.f.^2,1)));
      for i = 1:obj.num_contacts
        sol.fc{i}.plot(obj.use_lcmgl,force_scalar*norm(sol.f(:,i))*0.5,sprintf('fc%d',i));
      end
      if(obj.use_lcmgl)
        for i = 1:obj.num_contacts
          lcmgl_force = drake.matlab.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton,sprintf('force%d',i));
          lcmgl_force.glColor3f(1,0,0);
          lcmgl_force.drawVector(sol.contact_pos(:,i),sol.f(:,i)*force_scalar,0.01*force_scalar,0.04*force_scalar,0.04*force_scalar);
          lcmgl_force.switchBuffers();
        end
      else
        hold on;
        for k = 1:obj.num_contacts
          arrow_start = sol.contact_pos(:,k);
          arrow_end = arrow_start+sol.f(:,k)*force_scalar;
          arrow_force = [arrow_start arrow_end];
          arrow_radius = norm(sol.f(:,k))*force_scalar*0.1;
          h = arrow3d(arrow_force(1,:),arrow_force(2,:),arrow_force(3,:),0.7,arrow_radius,arrow_radius*2,[1,0,0]);
        end
        hold off;
      end
    end
  end
end
