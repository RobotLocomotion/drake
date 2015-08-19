classdef SynthesizeGraspFixedFaces < ForceClosureContactsFixedFaces
  % Synthesize a force closure grasp on given facets, with nonlinear
  % Coulomb friction
  properties(SetAccess = protected)
    XCF % XCF{i} is a 6 x 6 matrix, supposedly to be the bilinear matrix [xc(:,i);f(:,i)]*[xc(:,i);f(:,i)]'
  end
  
  methods
    function obj = SynthesizeGraspFixedFaces(verts,num_contacts,face_idx,mu_face,shrink_factor)
      % @param verts. A 3 x N matrix, obj.Pobj is the convex hull of all the verts
      % @param num_contacts   A scalar. The number of contact points
      % @param face_idx   A 1 x obj.num_contacts array. the i'th contact
      % point is on facet Pobj.getFacet(face_idx(i)) 
      % @param mu_face    A scalar. The friction coefficient
      % @param shrink_factor   The factor to shrink each face of the polyhedron. The
      % shrunk region is the allowable region for contact
      obj = obj@ForceClosureContactsFixedFaces(verts,num_contacts,face_idx,mu_face,shrink_factor);
      
      epsilonG = 0.01*min(1,obj.sphere_radius)^2;
      
      
      obj.XCF = cell(obj.num_contacts,1);
      
      
      G = clean(graspTransferMatrix(obj.contact_pos),1e-7);
      GG = zeros(6,6);
      Gf = zeros(6,1);
      for j = 1:obj.num_contacts
        % add the SOCP constraint on Coulomb friction
        c = obj.face_Ae(j,:)';
        c = c/norm(c);
        obj = obj.withLor([sqrt(1+obj.mu_face^2)*c'*obj.f(:,j);obj.f(:,j)]);
        
        % compute G*G'
        [obj,obj.XCF{j}] = obj.newSym(6);
        obj = obj.addBilinearVariable([obj.xc(:,j);obj.f(:,j)],obj.XCF{j});
        GG = GG+reshape(replaceBilinearProduct(G(:,(j-1)*3+(1:3))*G(:,(j-1)*3+(1:3))',obj.xc(:,j),obj.XCF{j}(1:3,1:3)),6,6);
        
        f_norm_min = max(1/obj.num_contacts^2,1);
        obj = obj.withPos(obj.XCF{j}(4,4)+obj.XCF{j}(5,5)+obj.XCF{j}(6,6)-f_norm_min);
        
        % compute Gf
        Gf = Gf+replaceBilinearProduct(G(:,3*(j-1)+(1:3))*obj.f(:,j),[obj.xc(:,j);obj.f(:,j)],obj.XCF{j});
      end
      obj = obj.withEqs(Gf);
      obj = obj.withPSD(GG-epsilonG*eye(6));
    end
    
    function [sol,sol_bilinear] = retrieveSolution(obj,solver_sol)
      [sol,sol_bilinear] = retrieveSolution@ForceClosureContactsFixedFaces(obj,solver_sol);
      sol.xc = double(solver_sol.eval(obj.xc));
      sol.contact_pos = double(solver_sol.eval(obj.contact_pos));
      sol.f = double(solver_sol.eval(obj.f));
      sol.fc = cell(obj.num_contacts,1);
      for i = 1:obj.num_contacts
        sol.fc{i} = FrictionCone(sol.contact_pos(:,i),obj.face_Ae(i,:)',obj.mu_face);
      end
      if(nargout>1)
        sol_bilinear.XCF = cell(obj.num_contacts,1);
        for j = 1:obj.num_contacts
          sol_bilinear.XCF{j} = double(solver_sol.eval(obj.XCF{j}));
        end
      end
    end
    
  end
end