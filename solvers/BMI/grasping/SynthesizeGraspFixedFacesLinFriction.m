classdef SynthesizeGraspFixedFacesLinFriction < ForceClosureContactsFixedFaces
  % Synthesize a force closure grasp on given facets, with linearized
  % Coulomb friction
  properties(SetAccess = protected)
    num_fc_edges  % Number of edges in each friction cones 
    face_fc_edge  % A num_contacts x 1 cell, face_fc_edge{i}(:,j) is the j'th edge of the friction cone at the i'th contact
    face_rotate_R  % A num_contacts x 1 cell, face_rotate_R{i} is a 3 x 3 rotation matrix, that rotates [0;0;1] to align with the normal vector on the contact facet of i'th contact point
    
    fc_weights    % A num_fc_edges x num_contacts matrix. fc_weights(:,i) are the weights on the friction cone edges for the i'th contact point
    
    XCWeights     % A num_contacts x 1 cell, XCWeights{i} is supposed to be the bilinear matrix of [xc(:,i);fc_weights(:,i)]*[xc(:,i);fc_weights(:,i)]'
  end
  
  methods
    function obj = SynthesizeGraspFixedFacesLinFriction(verts,num_contacts,face_idx,mu_face,num_fc_edges,shrink_factor)
      obj = obj@ForceClosureContactsFixedFaces(verts,num_contacts,face_idx,mu_face,shrink_factor);
      
      if(numel(num_fc_edges) ~= 1)
        error('num_fc_edges should be a scalar');
      end
      obj.num_fc_edges = num_fc_edges;
      fc_edge1 = linFCedges(obj.num_fc_edges,obj.mu_face);
      obj.face_fc_edge = cell(obj.num_contacts,1);
      obj.face_rotate_R = cell(obj.num_contacts,1);
      
      [obj,obj.fc_weights] = obj.newFree(obj.num_fc_edges,obj.num_contacts);
      obj.XCWeights = cell(obj.num_contacts,1);
      
      
      G = graspTransferMatrix(obj.contact_pos);
      % GG is G*G'
      GG = zeros(6,6);
      Gf = zeros(6,1);
      for i = 1:obj.num_contacts
        obj.face_rotate_R{i} = rotateVectorToAlign([0;0;1],obj.face_Ae(i,:)');
        obj.face_fc_edge{i} = obj.face_rotate_R{i}*fc_edge1;
        
        % non-zero force
        obj = obj.withPos(sum(obj.fc_weights(:,i))-1);
        
        [obj,obj.XCWeights{i}] = obj.newSym(3+obj.num_fc_edges);
        GG = GG+clean(reshape(replaceBilinearProduct(G(:,3*(i-1)+(1:3))*G(:,3*(i-1)+(1:3))',obj.xc(:,i),obj.XCWeights{i}(1:3,1:3)),6,6),1e-7);
        
        obj = obj.withEqs(obj.f(:,i) - obj.face_fc_edge{i}*obj.fc_weights(:,i));
        Gf = Gf+replaceBilinearProduct(G(:,3*(i-1)+(1:3))*obj.f(:,i),[obj.xc(:,i);obj.fc_weights(:,i)],obj.XCWeights{i});
        obj = obj.addBilinearVariable([obj.xc(:,i);obj.fc_weights(:,i)],obj.XCWeights{i});
        
        obj = obj.withPos(obj.fc_weights(:,i));
      end
      epsilonG = 0.01*min(1,obj.sphere_radius)^2;
      obj = obj.withPSD(GG-epsilonG*eye(6));
      obj = obj.withEqs(Gf);
    end
    
    function [sol,sol_bilinear] = retrieveSolution(obj,solver_sol)
      [sol,sol_bilinear] = retrieveSolution@ForceClosureContactsFixedFaces(obj,solver_sol);
      sol.xc = double(solver_sol.eval(obj.xc));
      sol.contact_pos = double(solver_sol.eval(obj.contact_pos));
      sol.f = double(solver_sol.eval(obj.f));
      sol.fc = cell(obj.num_contacts,1);
      for i = 1:obj.num_contacts
        face_fc_edges_i = double(solver_sol.eval(obj.face_fc_edge{i}));
        sol.fc{i} = LinearizedFrictionCone(sol.contact_pos(:,i),obj.face_Ae(i,:)',obj.mu_face,face_fc_edges_i);
      end
      if(nargout>1)
        sol_bilinear.XCWeights = cell(obj.num_contacts,1);
        for j = 1:obj.num_contacts
          sol_bilinear.XCWeights{j} = double(solver_sol.eval(obj.XCWeights{j}));
        end
      end
    end
  end
end