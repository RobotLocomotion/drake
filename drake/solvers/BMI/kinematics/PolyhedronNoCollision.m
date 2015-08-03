classdef PolyhedronNoCollision < BMIspotless
  % search for separating hyperplane c'x+d between two polyhedron to avoid
  % contact
  properties(SetAccess = protected)
    verts1
    verts2
    
    % The separating hyperplane is c'x+d = 0
    c 
    d
    
    quat1
    quat2
    pos1
    pos2
    
    pos_vert1
    pos_vert2
    
    Quat1
    Quat2
    
    cQuat1
    cQuat2
    
    c_pos1
    c_pos2
  end
  
  methods
    function obj = PolyhedronNoCollision(verts1,verts2)
      % Given two polyhedron with vertex verts1 and verts2 expressed in
      % their body frame, find the position and orientation of the two
      % polyhderon, such that they do not collide.
      if(size(verts1,1) ~= 3 || length(size(verts1)) ~= 2)
        error('verts1 should be a 3 x N matrix');
      end
      if(size(verts2,1) ~= 3 || length(size(verts2)) ~= 2)
        error('verts2 should be a 3 x M matrix');
      end
      obj = obj@BMIspotless();
      [Ain1,bin1] = vert2lcon(verts1');
      obj.verts1 = lcon2vert(Ain1,bin1,[],[])';
      [Ain2,bin2] = vert2lcon(verts2');
      obj.verts2 = lcon2vert(Ain2,bin2,[],[])';
      
      V1 = obj.verts1;
      V2 = obj.verts2;
      num_verts1 = size(V1,2);
      num_verts2 = size(V2,2);

      [obj,obj.c] = obj.newFree(3,1);
      [obj,obj.d] = obj.newFree(1,1);
      
      [obj,obj.quat1] = obj.newFree(4,1);
      [obj,obj.quat2] = obj.newFree(4,1);
      [obj,obj.pos1] = obj.newFree(3,1);
      [obj,obj.pos2] = obj.newFree(3,1);
      
      [obj,obj.Quat1] = obj.newSym(4);
      [obj,obj.Quat2] = obj.newSym(4);
      obj = obj.addBilinearVariable(obj.quat1,obj.Quat1);
      obj = obj.addBilinearVariable(obj.quat2,obj.Quat2);
      obj = obj.withEqs(obj.Quat1(1,1)+obj.Quat1(2,2)+obj.Quat1(3,3)+obj.Quat1(4,4)-1);
      obj = obj.withEqs(obj.Quat2(1,1)+obj.Quat2(2,2)+obj.Quat2(3,3)+obj.Quat2(4,4)-1);
      
      % compute the position of the vertices using forward kinematics
      obj.pos_vert1 = repmat(obj.pos1,1,num_verts1)+rotatePtByQuatBilinear(obj.Quat1,obj.verts1);
      obj.pos_vert2 = repmat(obj.pos2,1,num_verts2)+rotatePtByQuatBilinear(obj.Quat2,obj.verts2);
      
      [obj,obj.c_pos1] = obj.newSym(6);
      [obj,obj.c_pos2] = obj.newSym(6);
      obj = obj.addBilinearVariable([obj.c;obj.pos1],obj.c_pos1);
      obj = obj.addBilinearVariable([obj.c;obj.pos2],obj.c_pos2);
      obj = obj.withEqs(obj.c_pos1(1:3,1:3)-obj.c_pos2(1:3,1:3));
      % normal vector cannot be zero length
      obj = obj.withPos(obj.c_pos1(1,1)+obj.c_pos1(2,2)+obj.c_pos1(3,3)-1);
      
      [obj,obj.cQuat1] = obj.newSym(13);
      [obj,obj.cQuat2] = obj.newSym(13);
      Quat1_lower = [obj.Quat1(1,1);obj.Quat1(2,1);obj.Quat1(3,1);obj.Quat1(4,1);...
        obj.Quat1(2,2);obj.Quat1(3,2);obj.Quat1(4,2);obj.Quat1(3,3);obj.Quat1(4,3);obj.Quat1(4,4)];
      Quat2_lower = [obj.Quat2(1,1);obj.Quat2(2,1);obj.Quat2(3,1);obj.Quat2(4,1);...
        obj.Quat2(2,2);obj.Quat2(3,2);obj.Quat2(4,2);obj.Quat2(3,3);obj.Quat2(4,3);obj.Quat2(4,4)];
      obj = obj.addBilinearVariable([obj.c;Quat1_lower],obj.cQuat1);
      obj = obj.addBilinearVariable([obj.c;Quat2_lower],obj.cQuat2);
      obj = obj.withEqs(obj.cQuat1(1:3,1:3)-obj.c_pos1(1:3,1:3));
      obj = obj.withEqs(obj.cQuat2(1:3,1:3)-obj.c_pos2(1:3,1:3));
      
      % add the constraint that c'x+d>=0 for object 1
      obj = obj.withPos(repmat(clean(replaceBilinearProduct(obj.c'*obj.pos1,[obj.c;obj.pos1],obj.c_pos1),1e-7),1,num_verts1)+...
        reshape(clean(replaceBilinearProduct(obj.c'*rotatePtByQuatBilinear(obj.Quat1,obj.verts1),[obj.c;Quat1_lower],obj.cQuat1),1e-7),1,[])+...
        repmat(obj.d,1,num_verts1));
      % add the constraint that c'x+d<=0 for object 2
      obj = obj.withPos(-(repmat(clean(replaceBilinearProduct(obj.c'*obj.pos2,[obj.c;obj.pos2],obj.c_pos2),1e-7),1,num_verts2)+...
        reshape(clean(replaceBilinearProduct(obj.c'*rotatePtByQuatBilinear(obj.Quat2,obj.verts2),[obj.c;Quat2_lower],obj.cQuat2),1e-7),1,[])+...
        repmat(obj.d,1,num_verts2)));
    end
    
    function [sol,sol_bilinear] = retrieveSolution(obj,solver_sol)
      sol.pos1 = double(solver_sol.eval(obj.pos1));
      sol.pos2 = double(solver_sol.eval(obj.pos2));
      sol.quat1 = double(solver_sol.eval(obj.quat1));
      sol.quat2 = double(solver_sol.eval(obj.quat2));
      sol.pos_vert1 = double(solver_sol.eval(obj.pos_vert1));
      sol.pos_vert2 = double(solver_sol.eval(obj.pos_vert2));
      sol.c = double(solver_sol.eval(obj.c));
      sol.d = double(solver_sol.eval(obj.d));
      if(nargout > 1)
        sol_bilinear.Quat1 = double(solver_sol.eval(obj.Quat1));
        sol_bilinear.Quat2 = double(solver_sol.eval(obj.Quat2));
        sol_bilinear.c_pos1 = double(solver_sol.eval(obj.c_pos1));
        sol_bilinear.c_pos2 = double(solver_sol.eval(obj.c_pos2));
        sol_bilinear.cQuat1 = double(solver_sol.eval(obj.cQuat1));
        sol_bilinear.cQuat2 = double(solver_sol.eval(obj.cQuat2));
      end
    end
    
    function plotSolution(obj,sol)
      figure;
      [Ain1,bin1] = vert2lcon(sol.pos_vert1');
      [Ain2,bin2] = vert2lcon(sol.pos_vert2');
      hold on;
      plotregion(-Ain1,-bin1,[],[],[0,0,1],0.5);
      plotregion(-Ain2,-bin2,[],[],[1,0,0],0.5);
      
      axis equal;
      grid off;
      drawnow;
      hold off
    end
  end
end