classdef Gripper2D < PlanarRigidBodyManipulator
  
  properties (SetAccess=protected,GetAccess=public)
    
  end
  
  methods
    function obj = Gripper2D()
      obj = obj@PlanarRigidBodyManipulator('gripper.urdf')
      obj.num_contacts = obj.num_contacts + 1; %add ground contact
    end
    
    function [phi_n,n,D,mu,dn,dD,psi,dPsi,phi_f,dPhi] = contactConstraints(obj,q,qd)
      [phi, psi, dPhi, dPsi, J, dJ, psi_full] = contactPositionsAndVelocities(obj,q,qd);
      n = J(2:2:end,:);
      D{1} = J(1:2:end,:);
      D{2} = -D{1};
      mu = 1;
      phi_f = phi(1:2:end);
      phi_n = phi(2:2:end);
      
      dn = reshape(dJ(2:2:end,:),obj.num_positions*length(phi_n),obj.num_positions);
      dD{1} = reshape(dJ(1:2:end,:),obj.num_positions*length(phi_n),obj.num_positions);
      dD{2} = -dD{1};
      
      %not sure if this is right
      psi = reshape(psi_full,2,[]);
    end
    
    function [a,dA] = test(obj,q)
      [phi, psi, dPhi, dPsi, J, dJ, psi_full] = contactPositionsAndVelocities(obj,q,q*0);
      a = J(:);
      dA = dJ;
    end
    
    function [phi, psi, dPhi, dPsi, J, dJ, psi_full] = contactPositionsAndVelocities(obj,q,qd)
      % Calculate positions and velocities of all contact points.
      % Also calculates gradients of positions and velocities, the relevant
      % Jacobian and its gradient.
      % For efficiency purposes, Requires that that
      % doKinematicsAndVelocites has already been called
      
      kinsol = doKinematics(obj,q,true);
      
      r=0.1;
      phi = [];
      psi = [];
      psi_full = [];
      J = [];
      dPhi = [];
      dPsi = [];
      dJ = [];
      [sphere_coords, dPts_sphere, ddPts_sphere] = obj.forwardKin(kinsol,4,[0;0]);
     
      
      nBodies = length(obj.body);
      for i=1:nBodies;
        contact_pts = obj.model.body(i).contact_pts;
        nC = size(contact_pts,2);
        if nC>0
          for y=1:nC,
            [pts,dPts,ddPts] = obj.forwardKin(kinsol,i,contact_pts(:,y));
            [vp,vvp] = obj.forwardKinVel(kinsol,i,contact_pts(:,y),qd);
            
            c = pts;
            n = c - sphere_coords;
            dist = norm(n) - r;
            n = n/norm(n);
            t = [0 1; -1 0]*n;
%             t = cross([n;0],[0;0;1]);
%             t = t(1:2);

            dn = (dPts - dPts_sphere)/(dist+r) - (pts - sphere_coords)*(pts - sphere_coords)'*(dPts - dPts_sphere)/(dist+r)^3;
            dt = [0 1; -1 0]*dn;
            

            % calculate the velocity of the point on the surface of the
            % ball
            pt_on_sphere = rotmat(q(3))*n*r;
            %calculating this manually to get a point on the sphere
            [vb,vvb] = obj.forwardKinVel(kinsol,4,pt_on_sphere,qd);
%             vb = [qd(1);qd(2)]-qd(3)*[n(2)*r;-n(1)*r];
%             vvb(:,obj.num_positions+1:end) = vvb(:,obj.num_positions+1:end) - qd(3)*r*dt;
            
            
%             %manually calculating ddn, the hard way
%             R = (pts - sphere_coords);
%             dR = (dPts - dPts_sphere);
%             ddR = (ddPts - ddPts_sphere);
%             ddn = (ddPts - ddPts_sphere)/(dist+r) - R*R'*ddR/(dist+r)^3;
%             for k=1:obj.num_positions,
%               for j=1:obj.num_positions,
%                 ddn(:,(k-1)*obj.num_positions + j) = ddn(:,(k-1)*obj.num_positions + j) - dR(:,k)*R'*dR(:,j)/(dist+r)^3 - dR(:,j)*R'*dR(:,k)/(dist+r)^3 + ...
%                   -R*dR(:,j)'*dR(:,k)/(dist+r)^3 + 3*R*R'*dR(:,j)*R'*dR(:,k)/(dist+r)^5;
%               end
%             end
            
%             sphere_surf = sphere_coords + r*n;
%             dsphere_surf = dPts_sphere + r*dn;
%             ddsphere_surf = ddPts_sphere + r*ddn;
            [sphere_surf, dsphere_surf, ddsphere_surf] = obj.forwardKin(kinsol,4,pt_on_sphere*0);
            ddsphere_surf(:,3:obj.num_positions:end) = dt*r*0;
            
%             sphere_surf = sphere_coords - r*n;
%             dsphere_surf = dPts_sphere;
%             dsphere_surf(:,3) = t*r;
%             ddsphere_surf = ddPts_sphere - r*ddn;
            
            phi=[phi;[0;dist]];
            
            vel_n = (vp - vb)'*n;
            vel_t = (vp - vb)'*t;
            psi = [ psi ; vel_t ];
            psi_full=[psi_full;[vel_t;vel_n]];
            
            dvel_t = t'*(vvp - vvb);
            dvel_n = n'*(vvp - vvb);
            %these are actually the jacobians, not dphi/dq.  should clarify the distinction more carefully
            J_t = t'*(dPts-dsphere_surf);  
            dJ_t = t'*(ddPts - ddsphere_surf);
            
            J_n = n'*(dPts-dsphere_surf);
            dJ_n = n'*(ddPts - ddsphere_surf);
            
            %add in contribution from dn, dt to dJ
            tmp=(dPts-dsphere_surf)'*dn;
            dJ_n = dJ_n + tmp(:)';
            
            tmp=(dPts-dsphere_surf)'*dt;
            dJ_t = dJ_t + tmp(:)';
            
            %add in contribution from dn, dt to dpsi
            tmp=(vp-vb)'*dn;
            dvel_n(1:obj.num_positions) = dvel_n(1:obj.num_positions) + tmp(:)';
            
            tmp=(vp-vb)'*dt;
            dvel_t(1:obj.num_positions) = dvel_t(1:obj.num_positions) + tmp(:)';
            
            %add in contribution from dn, dt to dphi
            dphi_t = t'*(dPts-dPts_sphere) + (c - sphere_coords)'*dt;
            dphi_n = n'*(dPts-dPts_sphere) + (c - sphere_coords)'*dn;

            dPsi=[dPsi;dvel_t;dvel_n];
            J=[J;J_t;J_n];
            dJ=[dJ;dJ_t;dJ_n];
            dPhi = [dPhi;dphi_t;dphi_n];
          end
        end
      end
      [sphere_coords, dPts_sphere, ddPts_sphere] = obj.forwardKin(kinsol,4,[0;0]);
      dPts_sphere(1,:) = 0*dPts_sphere(1,:);
      
      [vb,vvb] = obj.forwardKinVel(kinsol,4,[0;0],qd);
      vb(1) = vb(1) - r*qd(3);
      vvb(1,obj.num_positions + 3) = vvb(1,obj.num_positions + 3) - r;

      %Add an extra contact for the ball and the ground
      %scale up for traj. opt numerics
      scale = 100;
      %[phi, psi, dPhi, dPsi, J, dJ, psi_full] 
      phi = [phi;0;(sphere_coords(2) - r)*scale];
      psi = [psi;vb(2)*scale];
      psi_full = [psi_full;vb(1:2)*scale];
      dPhi = [dPhi;dPts_sphere*scale];
      J = [J;dPts_sphere];
      dPsi = [dPsi;vvb*scale];
      dJ = [dJ;ddPts_sphere];
    end
  end
  
end