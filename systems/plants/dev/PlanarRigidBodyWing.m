classdef PlanarRigidBodyWing < RigidBodyForceElement

  properties
    body    % RigidBody
    center=zeros(3,1);  % xyz and rpy of the reference point for the wing
    fCl  % splines representing the *dimensional* coefficients
    fCd  %  with fCl = 1/2 rho*S*Cl, etc.
    fCm
  end
  
  methods
    function obj = PlanarRigidBodyWing(body,aerodynamic_center,chord,span,profile,stallAngle)
      % call xfoil and avl as necessary to compute splines for Cl, Cd, and Cm
      
      
    end
    
    function f_ext = computeSpatialForce(obj,manip,q,qd)
      kinsol = doKinematics(manip,q);
      
      [x,J] = forwardKin(manip,kinsol,obj.body,obj.center(1:2),true);
      v = J*qd;
      
      relwindvel = -v(1:2);  % assume still air
      airspeed = norm(relwindvel);
      
      aoa = obj.center(3)+x(3) - atan2(relwindvel(2),relwindvel(1));  

      f_ext = sparse(3,getNumDOF(manip));
      lift = ppval(fCl,aoa)*[-relwindvel(2);relwindvel(1)];  % equivalent to rotmat(pi/2)*relwindvel
      drag = ppval(fCd,aoa)*relwindvel;
      f_ext(:,obj.body.dofnum) = [ppval(fCm,aoa);zeros(2,1)] + ...
        RigidBodyForceElement.cartesianForceToSpatialForce(obj.center(1:2),lift+drag);
    end
    
  end
  
end
