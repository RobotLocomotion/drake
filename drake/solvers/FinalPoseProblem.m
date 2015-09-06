classdef FinalPoseProblem
  
  properties
    robot
    endEffectorId
    xStart
    xGoal
    additionalConstraints
    goalConstraints
    qNom
    endEffectorPoint
    minDistance
    capabilityMap
    graspingHand
    activeCollisionOptions
    jointSpaceTree
  end
  
  properties (Constant)
    SUCCESS = 1
    FAIL_NO_FINAL_POSE = 12
  end  
  
  methods
    
    function obj = FinalPoseProblem(robot, endEffectorId,...
         xStart, xGoal, additionalConstraints, goalConstraints, qNom, varargin)
      
      opt = struct('capabilitymap', [], 'graspinghand', 'right', ...
                   'mindistance', 0.005, ...
                   'activecollisionoptions', struct(), 'ikoptions', struct(), ...
                   'endeffectorpoint', [0; 0; 0]);
      optNames = fieldnames(opt);
      nArgs = length(varargin);
      if round(nArgs/2)~=nArgs/2
        error('Needs propertyName/propertyValue pairs')
      end
      for pair = reshape(varargin,2,[])
        inpName = lower(pair{1});
        if any(strcmp(inpName,optNames))
          opt.(inpName) = pair{2};
        else
          error('%s is not a recognized parameter name',inpName)
        end
      end
      
      obj.robot = robot;
      obj.endEffectorId = endEffectorId;
      obj.xStart = xStart;
      obj.xGoal = xGoal;
      obj.additionalConstraints = additionalConstraints;
      obj.goalConstraints = goalConstraints;
      obj.qNom = qNom;
      obj.endEffectorPoint = opt.endeffectorpoint;
      obj.minDistance = opt.mindistance;
      obj.capabilityMap = opt.capabilitymap;   
      obj.graspingHand = opt.graspinghand;
      obj.activeCollisionOptions = opt.activecollisionoptions;
      
      obj.jointSpaceTree = JointSpaceMotionPlanningTree(obj.robot);
      obj.jointSpaceTree.min_distance = 0.9*obj.minDistance; % minoring TaskSpaceMotionPlanningTree 0.9 magic number
      obj.jointSpaceTree.active_collision_options = opt.activecollisionoptions;
      obj.jointSpaceTree.ikoptions = opt.ikoptions;
      obj.jointSpaceTree = obj.jointSpaceTree.addKinematicConstraint(additionalConstraints{:});

    end    

    function [xGoal, info] = findFinalPose(obj, options)
      if nargin < 2, options = struct(); end
      
      info = obj.SUCCESS;
      
      %compute final pose
      tic
      if length(obj.xGoal) == 3 || length(obj.xGoal) == 7
        disp('Searching for a feasible final configuration...')
        qGoal = obj.searchFinalPose(obj.xStart, obj.xGoal);
        if isempty(qGoal)
          info = obj.FAIL_NO_FINAL_POSE;
          disp('Failed to find a feasible final configuration')
          return
        else
          kinsol = obj.robot.doKinematics(qGoal);
          obj.xGoal = obj.robot.forwardKin(kinsol, obj.endEffectorId, obj.endEffectorPoint, 2);
          obj.xGoal = [obj.xGoal; qGoal];
          disp('Final configuration found')
        end
      elseif length(obj.xGoal) == 7 + obj.robot.num_positions
        disp('Final configuration input found A')
      elseif obj.robot.num_positions
        kinsol = obj.robot.doKinematics(obj.xGoal);
        obj.xGoal = [obj.robot.forwardKin(kinsol, obj.endEffectorId, obj.endEffectorPoint, 2); obj.xGoal];
        disp('Final configuration input found B')
      else
        error('Bad final configuration input')
      end    
    
      xGoal = obj.xGoal;
    end
    
    function [qOpt, cost] = searchFinalPose(obj, xStart, xGoal)
      
      kinSol = obj.robot.doKinematics(xStart(8:end));
      options.rotation_type = 2;
      options.use_mex = false;
      
      root = obj.capabilityMap.rootLink.(obj.graspingHand);
      endEffector = obj.capabilityMap.endEffectorLink.(obj.graspingHand);
      rootPoint = obj.capabilityMap.rootPoint.(obj.graspingHand);
      EEPoint = obj.capabilityMap.endEffectorPoint.(obj.graspingHand);
      base = obj.capabilityMap.baseLink;
      
      rootPose = obj.robot.forwardKin(kinSol, root, rootPoint, 2);
      trPose = obj.robot.forwardKin(kinSol, base, [0;0;0], 2);
      tr2root = quat2rotmat(trPose(4:end))\(rootPose(1:3)-trPose(1:3));
      armJoints = 13:19;
      nArmJoints = size(armJoints, 2);
      np = obj.robot.num_positions;
      collisionLinksBody = setdiff(obj.activeCollisionOptions.body_idx, 9:15);
      mapMirror.right = [1; 1; 1];
      mapMirror.left = [1; -1; 1];
       
      obj = obj.pruneCapabilityMap(0, 0, 0.6, 0.9, 7.5);
      sphCenters = obj.capabilityMap.sphCenters;
      nSph = obj.capabilityMap.nSph;
      iter = 0;
      qOpt = [];
      cost = [];
      c = 1/obj.minDistance;
      deltaQmax = 0.05;
      validConfs =  double.empty(np+1, 0);
      succ = zeros(nSph, 2);
      
      for sph = randperm(nSph)
        iter = iter + 1;
        point = (sphCenters(:,sph).*mapMirror.(obj.graspingHand)) + tr2root;
        shConstraint = WorldPositionConstraint(obj.robot, base, point, xGoal(1:3), xGoal(1:3));
        constraints = [{shConstraint}, obj.goalConstraints];
        [q, valid] = obj.jointSpaceTree.solveIK(obj.qNom, obj.qNom, constraints);
        kinSol = obj.robot.doKinematics(q, ones(obj.robot.num_positions, 1), options);
        palmPose = obj.robot.forwardKin(kinSol, endEffector, EEPoint, options);
        targetPos = [palmPose(1:3); quat2rpy(palmPose(4:7))];
        deltaX = zeros(6,1);
        if valid
          phiBody = obj.robot.collisionDetect(q, false, struct('body_idx', collisionLinksBody));
          if all(phiBody > obj.minDistance)
            eps  = Inf;
            nIter = 0;
            [phi,normal,~,~,idxA,idxB] = obj.robot.collisionDetect(q, false, obj.activeCollisionOptions);
            if any(phi < obj.minDistance)
              phi = phi - obj.minDistance;
              while (eps > 1e-3 || any(phi < 0)) && nIter < 5
                qNdot = zeros(nArmJoints, 1);
                for joint = 1:nArmJoints
                  dgamma_dq = zeros(size(phi));
                  for coll = 1:size(phi,1)
                    if phi(coll) < 0
                      JA = obj.computeJacobian(kinSol, armJoints, idxA(coll));
                      JB = obj.computeJacobian(kinSol, armJoints, idxB(coll));
                      dD_dq = normal(:,coll)'*(JB(1:3,joint) - JA(1:3,joint));
                      dgamma_dq(coll) = exp(1./(c*phi(coll))).*(1-c*phi(coll))./(c*phi(coll))*c.*dD_dq;
                    else
                      dgamma_dq(coll) = 0;
                    end
                  end
                  qNdot(joint) = sum(dgamma_dq);
                end
                J = obj.computeJacobian(kinSol, armJoints, endEffector);
                Jpsi = J'*inv(J*J');
                deltaQ = Jpsi*deltaX + (eye(nArmJoints) - Jpsi*J) * qNdot;
                if any(abs(deltaQ) > deltaQmax)
                  alpha = deltaQmax/abs(deltaQ);
                elseif all(deltaQ < 1e-3)
                  break
                else
                  alpha = 1;
                end
                q(armJoints) = q(armJoints) + alpha*deltaQ;
                [phi,normal,~,~,idxA,idxB] = obj.robot.collisionDetect(q, false, obj.activeCollisionOptions);
                kinSol = obj.robot.doKinematics(q, ones(obj.robot.num_positions, 1), options);
                palmPose = obj.robot.forwardKin(kinSol, endEffector, [0;0;0], options);
                deltaX = targetPos - [palmPose(1:3); quat2rpy(palmPose(4:7))];
                eps = norm(deltaX);
                nIter = nIter + 1;
                phi = phi - obj.minDistance;
              end              
            else
              phi = phi - obj.minDistance;
              eps = 1e-3;
            end
            if eps <= 1e-3 && all(phi >= 0)
              cost = (obj.qNom - q)'*obj.jointSpaceTree.ikoptions.Q*(obj.qNom - q);
              validConfs(:,sph) = [cost; q];
              succ(sph, :) = [1, nIter];
              if cost < 20
                break
              end
            else              
              succ(sph, :) = [0, nIter];
            end
          end
        end
      end
      if ~isempty(validConfs)
        validConfs = validConfs(:, validConfs(1,:) > 0);
        [cost, qOptIdx] =  min(validConfs(1,:));
        qOpt = validConfs(2:end, qOptIdx);
      end
    end    
    
    
    function obj = pruneCapabilityMap(obj, sagittalAngle,...
        transverseAngle, sagittalWeight, transverseWeight, reachabilityWeight)
      
      Dmax = max(obj.capabilityMap.reachabilityIndex);
      nSph = length(obj.capabilityMap.map);
      indices = [];
      
      for sph = 1:nSph
        sa = atan2(obj.capabilityMap.sphCenters(3,sph), obj.capabilityMap.sphCenters(1,sph));
        ta = atan2(obj.capabilityMap.sphCenters(2,sph), obj.capabilityMap.sphCenters(1,sph));
        sagittalCost = sagittalWeight * abs(sa - sagittalAngle);
        transverseCost = transverseWeight * abs(ta - transverseAngle);
        reachabilityCost = reachabilityWeight * (Dmax - obj.capabilityMap.reachabilityIndex(sph));
        if sqrt(sagittalCost^2 + transverseCost^2) + reachabilityCost < 2
          indices(end + 1) = sph;
        end
      end
      obj.capabilityMap.nSph = length(indices);
      obj.capabilityMap.reachabilityIndex = obj.capabilityMap.reachabilityIndex(indices);
      obj.capabilityMap.map = obj.capabilityMap.map(indices, :);
      obj.capabilityMap.sphCenters = obj.capabilityMap.sphCenters(:, indices);
    end    
    
  end
  
end