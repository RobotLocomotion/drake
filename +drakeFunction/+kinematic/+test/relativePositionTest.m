function relativePositionTest(visualize)
  import drakeFunction.*
  %import drakeFunction.euclidean.*
  import drakeFunction.kinematic.*

  if nargin < 1, visualize = false; end

  w = warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
  options.floating = true;
  urdf = fullfile(getDrakePath(),'examples','Atlas','urdf','atlas_minimal_contact.urdf');
  rbm = RigidBodyManipulator(urdf,options);
  warning(w);
  if visualize
    lcmgl = LCMGLClient('relativePositionTest');
    v = rbm.constructVisualizer();
  end

  hand_pt_expr = RelativePosition(rbm,'l_hand','world',rbm.getBody(rbm.findLinkInd('l_hand')).getTerrainContactPoints());

  q0 = zeros(rbm.getNumPositions(),1);
  S = load(fullfile(getDrakePath(),'examples','Atlas','data','atlas_fp.mat'));
  q_nom = S.xstar(1:numel(q0));
  q0 = q_nom;

  [pos,J] = hand_pt_expr.eval(q0);

  if visualize
    lcmgl.glColor3f(1,0,0);
    for pt = reshape(pos,3,[])
      lcmgl.sphere(pt,0.05,20,20);
    end
    rbm.drawLCMGLAxes(lcmgl,q0,rbm.findLinkInd('l_hand'));
    lcmgl.switchBuffers();
    v.draw(0,q0);
  end

  [pos,J_taylorvar] = geval(@hand_pt_expr.eval,q0,struct('grad_method','taylorvar'));
  [f,df] = geval(@hand_pt_expr.eval,q0,struct('grad_method',{{'user','taylorvar'}},'tol',1e-6));

  %foot_pts_expr = RelativePosition(rbm,'l_foot','world',rbm.getBody(rbm.findLinkInd('l_foot')).getTerrainContactPoints());
  %R3 = drakeFunction.frames.R(3);
  %dist_to_ground_expr = duplicate(SignedDistanceToHyperplane(Point(R3,0),Point(R3,[0;0;1])),foot_pts_expr.n_pts);
  %lb = zeros(foot_pts_expr.n_pts,1);
  %ub = lb;
  %foot_on_ground_cnstr = ExpressionConstraint(lb,ub,compose(dist_to_ground_expr,foot_pts_expr));

  %lb = zeros(hand_pt_expr.n_pts,1);
  %ub = lb;
  %dist_to_ground_expr = duplicate(SignedDistanceToHyperplane(Point(R3,0),Point(R3,[0;0;1])),hand_pt_expr.n_pts);
  %hand_on_ground_cnstr = ExpressionConstraint(lb,ub,compose(dist_to_ground_expr,hand_pt_expr));

  %com_expr = RelativePosition(rbm,0,'world',[0;0;0]);
  %weights_frame = frames.R(foot_pts_expr.n_pts);
  %support_polygon{1} = LinearCombination(foot_pts_expr.n_pts,R3);
  %support_polygon{1} = compose(support_polygon{1},[Identity(weights_frame);foot_pts_expr]);
  %support_polygon{1} = compose(Difference(R3),VertCat(Zeros(weights_frame,R3)+com_expr,support_polygon{1},true));
  %support_polygon{2} = Identity(weights_frame);
  %support_polygon{3} = Affine(weights_frame,frames.R(1),ones(1,foot_pts_expr.n_pts),0);
  %qsc_cnstr{1} = ExpressionConstraint([0;0;-Inf],[0;0;Inf],support_polygon{1});
  %qsc_cnstr{2} = ExpressionConstraint(zeros(foot_pts_expr.n_pts,1),ones(foot_pts_expr.n_pts,1),support_polygon{2});
  %qsc_cnstr{3} = ExpressionConstraint(1,1,support_polygon{3});

  %foot_z_expr = RelativeVector(rbm,'l_foot','world',[0;0;1]);
  %foot_vertical_cnstr = ExpressionConstraint([0;0;1],[0;0;1],foot_z_expr);

  %Q = eye(numel(q_nom));
  %prog = InverseKinematics(rbm,q_nom);
  %prog = prog.setQ(Q);
  %q_inds = prog.q_idx;
  %w_inds = reshape(prog.num_vars+(1:foot_pts_expr.n_pts),[],1);
  %prog = prog.addDecisionVariable(numel(w_inds));

  %prog = prog.addConstraint(foot_on_ground_cnstr,q_inds);
  %prog = prog.addConstraint(foot_vertical_cnstr,q_inds);

  %prog = prog.addConstraint(hand_on_ground_cnstr,q_inds);

  %prog = prog.addConstraint(qsc_cnstr{1},[w_inds;q_inds]);
  %prog = prog.addConstraint(qsc_cnstr{2},w_inds);
  %prog = prog.addConstraint(qsc_cnstr{3},w_inds);

  %time_prog = tic;
  %[q,F,info] = solve(prog,q0);
  %toc(time_prog);

  %if visualize
    %kinsol = doKinematics(rbm,q);
    %com = getCOM(rbm,kinsol);
    %lcmgl.glColor3f(1,0,0);
    %lcmgl.sphere([com(1:2);0],0.02,20,20);
    %lcmgl.switchBuffers();
    %v.draw(0,q);
  %end
end
