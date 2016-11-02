function constraintTester(testName,r,makeCon,makeQnom,makeQseed,n,draw_pause,user_options,objFun,use_mex_kinematics)
% @param testName   string
% @param r          RigidBodyManipulator object 
% @param makeCon    Function handle of the form con = makeCon(r), where con
%                   is an IKconstraint object
% @param makeQnom   Function handle of the form q_nom = makeQnom(r)
% @param makeQseed  Function handle of the form q_seed = makeQseed(r)
% @param n          Number of test iterations @default 10

  if (nargin < 10) || isempty(use_mex_kinematics)
    use_mex_kinematics = true;
  end
  if (nargin < 9) || isempty(objFun)
    objFun = @defaultObjFun;
  end
  if (nargin < 8) || isempty(user_options)
    user_options = [];
  end
  if (nargin < 7) || isempty(draw_pause)
    draw_pause = 0.05;
  end
  if (nargin < 6) || isempty(n)
    n = 10;
  end
  
  checkDependency('lcmgl');
  lcmgl = drake.matlab.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),testName);
  v = r.constructVisualizer();

  problem.solver = 'fmincon';
  problem.options=optimset('GradConstr','on','Algorithm','interior-point','Display','iter','TolX',1e-14,'TolCon',1e-9,'MaxFunEvals',5000);
  problem.options=optimset(problem.options,'TolFun',1e3);
  problem.options=optimset(problem.options,'Display','off');
  problem.options=optimset(problem.options,user_options);

  for i = 1:n
    con = makeCon(r);
    q_nom = makeQnom(r);
    q_seed = makeQseed(r);
    [lb,ub] = bounds(con,0);

    problem.objective = @objFunWrapper;  % fmincon doesn't like not having an objective.
    %problem.objective = @(q) (q-q_seed)'*(q-q_seed);  % fmincon doesn't like not having an objective.
    % We set TolFun to a large value below so that
    % we aren't really trying for optimality.
    
    problem.nonlcon = @(q) mycon(r,q,con,lb,ub,use_mex_kinematics);
    %c = problem.nonlcon(randn(r.getNumPositions(),1));  % call it once to make sure it doesn't crash

    problem.options = optimset(problem.options,'OutputFcn', ...
      @(q,optVal,state) drawme(v,q,con,lcmgl,draw_pause));
    problem.x0 = q_seed;

    drawme(v,q_seed,con,lcmgl,10*draw_pause);
    [qsol,~,exitflag] = fmincon(problem);
    success=(exitflag==1) || (exitflag==2);
    v.draw(0,[qsol;0*qsol]);

    if (~success)
      if (exitflag==0) % fmincon reached maximum iteration limit
        warning('constraintTester:fmincon_max_iter', ...
          ['%s stopped because fmincon reached its maximum number of ' ... 
           'function evaluations. This may happen occaisonally due to ' ...
           'difficult initial conditions.'],testName);
      elseif (exitflag==-2) % fmincon failed to find feasible solution
        warning('constraintTester:fmincon_failure', ...
          ['%s stopped because fmincon failed to find a feasible solution. ' ...
           'This may happen occaisonally due to difficult initial ' ...
           'conditions.'],testName);
      else
        error('%s: fmincon failed with exitflag = %d',testName,exitflag);
      end
    end
  end

  function [f, Gf] = objFunWrapper(q)
    [f, Gf] = objFun(q,q_nom,q_seed);
  end

  function [f, Gf] = defaultObjFun(q,q_nom,q_seed)
    f = (q-q_nom)'*(q-q_nom);
    Gf = 2*(q-q_nom);
  end

end

function [c,ceq,GC,GCeq] = mycon(r,q,con,lb,ub,use_mex_kinematics)
  ceq=[]; GCeq=[];
  kinsol = doKinematics(r,q,false,use_mex_kinematics);
  [c,GC] = eval(con,0,kinsol);
  c = max([lb-c;c-ub],-1);
  GC = [-GC',GC'];
end

function stop=drawme(v,q,con,lcmgl,draw_pause)
  stop=false;
  con.drawConstraint(q,lcmgl);
  v.draw(0,[q; 0*q]);
  pause(draw_pause)
end
%NOTEST
