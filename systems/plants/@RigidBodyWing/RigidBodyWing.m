classdef RigidBodyWing < RigidBodyForceElement

  properties
    kinframe;  % index to RigidBodyFrame 
    fCl  % PPTrajectories (splines) representing the *dimensional* coefficients
    fCd  % with fCl = 1/2 rho*S*Cl, etc. (S=area)
    fCm
    dfCl
    dfCd
    dfCm
    area
    %Air density for 20 degC dry air, at sea level
    rho = 1.204;
  end
  
  methods
    function obj = RigidBodyWing(frame_id, profile, chord, span, stallAngle, velocity)
      %calls AVL and XFOIL over different angles of attack at the
      %given velocity, generates first order polynomials of the CL,
      %CD, and pitch moments of the wing.  The axes for use in DRAKE are:
      %   X = forward, in the usual direction of travel
      %   Y = out the left side of the wing
      %   Z = up
      % Note: an sign inversion is needed for the moment coefficient, as a
      % positive pitching moment from AVL/XFOIL represents a pitch up, but
      % a positive y-axis rotation represents a pitch down.
      % @param frame_id = RigidBodyFrame specifying the location (and
      %         orientation) of the quarter-chord of the airfoil and the body which
      %         is it mounted on.
      % @param profile = Profile can either be a:
      %         -NACA airfoil designation starting with 'NACA'
      %         -File location of a .dat file generated by xfoil
      %         -File location of a .mat file which contains the
      %         variables 'CLSpline', 'CDSpline', 'CMSpline'
      %         -the words 'flat plate'
      % @param chord = the chord of the wing (meters)
      % @param span = the wing span (width) of the wing (meters)
      % @param stallAngle = user-defined angle at which the wing
      %          stalls (degrees).  The negative of the stall angle
      %          is used for AoA<0, even though airfoils do not stall
      %          at exactly the same abs(AoA).
      % @param velocity = the approximate operating velocity of the wing.
      %        Needed to pass in the correct Reynolds and Mach numbers to the
      %        to the numerical solvers to generate appropriate CL and CD
      %        splines.
      %
      %TODO:
      % Implement some values for post-stall CMs
      %     i.e. at 90 degrees aoa, the center of pressure should be
      %     at the half chord.
      % use variables for path to xfoil and AVL
      %
      
      typecheck(frame_id,'numeric');
      obj.kinframe = frame_id;
      linux = isunix();
      obj.area = chord*span;
      mach = velocity/341;
      %Reynolds number calculation for 10 deg C and sea level
      Re = velocity*chord/.0000144;
      profile = deblank(profile);
      disp('Constructing a new airfoil');
      setEnvVars();
      if strcmpi(profile(end-3:end), '.mat')
        load(profile, 'CLSpline', 'CDSpline', 'CMSpline')
        obj.fCl = CLSpline;
        obj.fCd = CDSpline;
        obj.fCm = CMSpline;
      elseif strcmpi(profile, 'flat plate')
        flatplate()
      else % not flat plate.  I.e. its either a NACA or .dat file
        checkDependency('avl');
        checkDependency('xfoil');
        conf = struct;
        load drake_config;
        avlpath = deblank(conf.avl);
        xfoilpath = deblank(conf.xfoil);
        
        if strcmp(profile(1:4),'NACA')
          profile = strtrim(profile(5:end));
          avlprofile = strcat('NACA', '\n', profile);
          xfoilprofile = strcat('NACA ', profile);
        elseif strcmpi(profile(end-3:end), '.dat') %Path to a .dat file
          if ~exist(profile)
              error('Cannot find wing input .dat file.  Please check file path')
          end
          % xfoil cannot handle long filenames (see bug 1734), so 
          % copy the profile to the tmp directory and call that instead.
          % yuck.
          [~,filename,fileext] = fileparts(profile);
          copyfile(which(profile),fullfile(tempdir,[filename,fileext]));
          profile = fullfile(tempdir,[filename,fileext])

          avlprofile = strcat('AFILE', '\n', profile);
          xfoilprofile = strcat('LOAD', '\n', profile);
        else
          err = MException('InputType:NotSupported', 'Input profile not supported. (.dat, .m, NACA, flat plate)');
          throw(err);
        end
        %{
                Reads in template avl files defining the geometry of the wing and the run
                profile (airspeed, AoA).  Then replaces the appropriate fields in the
                Template files with the correct numbers.
                The .0001 additions are to avoid repeated angles.
        %}
        angles = -stallAngle:stallAngle/12:(stallAngle-.0001);
        AoAs = [];
        CLs = [];
        CDs = [];
        CMs = [];
        prepAvlFile();
        runAvl();
        runXfoil();
        %at this point the laminar regime is done.  Now a flat
        %plate approximation is added on from -90 degrees to
        %-stall, and then from stall to +90 degrees
        disp('Laminar regime complete.  Adding post-stall points')
        addPostStallPoints();
        makeSplines();
        revertEnvVars();
      end % if not a flat plate
      function prepAvlFile()
          avlfile = fileread(which('avlblank.avl'));
          avlfile = regexprep(avlfile, '\$airfoil', avlprofile);
          avlfile = regexprep(avlfile, '\$span', sprintf('%.1f',span));
          avlfile = regexprep(avlfile, '\$chord', sprintf('%.1f',chord));
          avlfile = regexprep(avlfile, '\$area', sprintf('%.1f',obj.area));
          avlfile = regexprep(avlfile, '\$mach', sprintf('%.4f',mach));
          avlfile = regexprep(avlfile, '\$yle', sprintf('%.2f',span/2));
          avlfile = regexprep(avlfile, '\$Xorig', sprintf('%.2f',0));
          avlfile = regexprep(avlfile, '\$Yorig', sprintf('%.2f',0));
          avlfile = regexprep(avlfile, '\$Zorig', sprintf('%.2f',0));
          avlid = fopen(fullfile(tempdir,'URDF.avl'), 'w');
          fprintf(avlid, avlfile);
    end %prepAvlFile()
      function runAvl()
            %writes the case-specific files avl requires to run
            runid = fopen(fullfile(tempdir,'URDF.run'), 'w');
            avlcommandfile = fopen(fullfile(tempdir,'avlcommands.txt'), 'w');
            avlresultsloc = fullfile(tempdir,'avlresults.txt');
            if ~linux %because windows file paths use backslashes.
                avlresultsloc = regexprep(avlresultsloc, '\\', '\\\\');
            end
            avlcommandstring = sprintf('OPER\nX\nW\n%s\n', avlresultsloc);
            runfiletemplate = fileread(which('avlblank.run'));
            for x = 1:length(angles)
              runfile = regexprep(runfiletemplate, '\$casenum', sprintf('%d',x));
              runfile = regexprep(runfile, '\$alpha', sprintf('%.2f',angles(x)));
              runfile = regexprep(runfile, '\$mach', sprintf('%.1f',mach));
              runfile = regexprep(runfile, '\$vel', sprintf('%.1f',velocity));
              runfile = regexprep(runfile, '\$span', sprintf('%.1f',span));
              fprintf(runid, runfile);
              if x ~=1 % case 1 is already taken care of above.  Need to do this way to get the output filename correct
                avlcommandstring = [avlcommandstring, sprintf('%d\nX\nW\n\n', x)];
              end
            end
            avlcommandstring = [avlcommandstring, sprintf('\nquit')];
            fprintf(avlcommandfile, avlcommandstring);
            fclose(avlcommandfile);
            fclose(runid);
            if exist(fullfile(tempdir,'avlresults.txt'), 'file');
                disp('clearing old AVL results file');
                delete(fullfile(tempdir,'avlresults.txt'));
            end
            disp('AVL Config file written--running AVL...');
            % runs AVL.  This generates results.txt, which CL, Cm, and part of CD is
            % extracted from.
            try
              commandfilepath = fullfile(tempdir,'avlcommands.txt');
              avlfilepath = fullfile(tempdir, 'URDF.avl');
              runfilepath = fullfile(tempdir, 'URDF.run');

              commandstring = sprintf('%s %s %s < %s > %s', avlpath, avlfilepath, runfilepath, commandfilepath, fullfile(tempdir,'avlScreenOutput.txt'));
              result = systemWCMakeEnv(commandstring);
              
            catch E
              disp('Error running AVL.  Switching to Flat Plate.  Results likely inaccurate')
              flatplate()
              return
            end
            if result ~= 0 || ~ exist(fullfile(tempdir,'avlresults.txt'), 'file');%if AVL didn't execute properly
              warning('Error running AVL. The system() call did not execute properly.  Switching to Flat Plate.  Results likely inaccurate')
              flatplate()
              return
            end
            disp('Processing AVL output...')
            avlresult = fileread(fullfile(tempdir,'avlresults.txt'));
            AoAindices = strfind(avlresult, 'Alpha =');
            AoAindices = AoAindices + 8;
            for x = 1:length(AoAindices)
              AoAs = [AoAs str2double(avlresult(AoAindices(x):AoAindices(x)+6))];
            end
            CLindices = strfind(avlresult, 'CLtot =');
            CLindices = CLindices + 8;
            for x = 1:length(CLindices)
              CLs = [CLs str2double(avlresult(CLindices(x):CLindices(x)+6))];
            end
            CDindices = strfind(avlresult, 'CDtot =');
            CDindices = CDindices + 8;
            for x = 1:length(CDindices)
              CDs = [CDs str2double(avlresult(CDindices(x):CDindices(x)+6))];
            end
            Cmindices = strfind(avlresult, 'Cmtot =');
            Cmindices = Cmindices + 8;
            for x = 1:length(Cmindices)
              CMs = [CMs str2double(avlresult(Cmindices(x):Cmindices(x)+6))];
            end
        end %runAvl()
      function runXfoil()
            % Reads template Xfoil commands file and fills in appropriate values
            xfoilfile = fileread(which('xfoilblank.txt'));
            polarLoc = fullfile(tempdir,'xfoilPolar.txt');
            if ~linux %because Windows.
                polarLoc = regexprep(polarLoc, '\\', '\\\\\\\\');
            end
            xfoilfile = regexprep(xfoilfile, '\$airfoil', xfoilprofile);
            xfoilfile = regexprep(xfoilfile, '\$re', sprintf('%.2f',Re));
            xfoilfile = regexprep(xfoilfile, '\$mach', sprintf('%.4f',mach));
            xfoilfile = regexprep(xfoilfile, '\$negStallAngle', sprintf('%.1f',-stallAngle));
            xfoilfile = regexprep(xfoilfile, '\$stallAngle', sprintf('%.1f',stallAngle));
            xfoilfile = regexprep(xfoilfile, '\$polarLocation', polarLoc);
            xfoilid = fopen(fullfile(tempdir,'xfoilcommands.txt'), 'w');
            fprintf(xfoilid, xfoilfile);
            fclose(xfoilid);
            %runs Xfoil.
            disp('Xfoil Commands written. Running XFOIL...')
            try
              if exist(fullfile(tempdir,'xfoilPolar.txt'), 'file');
                disp('Clearing old Xfoil results');
                delete(fullfile(tempdir,'xfoilPolar.txt'));
              end
              
              %system(sprintf('LD_LIBRARY_PATH=/usr/lib/gcc/x86_64-linux-gnu/4.6 %s < xfoilcommands.txt > xfoilCMDoutput.txt', xfoilpath));
              commandstring = sprintf('%s < %s > %s', xfoilpath, fullfile(tempdir,'xfoilcommands.txt'), fullfile(tempdir,'xfoilCMDoutput.txt'));
              result = systemWCMakeEnv(commandstring);
              
              disp('Processing Xfoil output')
              xfoilresult = fopen(fullfile(tempdir,'xfoilPolar.txt'));
              xfoillines = textscan(xfoilresult, '%[^\r\n]');
              fclose(xfoilresult);
              %Strips down the output so its just a list of the alpha,
              %CL, CD, CDp, CM numbers from xfoil.  The while loop should run
              %~6 times
              xfoillines = xfoillines{1}(1:end);
              while ~strcmp(xfoillines{1}(1:5), '-----')
                xfoillines = xfoillines(2:end);
              end
              xfoillines = xfoillines(2:end);
            catch E
              disp('Warning: Error in running XFOIL. Drag forces pre-stall are likely underestimated. Check the location of your wing data file?')
              addPostStallPoints()
              makeSplines()
              return
            end
            if result ~= 0 %if XFOIL didn't execute properly
              disp('Warning: Error in running XFOIL. Drag forces pre-stall are likely underestimated')
              addPostStallPoints()
              makeSplines()
              return
            end

            Cds = [];
            Cls = [];
            alphas = [];
            for x = 1:length(xfoillines)
              currline = textscan(xfoillines{x}, '%f');
              currline = currline{1};
              alphas = [alphas currline(1)];
              Cls = [Cls currline(2)];
              Cds = [Cds currline(3)];
            end
            %{
                    xfoil runs from 0 to negstallangle, then resets and runs
                    from 0 to stallangle to promote convergence of solutions.
                    (If it started at negstallangle, it may not converge if
                    the wing is stalled, so it starts from 0 and works its way
                    outwards.)  This creates improper ordering of the output
                    file, which these next four lines take care of.
                    reorders the matricies from -stallangle to stallangle
            %}

            [~,pivot] = max(diff(alphas));
            %Xfoil on my lab machine would have two zero data points, which would
            %break spline generation.  If there are two zeros, then skip the
            %second one. I'm not sure why this happens.  Something in the 
            %Xfoil install? --Tim  
            if length(find(alphas==0))==2
                inc = 2;
            else
                inc = 1;
            end
            alphas = [fliplr(alphas(1:pivot)) alphas(pivot+inc:end)];
            Cls = [fliplr(Cls(1:pivot)) Cls(pivot+inc:end)];
            Cds = [fliplr(Cds(1:pivot)) Cds(pivot+inc:end)];
            [~, maxloc] = max(Cls);
            if alphas(maxloc)+1 < stallAngle
              disp('Warning: Wing stall detected earlier than the user-specified stall')
            end
            xfoilspline = spline(alphas, Cds);
            %Add the xfoil Cd to the AVL Cd
            try
              for x = 1:length(AoAs)
                CDs(x) = CDs(x) + ppval(xfoilspline, AoAs(x));
              end
            catch E
              disp('Warning: Error in matching up XFOIL Cds. Drag forces are likely underestimated')
            end
            %output piped to a file to avoid cluttering
            %Matlab's screen output
            delete(fullfile(tempdir,'xfoilCMDoutput.txt'));
        end %runXfoil()
      function addPostStallPoints()
        postStallAngles = stallAngle+2:2:180;
        postStallCLs = 2*sind(postStallAngles).*cosd(postStallAngles);
        postStallCDs = 2*sind(postStallAngles).^2;
        postStallCMs = -2*postStallAngles./(90*4);
        AoAs = [-fliplr(postStallAngles) AoAs postStallAngles];
        CLs = [-fliplr(postStallCLs) CLs postStallCLs];
        CDs = [fliplr(postStallCDs) CDs postStallCDs];
        CMs = [-fliplr(postStallCMs) CMs postStallCMs];
      end
      function flatplate()
        disp('Using a flat plate airfoil.')
        laminarpts = 30;
        angles = [-180:2:-(stallAngle+.0001) -stallAngle:2*stallAngle/laminarpts:(stallAngle-.0001) stallAngle:2:180];
        %CMangles is used to make the Moment coefficient zero when the wing
        %is not stalled
        CMangles = [-180:2:-(stallAngle+.0001) zeros(1,laminarpts) stallAngle:2:180];
        obj.fCm = foh(angles, -(CMangles./90)*obj.rho*obj.area*chord/4);
        obj.fCl = spline(angles, .5*(2*sind(angles).*cosd(angles))*obj.rho*obj.area);
        obj.fCd = spline(angles, .5*(2*sind(angles).^2)*obj.rho*obj.area);
      end
      function makeSplines()
        %Dimensionalized splines, such that you only need to
        %multiply by vel^2.  Lift = .5*Cl*rho*area*vel^2
        obj.fCl = spline(AoAs, .5*CLs*obj.rho*obj.area);
        obj.fCd = spline(AoAs, .5*CDs*obj.rho*obj.area);
        obj.fCm = spline(AoAs, .5*CMs*obj.rho*obj.area*chord);
        disp('Aerodynamic Splines Finished')
      end
      %These are needed in order to make avl and xfoil run correctly.  Not
      %entirely sure what the underlying problem is, but this was the
      %workaround from mathworks.com.
      function setEnvVars()
          setenv('GFORTRAN_STDIN_UNIT', '5') 
          setenv('GFORTRAN_STDOUT_UNIT', '6') 
          setenv('GFORTRAN_STDERR_UNIT', '0')
      end %setEnvVars()
      function revertEnvVars()
          setenv('GFORTRAN_STDIN_UNIT', '-1') 
          setenv('GFORTRAN_STDOUT_UNIT', '-1') 
          setenv('GFORTRAN_STDERR_UNIT', '-1')
      end %revertEnvVars()
      
      % convert the splines to PPTrajectory, allowing
      % for fasteval improving performance a lot
      obj.fCm = PPTrajectory(obj.fCm);
      obj.fCl = PPTrajectory(obj.fCl);
      obj.fCd = PPTrajectory(obj.fCd);
      obj.dfCm = obj.fCm.fnder(1);
      obj.dfCl = obj.fCl.fnder(1);
      obj.dfCd = obj.fCd.fnder(1);
      
    end %constructor

    function [force, dforce] = computeSpatialForce(obj,manip,q,qd)
      frame = getFrame(manip,obj.kinframe);
      if (nargout>1)
        % enable second order gradients
        kinsol = doKinematics(manip,q,true);
        [~,J,dJ] = forwardKin(manip,kinsol,obj.kinframe,zeros(3,1));
      else
        kinsol = doKinematics(manip,q);
        [~,J] = forwardKin(manip,kinsol,obj.kinframe,zeros(3,1));
      end
      wingvel_world = J*qd; % assume still air. Air flow over the wing

      % Implementation note: for homogenous transforms, I could do the following 
      % vector transforms more efficiently.  forwardKin is adding a 1 on 
      % the end of every pt for the homogenous coordinates.  it would be 
      % equivalent, cleaner, and more efficient, to just add a zero on the
      % end instead of the 1...  because for homogeneous transform matrix 
      % T we have:
      %   T * [x;1] - T * [0;1] = T * [x;0]
      % but I made this change everywhere and decided it was more important
      % to keep the interface to the forwardKin method clean instead of
      % polluting it (and bodyKin, and the mex files, ...) with an extra
      % input/option for "vectors_not_points".
      
      %project this onto the XZ plane of the wing (ignores sideslip)
      wingYunit = forwardKin(manip,kinsol,obj.kinframe,[0 0; 1 0; 0 0]); wingYunit = wingYunit(:,1)-wingYunit(:,2);
      sideslip = dot(wingvel_world,wingYunit);
      wingvel_world = wingvel_world - sideslip*wingYunit;
      
      if (nargout>1)
        [wingYunit,dwingYunitdq] = forwardKin(manip,kinsol,obj.kinframe,[0 0; 1 0; 0 0]);
      else
        wingYunit = forwardKin(manip,kinsol,obj.kinframe,[0 0; 1 0; 0 0]);
      end
      wingYunit = wingYunit(:,1)-wingYunit(:,2);
      sideslip = dot(wingvel_world,wingYunit);
      wingvel_world = wingvel_world - sideslip*wingYunit;
      
      if (nargout>1) 
        [wingvel_rel,wingvel_relJ,wingvel_relP] = bodyKin(manip,kinsol,obj.kinframe,[wingvel_world,zeros(3,1)]);
      else
        wingvel_rel = bodyKin(manip,kinsol,obj.kinframe,[wingvel_world,zeros(3,1)]);
      end
      wingvel_rel = wingvel_rel(:,1)-wingvel_rel(:,2);
      
      airspeed = norm(wingvel_world);
      aoa = -(180/pi)*atan2(wingvel_rel(3),wingvel_rel(1));
      
      % mod 360 so -180<AoA<180 Shouldn't be necessary because of acosd
      if aoa>180
        aoa = aoa-360*fix((aoa+180)/360);
      elseif aoa<-180
        aoa = aoa-360*fix((aoa-180)/360);
      end
      
      force = sparse(6,getNumBodies(manip));
      
      %lift and drag are the forces on the body in the world frame.
      %cross(wingXZvelocity, wingYunit) rotates it by 90 degrees
      if (nargout>1)
        [CL, CD, CM, dCL, dCD, dCM] = obj.coeffs(aoa);
      else
        [CL, CD, CM] = obj.coeffs(aoa);
      end
      % cross is a rather expensive operation, store it for gradients
      x_wingvel_world_wingYunit = cross(wingvel_world, wingYunit);
      lift_world = CL*airspeed*x_wingvel_world_wingYunit;
      drag_world = CD*airspeed*(-wingvel_world);
      torque_world = -CM*airspeed*airspeed*wingYunit;

      % convert torque to joint frame (featherstone dynamics algorithm never reasons in body coordinates)
      if (nargout>1)  
        [torque_body, torque_bodyJ, torque_bodyP] = bodyKin(manip,kinsol,frame.body_ind,[torque_world,zeros(3,1)]);
      else
        torque_body = bodyKin(manip,kinsol,frame.body_ind,[torque_world,zeros(3,1)]);
      end
      torque_body = torque_body(:,1)-torque_body(:,2);
      
      torque_joint = manip.body(frame.body_ind).X_joint_to_body'*[torque_body;0;0;0];
      
      %inputs of point (body coordinates), and force (world coordinates)
      %returns [torque; xforce; yforce] in the body coordinates
      %obj.body.dofnum should have 6 elements for
      %linkID = manip.findLinkInd(obj.body.linkname, 0);
      if (nargout>1)
        [f,fJ,fP] = cartesianForceToSpatialForce(manip, kinsol, frame.body_ind, frame.T(1:3,4),lift_world+drag_world);
      else
        f = cartesianForceToSpatialForce(manip, kinsol, frame.body_ind, frame.T(1:3,4),lift_world+drag_world);
      end 

      force(:,frame.body_ind) = torque_joint + f;
        
      if (nargout>1)
        nq = size(q,1);
        dwingvel_worlddq = reshape(reshape(dJ,[],nq)*qd,[],nq);
        dwingvel_worlddqd = J;
        dwingYunitdq = dwingYunitdq(1:3,:)-dwingYunitdq(4:6,:); 
        dwingYunitdqd = zeros(3,nq);
        dsideslipdq = dot(dwingvel_worlddq,repmat(wingYunit,1,nq),1)+dot(repmat(wingvel_world,1,nq),dwingYunitdq,1);
        dsideslipdqd = dot(dwingvel_worlddqd,repmat(wingYunit,1,nq),1)+dot(repmat(wingvel_world,1,nq),dwingYunitdqd,1);
        dwingvel_worlddq = dwingvel_worlddq - wingYunit*dsideslipdq - sideslip*dwingYunitdq;
        dwingvel_worlddqd = dwingvel_worlddqd - wingYunit*dsideslipdqd - sideslip*dwingYunitdqd;
        wingvel_relJ = wingvel_relJ(1:3,:)-wingvel_relJ(4:6,:);
        wingvel_relP = wingvel_relP(1:3,:)-wingvel_relP(4:6,:);
        dwingvel_reldq = wingvel_relJ+wingvel_relP(:,1:3)*dwingvel_worlddq;
        dwingvel_reldqd = wingvel_relP(:,1:3)*dwingvel_worlddqd;
        dairspeeddq = (dwingvel_worlddq'*wingvel_world)'/norm(wingvel_world);
        dairspeeddqd = (dwingvel_worlddqd'*wingvel_world)'/norm(wingvel_world);
        daoadq = -(180/pi)*(wingvel_rel(1)*dwingvel_reldq(3,:)-wingvel_rel(3)*dwingvel_reldq(1,:))/(wingvel_rel(1)^2+wingvel_rel(3)^2);
        daoadqd = -(180/pi)*(wingvel_rel(1)*dwingvel_reldqd(3,:)-wingvel_rel(3)*dwingvel_reldqd(1,:))/(wingvel_rel(1)^2+wingvel_rel(3)^2);
        dforce = sparse(6,getNumBodies(manip)*2*nq);
        dlift_worlddq = dCL*airspeed*(x_wingvel_world_wingYunit*daoadq) + CL*(x_wingvel_world_wingYunit*dairspeeddq) + CL*airspeed*(cross(dwingvel_worlddq, repmat(wingYunit,1,nq), 1)+cross(repmat(wingvel_world,1,nq), dwingYunitdq, 1));
        dlift_worlddqd = dCL*airspeed*(x_wingvel_world_wingYunit*daoadqd) + CL*(x_wingvel_world_wingYunit*dairspeeddqd) + CL*airspeed*(cross(dwingvel_worlddqd, repmat(wingYunit,1,nq), 1));
        ddrag_worlddq = dCD*airspeed*(-wingvel_world*daoadq) + CD*(-wingvel_world*dairspeeddq) + CD*airspeed*-dwingvel_worlddq;
        ddrag_worlddqd = dCD*airspeed*(-wingvel_world*daoadqd) + CD*(-wingvel_world*dairspeeddqd) + CD*airspeed*-dwingvel_worlddqd;
        dtorque_worlddq = -dCM*airspeed*airspeed*(wingYunit*daoadq) + -CM*2*airspeed*(wingYunit*dairspeeddq) + -CM*airspeed*airspeed*dwingYunitdq;
        dtorque_worlddqd = -dCM*airspeed*airspeed*(wingYunit*daoadqd) + -CM*2*airspeed*(wingYunit*dairspeeddqd);
        torque_bodyJ = torque_bodyJ(1:3,:)-torque_bodyJ(4:6,:);
        torque_bodyP = torque_bodyP(1:3,:)-torque_bodyP(4:6,:);
        dtorque_bodydq = torque_bodyJ+torque_bodyP(1:3,1:3)*dtorque_worlddq;
        dtorque_bodydqd = torque_bodyP(1:3,1:3)*dtorque_worlddqd;
        dtorque_jointdq = manip.body(frame.body_ind).X_joint_to_body'*[dtorque_bodydq;zeros(3,nq)];
        dtorque_jointdqd = manip.body(frame.body_ind).X_joint_to_body'*[dtorque_bodydqd;zeros(3,nq)];
        dfdq = fJ+fP(1:6,1:6)*[(dlift_worlddq+ddrag_worlddq);zeros(3,nq)];
        dfdqd = fP(1:6,1:6)*[(dlift_worlddqd+ddrag_worlddqd);zeros(3,nq)];
        dforce_bodydq = dtorque_jointdq + dfdq;
        dforce_bodydqd = dtorque_jointdqd + dfdqd;
        dforcebody = [dforce_bodydq, dforce_bodydqd];
        numbodies = getNumBodies(manip);
        for i=1:2*nq
          dforce(:,(i-1)*numbodies+frame.body_ind) = dforcebody(:,i);
        end
      end
    
    end
        
    function [CL, CD, CM, dCL, dCD, dCM] = coeffs(obj, aoa)
      %returns dimensionalized coefficient of lift, drag, and pitch moment for a
      %given angle of attack
      CL = obj.fCl.fasteval(aoa);
      CD = obj.fCd.fasteval(aoa);
      CM = obj.fCm.fasteval(aoa);
      if (nargout>3)
        dCL = obj.dfCl.fasteval(aoa);
        dCD = obj.dfCd.fasteval(aoa);
        dCM = obj.dfCm.fasteval(aoa);
      end
    end
    
  end
  
  methods (Static)
    function [model,obj] = parseURDFNode(model,robotnum,node,options)
      name = char(node.getAttribute('name'));
      name = regexprep(name, '\.', '_', 'preservecase');
      
      elNode = node.getElementsByTagName('parent').item(0);
      parent = findLinkInd(model,char(elNode.getAttribute('link')),robotnum);
      
      xyz=zeros(3,1); rpy=zeros(3,1);
      elnode = node.getElementsByTagName('origin').item(0);
      if ~isempty(elnode)
        if elnode.hasAttribute('xyz')
          xyz = reshape(parseParamString(model,robotnum,char(elnode.getAttribute('xyz'))),3,1);
        end
        if elnode.hasAttribute('rpy')
          rpy = reshape(parseParamString(model,robotnum,char(elnode.getAttribute('rpy'))),3,1);
          if any(rpy), error('rpy not implemented yet for wings'); end
        end
      end
      [model,frame_id] = addFrame(model,RigidBodyFrame(parent,xyz,rpy,[name,'_frame']));
      
      profile = char(node.getAttribute('profile'));
      chord = parseParamString(model,robotnum,char(node.getAttribute('chord')));
      span = parseParamString(model,robotnum,char(node.getAttribute('span')));
      stall_angle = parseParamString(model,robotnum,char(node.getAttribute('stall_angle')));
      nominal_speed = parseParamString(model,robotnum,char(node.getAttribute('nominal_speed')));
      
      obj = RigidBodyWing(frame_id, profile, chord, span, stall_angle, nominal_speed);
      
      obj.name = name;
    end
  end
  
end
