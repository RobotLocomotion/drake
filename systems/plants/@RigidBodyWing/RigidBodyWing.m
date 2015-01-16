classdef RigidBodyWing < RigidBodyForceElement
  % This class implements a wing element, supporting flat plates and
  % airfoils defined with NACA numbers, a geometry profile (with xfoil and
  % AVL backend) and .mat files defining wing performance
  
  properties
    fCl  % PPTrajectories (splines) representing the *dimensional* coefficients
    fCd  % with fCl = 1/2 rho*S*Cl, etc. (S=area)
    fCm
    dfCl
    dfCd
    dfCm
    area
    %Air density for 20 degC dry air, at sea level
    rho = 1.204;
    has_control_surface = false;
    span;
    stall_angle;
    chord;
    profile;
    visual_geometry; % true for automatic drawing of geometry from URDF
    parent_id; % parent id from URDF
    airfoil_needs_update = true; % flag that tells us if we need to create a new airfoil on compile
    velocity; % approximate operating velocity of the wing
    kinframe;
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
      %          <pre>
      %         -NACA airfoil designation starting with 'NACA'
      %         -File location of a .dat file generated by xfoil
      %         -File location of a .mat file which contains the
      %         variables 'CLSpline', 'CDSpline', 'CMSpline'
      %         -the words 'flat plate'
      %         </pre>
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
      
      % we need to be able to construct with no arguments per 
      % http://www.mathworks.com/help/matlab/matlab_oop/class-constructor-methods.html#btn2kiy
      if (nargin == 0)
        return;
      end

      typecheck(frame_id,'numeric');
      obj.kinframe = frame_id;
      
      obj.area = chord*span;
      obj.chord = chord;
      obj.span = span;
      obj.stall_angle = stallAngle;
      obj.profile = deblank(profile);
      obj.velocity = velocity;
      
      
    end
    
    function [obj, model] = onCompile(obj, model)
      % Runs on model compilation, constructs an airfoil if one does not
      % exist with these parameters
      %
      % @param model model we are a part of
      %
      % @retval obj updated object
      % @retval model updated model. you need to put the updated object
      % back into this model on return from this function
      
      if (obj.airfoil_needs_update == false)
        return;
      end
      
      model = obj.updateVisualGeometry(model);
      profile = obj.profile;
      velocity = obj.velocity;
      stallAngle = obj.stall_angle;
      chord = obj.chord;
      span = obj.span;
        
      linux = isunix();
      mach = velocity/341; % mach 1 at sea level is about 341 m/s
      
      %Reynolds number calculation for 10 deg C and sea level
      Re = velocity*obj.chord/.0000144;
      
      
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
          profile = fullfile(tempdir,[filename,fileext]);

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
      
      
      function runAvl()
        checkDependency('avl');
        avlpath = deblank(getCMakeParam('avl'));

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
        filename = tempname;
        avl_cleanup = onCleanup(@()delete([filename,'*']));
        avlfilepath = [filename,'.avl'];
        avlid = fopen(avlfilepath, 'w');
        fprintf(avlid, avlfile);
        fclose(avlid);
        
        %writes the case-specific files avl requires to run
        runfilepath = [filename,'.run'];
        runid = fopen(runfilepath, 'w');
        commandfilepath = [filename,'_command.txt'];
        avlcommandfile = fopen(commandfilepath, 'w');
        avlresultsloc = [filename,'_results.txt'];
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
        
        disp('Running AVL...');
        % runs AVL.  This generates results.txt, which CL, Cm, and part of CD is
        % extracted from.
        try
          commandstring = sprintf('%s %s %s < %s > %s', avlpath, avlfilepath, runfilepath, commandfilepath, [filename,'_screen_output.txt']);
          result = systemWCMakeEnv(commandstring);
        catch E
          disp('Error running AVL.  Switching to Flat Plate.  Results likely inaccurate')
          flatplate()
          return
        end
        if result ~= 0 || ~ exist(avlresultsloc, 'file');%if AVL didn't execute properly
          warning('Error running AVL. The system() call did not execute properly.  Switching to Flat Plate.  Results likely inaccurate')
          flatplate()
          return
        end
        %            disp('Processing AVL output...')
        avlresult = fileread(avlresultsloc);
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
        checkDependency('xfoil');
        xfoilpath = deblank(getCMakeParam('xfoil'));
        
        % Reads template Xfoil commands file and fills in appropriate values
        xfoilfile = fileread(which('xfoilblank.txt'));
        filename = tempname;
        xfoil_cleanup = onCleanup(@()delete([filename,'*']));
        polarLoc = [filename,'_polar.txt'];
        if ~linux %because Windows.
          polarLoc = regexprep(polarLoc, '\\', '\\\\\\\\');
        end
        xfoilfile = regexprep(xfoilfile, '\$airfoil', xfoilprofile);
        xfoilfile = regexprep(xfoilfile, '\$re', sprintf('%.2f',Re));
        xfoilfile = regexprep(xfoilfile, '\$mach', sprintf('%.4f',mach));
        xfoilfile = regexprep(xfoilfile, '\$negStallAngle', sprintf('%.1f',-stallAngle));
        xfoilfile = regexprep(xfoilfile, '\$stallAngle', sprintf('%.1f',stallAngle));
        xfoilfile = regexprep(xfoilfile, '\$polarLocation', polarLoc);
        commandfile = [filename,'_commands.txt'];
        xfoilid = fopen(commandfile, 'w');
        fprintf(xfoilid, xfoilfile);
        fclose(xfoilid);
        %runs Xfoil.
        disp('Running XFOIL...')
        try
          commandstring = sprintf('%s < %s > %s', xfoilpath, commandfile, [filename,'_screen_output.txt']);
          result = systemWCMakeEnv(commandstring);
          
          %              disp('Processing Xfoil output')
          xfoilresult = fopen(polarLoc);
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
      nq = size(q,1);
      frame = getFrame(manip,obj.kinframe);
      
      kinsol = doKinematics(manip,q,true,true,qd);

      if (nargout > 1)

        [ wingvel_struct, wingvel_dstruct ] = RigidBodyWing.computeWingVelocity(obj.kinframe, manip, q, qd, kinsol);
        
        wingvel_world_xz = wingvel_struct.wingvel_world_xz;
        wingYunit = wingvel_struct.wingYunit;
        wingvel_world_xyz = wingvel_struct.wingvel_world_xyz;
        
        dwingvel_worlddq = wingvel_dstruct.dwingvel_worlddq;
        dwingvel_worlddqd = wingvel_dstruct.dwingvel_worlddqd;
        dwingYunitdq = wingvel_dstruct.dwingYunitdq;
        dwingYunitdqd = wingvel_dstruct.dwingYunitdqd;

      else
        wingvel_struct = RigidBodyWing.computeWingVelocity(obj.kinframe, manip, q, qd, kinsol);
        
        wingvel_world_xz = wingvel_struct.wingvel_world_xz;
        wingYunit = wingvel_struct.wingYunit;
        wingvel_world_xyz = wingvel_struct.wingvel_world_xyz;
        
      end

      if (nargout>1)
        [wingvel_rel, dwingvel_reldq, dwingvel_reldqd] = RigidBodyWing.computeWingVelocityRelative(obj.kinframe, manip, kinsol, wingvel_struct, wingvel_dstruct);
      else
        wingvel_rel = RigidBodyWing.computeWingVelocityRelative(obj.kinframe, manip, kinsol, wingvel_struct);
      end

      airspeed = norm(wingvel_world_xz);
      if (nargout>1)
        dairspeeddq = (wingvel_world_xz'*dwingvel_worlddq)/(norm(wingvel_world_xz)+eps);
        dairspeeddqd = (wingvel_world_xz'*dwingvel_worlddqd)/(norm(wingvel_world_xz)+eps);
      end

      aoa = -(180/pi)*atan2(wingvel_rel(3),wingvel_rel(1));
      if (nargout>1)
        daoadq = -(180/pi)*(wingvel_rel(1)*dwingvel_reldq(3,:)-wingvel_rel(3)*dwingvel_reldq(1,:))/(wingvel_rel(1)^2+wingvel_rel(3)^2+eps);
        daoadqd = -(180/pi)*(wingvel_rel(1)*dwingvel_reldqd(3,:)-wingvel_rel(3)*dwingvel_reldqd(1,:))/(wingvel_rel(1)^2+wingvel_rel(3)^2+eps);
      end

      %lift and drag are the forces on the body in the world frame.
      %cross(wingXZvelocity, wingYunit) rotates it by 90 degrees
      if (nargout>1)
        [CL, CD, CM, dCL, dCD, dCM] = obj.coeffs(aoa);
        dCLdq = dCL*daoadq;
        dCDdq = dCD*daoadq;
        dCMdq = dCM*daoadq;
        dCLdqd = dCL*daoadqd;
        dCDdqd = dCD*daoadqd;
        dCMdqd = dCM*daoadqd;
      else
        [CL, CD, CM] = obj.coeffs(aoa);
      end

      x_wingvel_world_wingYunit = cross(wingvel_world_xz, wingYunit);
      if (nargout>1)
        dx_wingvel_world_wingYunitdq = cross(dwingvel_worlddq, repmat(wingYunit,1,nq), 1) + cross(repmat(wingvel_world_xz,1,nq), dwingYunitdq, 1);
        dx_wingvel_world_wingYunitdqd = cross(dwingvel_worlddqd, repmat(wingYunit,1,nq), 1) + cross(repmat(wingvel_world_xz,1,nq), dwingYunitdqd, 1);
      end
      lift_world = CL*airspeed*x_wingvel_world_wingYunit; % this is norm(airspeed)^2 because it equals CL*airspeed* norm(wingvel_world_xz) * norm(a unit_vector) * sin(90 deg)
      drag_world = CD*airspeed*(-wingvel_world_xz);
      torque_world = -CM*airspeed*airspeed*wingYunit;
      if (nargout>1)
        dlift_worlddq = x_wingvel_world_wingYunit*airspeed*dCLdq + x_wingvel_world_wingYunit*CL*dairspeeddq + CL*airspeed*dx_wingvel_world_wingYunitdq;
        dlift_worlddqd = x_wingvel_world_wingYunit*airspeed*dCLdqd + x_wingvel_world_wingYunit*CL*dairspeeddqd + CL*airspeed*dx_wingvel_world_wingYunitdqd;
        ddrag_worlddq = airspeed*-wingvel_world_xz*dCDdq + CD*(-wingvel_world_xz*dairspeeddq) + CD*airspeed*-dwingvel_worlddq;
        ddrag_worlddqd = airspeed*-wingvel_world_xz*dCDdqd + CD*(-wingvel_world_xz*dairspeeddqd) + CD*airspeed*-dwingvel_worlddqd;
        dtorque_worlddq = -airspeed*airspeed*wingYunit*dCMdq + -CM*2*airspeed*(wingYunit*dairspeeddq) + -CM*airspeed*airspeed*dwingYunitdq;
        dtorque_worlddqd = -airspeed*airspeed*wingYunit*dCMdqd + -CM*2*airspeed*(wingYunit*dairspeeddqd);
      end

      % convert torque to joint frame (featherstone dynamics algorithm never reasons in body coordinates)
      if (nargout>1)
        [torque_body, torque_bodyP, torque_bodyJ] = bodyKin(manip,kinsol,frame.body_ind,[torque_world,zeros(3,1)]);
        torque_body = torque_body(:,1)-torque_body(:,2);
        torque_bodyJ = torque_bodyJ(1:3,:)-torque_bodyJ(4:6,:);
        torque_bodyP = torque_bodyP(1:3,:)-torque_bodyP(4:6,:);
        dtorque_bodydq = torque_bodyJ+torque_bodyP(1:3,1:3)*dtorque_worlddq;
        dtorque_bodydqd = torque_bodyP(1:3,1:3)*dtorque_worlddqd;
      else
        torque_body = bodyKin(manip,kinsol,frame.body_ind,[torque_world,zeros(3,1)]);
        torque_body = torque_body(:,1)-torque_body(:,2);
      end

      torque_joint = manip.body(frame.body_ind).X_joint_to_body'*[torque_body;0;0;0];
      if (nargout>1)
        dtorque_jointdq = manip.body(frame.body_ind).X_joint_to_body'*[dtorque_bodydq;zeros(3,nq)];
        dtorque_jointdqd = manip.body(frame.body_ind).X_joint_to_body'*[dtorque_bodydqd;zeros(3,nq)];
      end

      %inputs of point (body coordinates), and force (world coordinates)
      %returns [torque; xforce; yforce] in the body coordinates
      %obj.body.position_num should have 6 elements for
      %linkID = manip.findLinkInd(obj.body.linkname, 0);
      if (nargout>1)
        [f,fJ,fP] = cartesianForceToSpatialForce(manip, kinsol, frame.body_ind, frame.T(1:3,4),lift_world+drag_world);
        dfdq = fJ+fP*(dlift_worlddq+ddrag_worlddq);
        dfdqd = fP*(dlift_worlddqd+ddrag_worlddqd);
      else
        f = cartesianForceToSpatialForce(manip, kinsol, frame.body_ind, frame.T(1:3,4),lift_world+drag_world);
      end

      body_force = torque_joint + f;
      if (nargout>1)
        dbody_forcedq = dtorque_jointdq + dfdq;
        dbody_forcedqd = dtorque_jointdqd + dfdqd;
      end

      force = sparse(6,getNumBodies(manip))*q(1); % q(1) is for taylorvar
      force(:,frame.body_ind) = body_force;
      if (nargout>1)
        dforce = sparse(numel(force),2*nq)*q(1); % q(1) is for taylorvar
        dforce((frame.body_ind-1)*6+1:frame.body_ind*6,:) = [dbody_forcedq,dbody_forcedqd];
        dforce = reshape(dforce,6,[]);
      end

    end

    function [CL, CD, CM, dCL, dCD, dCM] = coeffs(obj, aoa)
      %returns dimensionalized coefficient of lift, drag, and pitch moment for a
      %given angle of attack
      CL = obj.fCl.eval(aoa);
      CD = obj.fCd.eval(aoa);
      CM = obj.fCm.eval(aoa);
      if (nargout>3)
        dCL = obj.dfCl.eval(aoa);
        dCD = obj.dfCd.eval(aoa);
        dCM = obj.dfCm.eval(aoa);
      end
    end
    
    function obj = addWingVisualShapeToBody(varargin)
      errorDeprecatedFunction('addWingVisualGeometryToBody');
    end
    
    function model = addWingVisualGeometryToBody(obj, model, body)
      % Adds a visual geometry of the wing to the model on the body given for
      % drawing the wing in a visualizer.
      %
      % @param model manipulator the wing is part of
      % @param body body to add the visual geometry to
      %
      % @retval model updated model

      wing_height = 0.01;


      box_size = [ obj.chord, obj.span, wing_height ];
      
      geometry = RigidBodyBox(box_size, model.getFrame(obj.kinframe).T);
      geometry.name = [ obj.name '_urdf_geometry' ];
      
      model = model.addVisualGeometryToBody(body, geometry);

    end
    
    function body = updateParams(body,poly,pval)
      % @override

      body.airfoil_needs_update = true;
      
      body = updateParams@RigidBodyElement(body, poly, pval);
      
    end
    
    function obj = updateVisualShapes(varargin)
      errorDeprecatedFunction('updateVisualGeometry');
    end
    
    function model = updateVisualGeometry(obj, model)
      % Update visual geometry.  This should be called after a parameter
      % update that may change the automatic drawing of geometry (ie the area
      % of the drag force is changed).
      %
      % @param model RigidBodyManipulator this is a part of
      %
      % @retval model updated model
      
      
      if (obj.visual_geometry)
        
        
        % remove any existing geometry for this body
        model = model.removeVisualGeometryFromBody(obj.parent_id, [ obj.name '_urdf_geometry' ]);
        
        % add new geometry
        model = addWingVisualGeometryToBody(obj, model, obj.parent_id);
        
        
      end
      
    end
    
    function body = bindParams(body,model,pval)
      % @override

      body.airfoil_needs_update = true;
      
      body = bindParams@RigidBodyElement(body,model,pval);

    end

  end
  
  
  
  

  methods (Static)
    
    function [ wingvel_struct, wingvel_dstruct ]= computeWingVelocity(kinframe, manip, q, qd, kinsol)
      % Computes the velcity of the wing in word coordinates
      %
      % @param kinframe frame id of the kinematic frame
      % @param manip manipulator we are a part of
      % @param q state vector
      % @param qd q-dot (time derivative of state vector)
      % @param kinsol solution from doKinematics
      %
      % @retval wingvel_struct which has the following values:
      %   <pre>
      %   wingvel_world_xz
      %     velocity of the wing in world coodinates
      %     projected onto the wing's XZ plane.  In other words, velocity with
      %     sideslip subtracted out.
      %   wingYunit
      %     unit vector along the wing's Y axis in world
      %     coordinates
      %   </pre>
      % @retval wingvel_dstruct which has the following values:
      %   <pre>
      %   dwingvel_worlddq
      %     derivative of wing velocity with respect to q
      %   dwingvel_worlddqd
      %     derivative of the wing velocity with respect to qdot
      %   wingvel_world_xzy
      %     velocity of the wing in world coordinates
      %   </pre>
      
      if (nargout>1)
        
        [~,J] = forwardKin(manip,kinsol,kinframe,zeros(3,1));
        Jdot = forwardJacDot(manip,kinsol,kinframe,zeros(3,1));
        wingvel_world_xyz = J*qd; % assume still air. Air flow over the wing
        dwingvel_worlddq = Jdot;
        dwingvel_worlddqd = J;
      else
        kinsol = doKinematics(manip,q);
        [~,J] = forwardKin(manip,kinsol,kinframe,zeros(3,1));
        wingvel_world_xyz = J*qd; % assume still air. Air flow over the wing
      end
      
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
      if (nargout>1)
        [wingYunit,dwingYunitdq] = forwardKin(manip,kinsol,kinframe,[0 0; 1 0; 0 0]);
        wingYunit = wingYunit(:,1)-wingYunit(:,2); % subtract out the origin
        dwingYunitdq = dwingYunitdq(1:3,:)-dwingYunitdq(4:6,:);
        dwingYunitdqd = zeros(3, size(q,1));
      else
        wingYunit = forwardKin(manip,kinsol,kinframe,[0 0; 1 0; 0 0]);
        wingYunit = wingYunit(:,1)-wingYunit(:,2); % subtract out the origin
      end
      
      sideslip = wingvel_world_xyz'*wingYunit;
      if (nargout>1)
        dsideslipdq = wingYunit'*dwingvel_worlddq + wingvel_world_xyz'*dwingYunitdq;
        dsideslipdqd = wingYunit'*dwingvel_worlddqd + wingvel_world_xyz'*dwingYunitdqd;
      end

      wingvel_world_xz = wingvel_world_xyz - sideslip*wingYunit;
      if (nargout>1)
        dwingvel_worlddq = dwingvel_worlddq - sideslip*dwingYunitdq - wingYunit*dsideslipdq;
        dwingvel_worlddqd = dwingvel_worlddqd - sideslip*dwingYunitdqd - wingYunit*dsideslipdqd;
      end
      
      wingvel_struct.wingvel_world_xz = wingvel_world_xz;
      wingvel_struct.wingYunit = wingYunit;
      wingvel_struct.wingvel_world_xyz = wingvel_world_xyz;
      
      if (nargout>1)
        wingvel_dstruct.dwingvel_worlddq = dwingvel_worlddq;
        wingvel_dstruct.dwingvel_worlddqd = dwingvel_worlddqd;
        wingvel_dstruct.dwingYunitdq = dwingYunitdq;
        wingvel_dstruct.dwingYunitdqd = dwingYunitdqd;
      end
      
    end
    
        
    function [wingvel_rel, dwingvel_reldq, dwingvel_reldqd] = computeWingVelocityRelative(kinframe, manip, kinsol, wingvel_struct, wingvel_dstruct)
      % Computes the relative wing velocity
      %
      % @param kinframe frame id (usually in obj.kinframe)
      % @param manip manipulator we are a part of
      % @param kinsol solution from doKinematics
      % @param wingvel_struct from computeWingVelocity
      % @param wingvel_dstruct from computeWingVelocity
      %
      % @retval wingvel_rel relative wing velocity
      % @retval dwingvel_reldq derivative of relative wing velocity with
      %   respect to q
      % @retval dwingvel_reldqd derivative of relative wing velocity with
      %   respect to q-dot
      
      if (nargout>1)
        [wingvel_rel,wingvel_relP,wingvel_relJ] = bodyKin(manip,kinsol,kinframe,[wingvel_struct.wingvel_world_xz,zeros(3,1)]);
        wingvel_rel = wingvel_rel(:,1)-wingvel_rel(:,2);
        wingvel_relJ = wingvel_relJ(1:3,:)-wingvel_relJ(4:6,:);
        wingvel_relP = wingvel_relP(1:3,:)-wingvel_relP(4:6,:);
        dwingvel_reldq = wingvel_relJ+wingvel_relP(:,1:3)*wingvel_dstruct.dwingvel_worlddq;
        dwingvel_reldqd = wingvel_relP(:,1:3)*wingvel_dstruct.dwingvel_worlddqd;
      else
        wingvel_rel = bodyKin(manip,kinsol,kinframe,[wingvel_struct.wingvel_world_xz,zeros(3,1)]);
        wingvel_rel = wingvel_rel(:,1)-wingvel_rel(:,2);
      end
      
    end

    function [model, obj] = parseURDFNode(model,robotnum,node,options)
      % Build a RigidBodyWing from a URDF.
      %
      % @param model model we are adding to
      % @param node parent node in the URDF
      % @param options unused at the current time
      %
      % @retval model updated model
      % @retval obj constructed RigidBodyWing
      
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
        end
      end
      
      
      wing_profile = char(node.getAttribute('profile'));
      wing_chord = parseParamString(model,robotnum,char(node.getAttribute('chord')));
      wing_span = parseParamString(model,robotnum,char(node.getAttribute('span')));
      wing_stall_angle = parseParamString(model,robotnum,char(node.getAttribute('stall_angle')));
      nominal_speed = parseParamString(model,robotnum,char(node.getAttribute('nominal_speed')));
      visual_geometry_urdf = parseParamString(model,robotnum,char(node.getAttribute('visual_geometry')));
      
      if isempty(visual_geometry_urdf)
        % visual geometry defaults to on
        visual_geometry_urdf = 1;
      end
      

      [model,this_frame_id] = addFrame(model,RigidBodyFrame(parent,xyz,rpy,[name,'_frame']));

      obj = RigidBodyWing(this_frame_id, wing_profile, wing_chord, wing_span, wing_stall_angle, nominal_speed);
      obj.visual_geometry = visual_geometry_urdf;
      obj.name = name;
      obj.parent_id = parent;

      % bind parameters before drawing so we know what to draw
      obj = obj.bindParams(model, double(model.getParams()));
      bound_frame = model.getFrame(this_frame_id).bindParams(model, double(model.getParams()));
      model = model.setFrame(this_frame_id, bound_frame);
      
      if (visual_geometry_urdf)
        % add visual geometry for automatic drawing of the wing
        model = obj.addWingVisualGeometryToBody(model, parent);
      end
      
    end

  end

end
