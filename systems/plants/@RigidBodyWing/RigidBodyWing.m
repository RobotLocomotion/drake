classdef RigidBodyWing < RigidBodyForceElement

  properties
    bodyind    % RigidBody index
    origin;  %xyz of the quarter-chord of the wing. 2x1 for planar, 3x1 for 3D
    fCl  % splines representing the *dimensional* coefficients
    fCd  %  with fCl = 1/2 rho*S*Cl, etc. (S=area)
    fCm
    area
    %Air density for 20 degC dry air, at sea level
    rho = 1.204;
  end
  
  methods
    function obj = RigidBodyWing(profile, aero_origin, chord, span, stallAngle, velocity, parent_body, threeD)
%{
      %TODO:
      % Implement some values for post-stall CMs
      %     i.e. at 90 degrees aoa, the center of pressure should be
      %     at the half chord.
      % use variables for path to xfoil and AVL
      %
      % Add support for Mac OS?  unless system calls are the same as
      % linux
      %
      %calls AVL and XFOIL over different angles of attack at the
      %given velocity, generates first order polynomials of the CL,
      %CD, and pitch moments of the wing.  The axes for use in DRAKE are:
      %   X = forward, in the usual direction of travel
      %   Y = out the left side of the wing
      %   Z = up
      % Note: an sign inversion is needed for the moment coefficient, as a
      % positive pitching moment from AVL/XFOIL represents a pitch up, but
      % a positive y-axis rotation represents a pitch down.
      % @param profile = Profile can either be a:
      %         -NACA airfoil designation starting with 'NACA'
      %         -File location of a .dat file generated by xfoil
      %         -File location of a .mat file which contains the
      %         variables 'CLSpline', 'CDSpline', 'CMSpline'
      %         -the words 'flat plate'
      % @param aero_origin = [x y z] Quarter-chord of the
      %         airfoil with respect to the body it is mounted on.
      %         NOTE: the wing axes are bound to the axis of the
      %         parent body in the convention above.  I.e. the x-axis
      %         of the parent body should be forward, in the typical
      %         direction of travel.  The Y-axis is tangent to the
      %         leading edge. This is done to avoid any complexity or confusion on
      %         angle conventions. If you want to rotate the wing,
      %         just define the rotation of its parent body in the
      %         URDF. (Note, this is unlike the planar case)
      %         The XYZ point should be the
      %         quarter-chord point for flat plates, symmetric
      %         airfoils, and when you're not exactly sure of what it
      %         should be.
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
 %}
      if (nargin<8)
          threeD = true;
      end
      %Check that parent_body is a RigidBody object.
      if ~isa(parent_body, 'numeric');
        error('Drake:RigidBodyWing:InvalidParent','Force Type Wing does not have a proper RigidBody parent');
      end
      if threeD
        obj.origin = reshape(aero_origin, 3, 1);
      else
        obj.origin = reshape(aero_origin, 2, 1);
      end
      obj.bodyind = parent_body;
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
          avlfile = regexprep(avlfile, '\$Xorig', sprintf('%.2f',obj.origin(1)));
          if threeD
            avlfile = regexprep(avlfile, '\$Yorig', sprintf('%.2f',obj.origin(2)));
          else
            avlfile = regexprep(avlfile, '\$Yorig', sprintf('%.2f',0));
          end
          avlfile = regexprep(avlfile, '\$Zorig', sprintf('%.2f',obj.origin(end)));
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
              result = system(commandstring);
              
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
              result = system(commandstring);
              
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
        angles = [-180:2:-(stallAngle+.0001) -stallAngle:stallAngle/12:(stallAngle-.0001) stallAngle:2:180];
        obj.fCl = spline(angles, .5*(2*sind(angles).*cosd(angles))*obj.rho*obj.area);
        obj.fCd = spline(angles, .5*(2*sind(angles).^2)*obj.rho*obj.area);
        obj.fCm = spline(angles, -(angles./90)*obj.rho*obj.area*chord/4);
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
    end %constructor

    function force = computeSpatialForce(obj,manip,q,qd)
      kinsol = doKinematics(manip,q, false, false);
      %origin = [x z theta] of the reference point
      [x,J] = forwardKin(manip,kinsol,obj.bodyind,obj.origin, 1);
      %Check Gradients
      %[xnum, Jnum] = geval(@forwardKin, manip,kinsol,obj.body,obj.origin(1:2), true);
      v = J*qd;
      %velocity of the wing (flight path) in WORLD coordinates
      wingvel = v(1:3);  % assume still air. Air flow over the wing
      %project this onto the XZ plane of the wing (ignores sideslip)
      kinsolT = kinsol.T{obj.bodyind};
      wingXunit = kinsolT(1:3,1:3)*[1 0 0]';
      wingYunit = kinsolT(1:3,1:3)*[0 1 0]';
      sideslip = dot(wingvel,wingYunit);
      wingXZvelocity = wingvel-(sideslip*wingYunit);
      airspeed = norm(wingXZvelocity);
      %need the angle of attack from the wing's XZ velocity.  This is done
      %by computing the angle between the velocity and the wing's X unit
      %(forward facing) vector, then using a the cross product of these two
      %to determine which direction this angle should be in. (Everything
      %should be in world coordinates)
      angVelandZ = acosd(dot(wingXunit,wingXZvelocity)/(norm(wingXunit)*norm(wingXZvelocity)));
      aoa = angVelandZ * sign(dot(cross(wingXunit, wingXZvelocity),wingYunit));
      % mod 360 so -180<AoA<180 Shouldn't be necessary because of acosd
      if aoa>180
        aoa = aoa-360*fix((aoa+180)/360);
      elseif aoa<-180
        aoa = aoa-360*fix((aoa-180)/360);
      end
      force = sparse(6,getNumBodies(manip));
      %lift and drag are the forces on the body in the world frame.
      %cross(wingXZvelocity, wingYunit) rotates it by 90 degrees
      lift_world = ppvalSafe(obj.fCl,aoa, false, false)*airspeed*cross(wingXZvelocity, wingYunit);
      drag_world = ppvalSafe(obj.fCd,aoa, false, false)*airspeed*-wingXZvelocity;
      torque = -ppvalSafe(obj.fCm,aoa, false, false)*airspeed*airspeed*wingYunit;
      %inputs of point (body coordinates), and force (world coordinates)
      %returns [torque; xforce; yforce] in the body coordinates
      %obj.body.dofnum should have 6 elements for
      %linkID = manip.findLinkInd(obj.body.linkname, 0);
      force(:,obj.bodyind) = [torque;0;0;0] + ...
        cartesianForceToSpatialForce(manip, kinsol, obj.bodyind, obj.origin,lift_world+drag_world);
      force;
    end
    function [CL CD CM] = coeffs(AoA)
      %returns dimensionalized coefficient of lift, drag, and pitch moment for a
      %given angle of attack
      CL = ppval(obj.fCl, AoA);
      CD = ppval(obj.fCd, AoA);
      CM = ppval(obj.fCm, AoA);
    end
    function obj = updateBodyIndices(obj,map_from_old_to_new)
      obj.bodyind = map_from_old_to_new(obj.bodyind);
    end
    
  end
  
end
