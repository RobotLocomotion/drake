function [] = addpath_drake()
% Adds relevant drake directories to the matlab path and performs a few
% system checks.

root = fileparts(mfilename('fullpath')); % drake/matlab dir

% add package directories to the matlab path
addpath(fullfile(root,'util'));

% OSX platform-specific
if (strcmp(computer('arch'),'maci64'))
  % Check if on Yosemite or after
  [OSXvers,~] = evalc('system(''sw_vers -productVersion'')');
  if ~isempty(regexp(OSXvers, '10\.1.', 'match'));
    % Check if reverted to IPv4
    ipv4_preferred = java.lang.System.getProperty('java.net.preferIPv4Stack');
    if isempty(ipv4_preferred)
      ipv4_preferred = 'false';
    end
    if ~(strcmp(ipv4_preferred,'true'))
      display('WARNING: Your JVM may crash if you do not set it to prefer IPv4 over IPv6.')
      display('This may cause any dependencies that involve the JVM (including LCM) to crash at runtime.')
      display('Please see bug report and solution here: https://github.com/RobotLocomotion/drake/issues/558.')
      display('(It just involves adding one line to your java.opts file for Matlab.)')
      display('Make sure to restart Matlab after editing your java.opts file.')
    end
  end
end

