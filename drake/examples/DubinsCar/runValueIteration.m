function [J, PI] = runValueIteration

% the location of obstacles is encoded in the cost function, with a region
% of increased cost centered around the obstacle.

    plant = DubinsPlant;
    options.dt = 1e-2;
    options.gamma = .999; % we have a discount factor to allow for convergence
    options.wrap_flag = [true;true;true];
    options.vectorized_x = true;
    % we have the controller wrap around for all three state variables.

    % wrapping around in x([1, 2]) = (x, y) allows us to simulate a infinite
    % obstacle field consisting of a grid of cells, with each cell containing
    % an identical obstacle.
    % wrapping around in x(3) = θ allows the robot to complete one full
    % rotation.

    cost = @lqrcost; 
    ulimit = 1;
    L = 10;
    spatial_resolution = 2*L/50; % resolution of discretization of spatial coordinates
    angular_resolution = 2*pi/65; % resolution of discretization of angular position of car


    xbins = {
        make_bins(-L, L, spatial_resolution), ...
        make_bins(-L, L, spatial_resolution), ...
        make_bins(0, 2*pi, angular_resolution)
            };
    ubins = linspace(-ulimit,ulimit,19);
    mdp = MarkovDecisionProcess.discretizeSystem(plant,cost,xbins,ubins,options);

    function A = extract_array(M,xbins,target_theta)
    % extracts the 2-d array corresponding to target_theta from M.
    % @param M array with size 
    % @param xbins cell array with the cell elements being the bins for
    %   the x, y, and theta state variable respectively.
    % @param target_theta target value of theta state variable.
        
        lengths = cellfun(@length, xbins);
        M_reshaped = reshape(M, lengths);
        [~, target_index] = min(abs(xbins{3}-target_theta));
        A = M_reshaped(:,:,target_index)';
    end

    % TODO can I change the plotting so it only plots sometimes? how to
    % do this safely?

    function drawfun_full(J,PI,current_theta,figure_number)
        
        if nargin == 3
            sfigure(1)
        else
            sfigure(figure_number);
        end
        
        clf;

        % value of current heading for which we are plotting the
        % control input and the costs
        control_input = extract_array(ubins(PI), xbins, current_theta);
        costs = extract_array(J, xbins, current_theta);
        
        time_per_cell = spatial_resolution / plant.v;
        % approximate time it takes to traverse one cell
        k = 10;
        new_theta = current_theta+control_input*time_per_cell*k;
        % we approximate the new bearing of the robot, exaggerating the
        % change by a factor of k
        
%         subplot(1,3,1);
%         quiver(xbins{1},xbins{2},cos(new_theta),sin(new_theta),0.5);
%         axis square;
%         % we avoid using the automatically provided x and y limits
%         % since the scale they provide is different from that of imagesc
%         xlim([min(xbins{1})-spatial_resolution/2 max(xbins{1})+spatial_resolution/2]);
%         ylim([min(xbins{2})-spatial_resolution/2 max(xbins{2})+spatial_resolution/2]);
%         xlabel('x');
%         ylabel('y');
%         title('$\theta + \dot{\theta} \cdot k \Delta t, k=10.$','interpreter','latex');
%         colorbar;
        
        % we specify clims so that the colors plotted stay consistent
        % even if not all the us are used in a given policy.
        clims = [min(ubins) max(ubins)];
        subplot(1,2,1);imagesc(xbins{1},xbins{2},control_input,clims);
        axis square;
        xlabel('x');
        ylabel('y');
        title('u(x)');
        set(gca,'YDir','normal')
        colorbar;
        
        % plotting cost array with scaled colors
        subplot(1,2,2);
        imagesc(xbins{1},xbins{2},costs);
        axis square;
        xlabel('x');
        ylabel('y');
        title('J(x)');
        set(gca,'YDir','normal')
        colorbar;
        
        drawnow;
    end

    drawfun = @(current_theta) (@(J, PI) drawfun_full(J, PI, current_theta));    

%     [J,PI] = valueIteration(mdp,0.000001);

%     drawfun_full(J,PI.PI,pi/4,1);
%     drawfun_full(J,PI.PI,3*pi/4,2);
%     drawfun_full(J,PI.PI,5*pi/4,3);
%     drawfun_full(J,PI.PI,7*pi/4,4);

end

function bins = make_bins(x_min,x_max,x_resolution)
    bins = linspace(x_min, x_max, (x_max-x_min)/x_resolution);
end

function c = point_obstacle_cost(x_robot, x_obs)
% calculates the cost an obstacle at x_obs imposes on a robot at
% x_robot.
    npts = size(x_robot,2);
    TheMatrix = x_robot-repmat(x_obs,1,npts);
    %distance = arrayfun(@(idx)norm(TheMatrix(:,idx)), 1:size(TheMatrix,2));
    distance = sqrt(sum(abs(TheMatrix).^2,1));
    % if distance > 2
    %     c = 0;
    % else
    %     % we use a standard gaussian
    %     c = exp(-distance^2/10);
    % end
    c = exp(-distance.^2);
end

% TODO: write obstacle class
% TODO: fully specify simulations
function g = lqrcost(sys,x,u)
    k = 2;
    obstacles = [[0;0]];
    % obstacles = [[k;k],[k;0],[k;-k],[0;k],[0;-k],[-k;k],[-k;0],[-k;-k]];
    % obstacles = [[-7;-7],[7;7],[2;-2]];
    % obstacles = [[0;0],[-3;0],[-6;0]];
    x_robot = x(1:2,:);

    g = 0.01*u^2;
    for n = 1:size(obstacles,2)
        g = g + point_obstacle_cost(x_robot, obstacles(:,n));
    end
    
end