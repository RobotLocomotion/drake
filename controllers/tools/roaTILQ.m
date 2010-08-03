function rho = roaTILQ(system,x0,u0,K,S,options)

    nx = length(x0);
    nu = length(u0);
    
    if nargin < 6, options = struct(); end
    if (~isfield(options,'plant_order')) options.plant_order = 3; end
    if (~isfield(options,'Umax')) options.Umax = repmat(system.umax,nu,1); end
    if (~isfield(options,'Umin')) options.Umin = repmat(system.umin,nu,1); end

    x = msspoly('x',nx); % Actually xbar (true state minus equlibrium)
    u = msspoly('u',nu);

    df = system.dynamicsGradients(0,x0,u0,options.plant_order);
    
    fhat = system.dynamics(0,x0,u0);

    for i = 1:options.plant_order
        % Contract the tensors.
        fhat = fhat + contract_tensor(df{i},[0;x-x0;u-u0])/factorial(i);
    end

    rho = roaTILQPoly(x,u,fhat,x0,u0,K,S,options);
end

function o = contract_tensor(T,x)
    if isnumeric(T)
        o = T*x;
    else
        s = [];
        for j = 1:length(T)
            s = [s contract_tensor(T{j},x)];
        end
        o = s*x;
    end
end