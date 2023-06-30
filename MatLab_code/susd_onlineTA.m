function [cost, C_, path, best_theta] = susd_onlineTA(P, x0, hyperparams, time_steps, S, ini_theta)

    %% basic setup
    
    % make random again
    rng(datenum(datetime));
    
    % get total number of parameters for the graph
    [~, N_goals] = size(P);
    [~, N_agents] = size(x0);
    N_param = N_agents*N_goals;
    
    %% SUSD approach 
    % construct N_param graph objects using custom_graph_ta for task allocation

    k1 = hyperparams(1); %  1.2
    k2 = hyperparams(2); %  0.01;
    d0 = hyperparams(3); %  0.1;
    eps = 0.2;
    N_vagents = 2*N_param;

    if ~exist('ini_theta','var')
        G_ = custom_graph_ta(N_agents, N_goals, 0);
        G = custom_graph_ta(N_agents, N_goals, eps);
        for i=2:N_vagents
            G(i) = custom_graph_ta(N_agents, N_goals, eps);
        end
    else
        G_ = custom_graph_ta(N_agents, N_goals, 0, ini_theta);
        G = custom_graph_ta(N_agents, N_goals, eps, ini_theta);
        for i=2:N_vagents
            G(i) = custom_graph_ta(N_agents, N_goals, eps, ini_theta);
        end
    end

    Cbest = Inf;
    xbest = [];
    
    % perform optimization loop
    T = time_steps;
    n_ = zeros(N_param,1); n_(1) = 1;
    C_ = zeros(1,T);
    
    for t=1:T
        % get the custom_graph_ta samples and parameters
        xi = {N_vagents}; %zeros(N_goals, N_vagents);
        theta_ = zeros(N_param, N_vagents);
        for i=1:N_vagents
            xi{i} = G(i).sample();
            theta_(:,i) = G(i).get_theta();
             % get cost samples for each custom_graph_ta sample
             Cx(1,i) = 0;
             for j=1:N_agents
                goal_indices = cell2mat(xi{i}(j));
                if size(goal_indices, 1) > 0
                    [~, curr_cost] = Assignment_Cost(x0(:,j), P(:, goal_indices), S(j,goal_indices));
                    Cx(1,i) = Cx(1,i)+ curr_cost;
                end
             end
        end
    
        % compute the SUSD direction over the parameter space
        cov_x = cov(theta_');
        [n,~] = eigs(cov_x,1,'SM');
        n_ = n*sign(n'*n_);
        [zmin, zarg] = min(Cx);

        % formation controller input

        % TODO: theta_ or theta_(:,i)
        d = theta_-mean(theta_,2);
        dnorm = vecnorm(d);
        umin = (d).*(d0-dnorm)./(dnorm.^2);
        umin(:,zarg) = 0; % why set formation vector of the v-agent with
        %smallest measurement/cost to 0 ?? doesn't want the smallest
        %v-agent to move
        
        % apply exp mapping
        z = 1-exp(-(Cx-zmin)); % for avoiding vanishing or exploding gradients
        %z = Cx; % old approach 

        theta_ = theta_ + k1.*n_*z + k2.*umin;

        % update the parameters in the graph
        for i=1:N_vagents
            G(i).set_theta(theta_(:,i), 1); % with gaussian nosie (std=1)
        end
        
        % get the minimum parameters and show a sampled graph
        G_.set_theta(theta_(:,zarg));
        allo = G_.sample();    
        C0 = 0;
        task_ordering = [];
        for j=1:N_agents
            goal_indices = cell2mat(allo(j));
            task_ordering{j} = [];
            if size(goal_indices, 1) > 0
                [local_task_ordering, curr_cost] = Assignment_Cost(x0(:,j),P(:, goal_indices), S(j,goal_indices));
                task_ordering{j} = goal_indices(local_task_ordering);
                C0 = C0 + curr_cost;
            end
        end
        C_(t) = C0;
        if C0 < Cbest
            %xbest = allo;
            xbest = task_ordering;
            Cbest = C0;
            best_theta = reshape(theta_(:,zarg),size(ini_theta));
        end
       
        cost = Cbest;
        path = xbest;
    end
end




