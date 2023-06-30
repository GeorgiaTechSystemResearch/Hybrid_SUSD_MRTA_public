function [cost, path] = market_onlineTA(P, x0, S, ini_assignment)

    %% basic setup
    
    % get total number of parameters for the graph
    [~, N_goals] = size(P);
    [~, N_agents] = size(x0);
   
    %% market-based approach
    if ~exist('ini_assignment','var')
        task_assignments{N_agents} = [];
    else
        task_assignments = ini_assignment;
    end

    % getting the results
    total_cost = 0;
    clear task_ordering;
    task_ordering{N_agents} = [];
    for j=1:N_agents
        if size (task_assignments{j}, 1) > 0
            goal_indices = task_assignments{j};
            %[local_task_ordering, curr_cost_greedy] = greedy_task_ordering(x0(:,j),P(:, goal_indices));
            [local_task_ordering, cost] = Assignment_Cost(x0(:,j),P(:, goal_indices), S(j,goal_indices));
            total_cost = total_cost + cost;
            curr_task_ordering = goal_indices(local_task_ordering);
            task_ordering{j} = curr_task_ordering;
        end
    end
    
    % bid on unassigned tasks
    unassigned_task_indices = setdiff(1:N_goals,cell2mat(task_ordering));
    task_assignments = task_ordering;

    for i = unassigned_task_indices
        C = Inf(N_agents,1);
        for j = 1:N_agents
            if size (task_ordering{j}, 1) > 0
                goal_indices = task_ordering{j};
                [~, curr_cost] = Assignment_Cost(x0(:,j),P(:, goal_indices), S(j,goal_indices));
                [~, poss_cost] = Assignment_Cost(x0(:,j),P(:, [goal_indices, i]), S(j,[goal_indices, i]));
                C(j) = poss_cost-curr_cost;
            else
               [~, curr_cost] = Assignment_Cost(x0(:,j),P(:,i), S(j,i));
               C(j) = curr_cost;
            end
        end
        [~, winner_idx] = min(C);
        task_assignments{winner_idx}(end+1) = i;
        task_ordering = task_assignments;
    end

    % sort task assignemnts for each robot to generate paths and calculate
    % the total cost
    total_cost = 0;
    clear task_ordering;
    for j=1:N_agents
        goal_indices = cell2mat(task_assignments(j));
        task_ordering{j} = [];
        if size(goal_indices, 1) > 0
            [local_task_ordering, curr_cost] = Assignment_Cost(x0(:,j),P(:, goal_indices), S(j,goal_indices));
            task_ordering{j} = goal_indices(local_task_ordering);
            total_cost = total_cost + curr_cost;
        end
    end

    cost = total_cost;
    path = task_ordering;
end



