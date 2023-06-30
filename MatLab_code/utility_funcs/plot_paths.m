function fig = plot_paths(P, x0, paths, title_name, task_types, width, height)

    if ~exist('task_types','var')
        task_types = zeros(size(P));
    end
    
    % get total number of parameters for the graph
    [d, N_tasks] = size(P);
    [d, N_agents] = size(x0);

    % setup
    %figure(1); 
    if N_agents == 3
        c = [1 0 0; 0 1 0; 0 0 1]'; % robot colors    
    elseif N_agents == 4
        c = [1 0 0; 0 1 0; 0 0 1; 1 1 0]'; % robot colors
    else
        c = rand(3, N_agents);
    end
    clf

    % plot the goal locations
    if sum(task_types, 'all') ~= 0
        % type 1 tasks
        t1 = (task_types(1,:) == 1);
        h0_g1 = scatter(P(1,t1),P(2,t1),'+'); 
        hold on;
        % type 2 tasks
%         t2 = (task_types(2,:) == 1);
%         h0_g2 = scatter(P(1,t2),P(2,t2),'*'); 
%         hold on;
    else
       h0_g = plot(P(1,:),P(2,:),'ko','markerfacecolor','k'); 
       hold on;
    end

    for j = 1:N_tasks
        text(P(1,j),P(2,j),int2str(j),'FontSize', 12);
    end
    
    % plot the starting locations
    h1_g = scatter(x0(1,1:N_agents),x0(2,1:N_agents),[],c(:,1:N_agents)');
    hold on;
    
    % plot paths
%     legend_set = {'task (type1) loc','task (type2) loc', 'staring loc'};
    legend_set = {'task loc', 'staring loc'};

    for j=1:N_agents
        if  ~isempty(paths{j})
            goal_indices_array_per_robot = paths{j};
            h_s = plot([x0(1,j), P(1,goal_indices_array_per_robot(1))], [x0(2,j), P(2,goal_indices_array_per_robot(1))],'.-', 'color', c(:,j));
            h = plot(P(1,goal_indices_array_per_robot), P(2,goal_indices_array_per_robot),'.-', 'color', c(:,j));
%             legend_set(end+1) = {append('robot',int2str(j))};
            %legend_set(end+1) = {append('robot',int2str(j))};
        end
    end
    legend(legend_set, 'FontSize', 12)
    grid on;
    axis([-2 width+5 -2 height+5]);
    pbaspect([1 1 1]);
    title(title_name,'FontSize', 12);

    drawnow; 
    pause(0.2);

    fig = gcf;

end