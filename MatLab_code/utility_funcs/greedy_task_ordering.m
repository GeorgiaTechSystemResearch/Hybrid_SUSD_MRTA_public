function [goal_assignment, cost_greedy] = greedy_task_ordering(x0, P)
% x0: the starting location (2 by M) where M is the number of robot
% when M > 1, we only consider the first robot
% P: the goal positions (2 by N) where N is the number of goal locations


N_goals =  size(P,2);
% Distance matrix (euclidean distance) between each goal pairs
Dist_goals= pdist2(P',P');
[M, I_goal] = min(Dist_goals');

% Distance between each agent's start postions and each goal
Dist_start_goal= pdist2(x0',P');
[M, I_start] = min(Dist_start_goal');

% avoid staying at the goal location 
for i = 1:N_goals
    Dist_goals(i, i) = inf;
end 

cost_greedy = 0;
goal_assignment = [];

goal_idx = I_start(1);
goal_assignment(end+1) = goal_idx;
cost_greedy = M(1);
Dist_goals(:, goal_idx) = inf;

for i = 2:N_goals
    [cost, next_goal_idx] = min(Dist_goals(goal_idx,:));
    % avoid going back and forth
    goal_idx = next_goal_idx;
    cost_greedy = cost_greedy + cost;
    goal_assignment(end+1) = goal_idx;
    Dist_goals(:, goal_idx) = inf;
end
