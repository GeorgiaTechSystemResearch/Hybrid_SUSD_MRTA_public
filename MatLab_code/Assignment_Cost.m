function [local_task_ordering, cost] = Assignment_Cost(x0, P, s_i)

% x0: the starting location (2 by M) where M is the number of robot
% when M > 1, we only consider the first robot
% P: the goal positions (2 by N) where N is the number of goal locations
% s_i: score vector of robot i for all tasks, u_ij: the score if robot i 
% perform task j

[local_task_ordering, cost_executing] = greedy_task_ordering(x0, P);


% utility function: approach 3
lamda = 0.1;
P1 = P(:,local_task_ordering);
P0 = [x0, P1(:,1:end-1)];
dist_vec = sqrt(sum((P1-P0).^2, 1));
s1 = s_i(local_task_ordering);
cost = -1*sum(lamda.^dist_vec.*s1);

% if cost > 0
%     cost_executing
%     sum(s_i)
%     cost
%     error('postive assignment cost (negative utility)')
% end


