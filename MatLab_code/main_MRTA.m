clear all;
close all;


%% Settings 
addpath('utility_funcs')

N_agents = 5;
N_goals_init = 5;
N_new_goals_per_iter = 5;
N_iter = 5;
N_goals = N_goals_init;
map_size = [25;25]; % 

% robot locations
%x0 = [0 0; 3 5; 5 0; 0 5; 5 5]';
x0 = rand(2, N_agents).*map_size;
% robot type (higher value means better at the task)
Q = [2 1; 2 1; 1 2; 1 2; 2 2]'; 

%% Hyperparameters 
N_time_steps = 20;
k1 = 2.0;
k2 = 1.0;
d_des = 0.2;
susd_hyperparams = [k1, k2, d_des];

brs_hyperparams = [4, 20, 0.2];
aco_hyperparams = [0.1, 1, 1, 0.2, 0.2, 50];

ini_bias = 8.0;

folder = "results/" + datestr(now, 1);
mkdir(folder)
save(folder+'/susd_hyperparams.mat','susd_hyperparams')

%% Running

% make deterministic for site generation
random_seed = 1;
rng(random_seed);

% making subfolder
subfolder = folder + "/rand"+num2str(random_seed);
mkdir(subfolder)

% Video
myVideo = VideoWriter(subfolder+"/onlineTA.mp4", 'MPEG-4');
myVideo.FrameRate = 1;
myVideo.Quality = 100;
open(myVideo);

% define goal physical positions (cost is euclidean distance)
P = rand(2,N_goals_init).*map_size;

indices = randi([1 2],1,N_goals_init);
Y = zeros(2, N_goals_init);
for i=1:N_goals_init
    Y(indices(i),i) = 1;
end
%      Y = [0	0	1	0	1
%         1	1	0	1	0];

goal_assignments = cell(N_agents,1);  
goal_assignments_baseline = cell(N_agents,1);  
goal_assignments_prim = cell(N_agents,1); 
goal_assignments_hungarian = cell(N_agents,1); 

for T = 1:N_iter % two new tasks at each iterations
    new_P = rand(2,N_new_goals_per_iter).*map_size;
    P = [P, new_P];
    [d, N_goals] = size(P);

    % Task type for new tasks
    indices = randi([1 2],1,N_new_goals_per_iter);
    new_Y = zeros(2, N_new_goals_per_iter);
    for i=1:N_new_goals_per_iter
        new_Y(indices(i),i) = 1;
    end
    Y = [Y, new_Y];
    % specialization Martix
    S = Q'*Y;

    %% Hybird SUSD
    % market-based (step 1)
    disp('Hybird SUSD')
    
    [cost_m, paths_m] = market_onlineTA(P, x0, S, goal_assignments);
    goal_assignments = paths_m;
    % create initialization of theta for SUSD using market-based results:
    theta_m = ones(N_agents, N_goals)/10;
    robot_idx = 1;
    for goal_assignment_per_robot = goal_assignments
        goal_array = cell2mat(goal_assignment_per_robot);
        %initilization
        theta_m(robot_idx, goal_array) = theta_m(robot_idx, goal_array) + ini_bias;
        robot_idx = robot_idx+1;
    end
    %fig = plot_paths(P, x0, paths_m, "Market C="+num2str(cost_m), Y, map_size(1), map_size(2));
    %frame = getframe(fig);
    %writeVideo(myVideo, frame);

    % SUSD (step 2)
    [cost_susd, C_, paths_susd, theta_susd] = susd_onlineTA(P, x0, susd_hyperparams, N_time_steps, S, theta_m);
    
    if cost_susd > cost_m
        cost_susd = cost_m;
        paths_susd = paths_m;
    end

    % deterministic
    goal_assignments = paths_susd;
    fig = plot_paths(P, x0, paths_susd, "SUSD U="+num2str(-cost_susd), Y, map_size(1), map_size(2));
    frame = getframe(fig);
    writeVideo(myVideo, frame);
    

end

% Store variable
save(subfolder+'/paths_susd.mat','paths_susd');
save(subfolder+'/cost_susd.mat','cost_susd');
save(subfolder+'/P.mat','P');
save(subfolder+'/x0.mat','x0');

close
close(myVideo);

