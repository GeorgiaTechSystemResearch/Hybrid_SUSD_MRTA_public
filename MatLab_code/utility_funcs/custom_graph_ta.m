classdef custom_graph_ta < handle
    % costume graph for virtual agents on task allocation
    properties
        N_agents
        N_goals
        theta
        eps
    end
    
    methods
        function obj = custom_graph_ta(N_agents, N_goals, eps, ini_theta)
            obj.N_agents = N_agents;
            obj.N_goals = N_goals;
            obj.eps = eps;
            if ~exist('ini_theta','var')
                obj.theta = rand(obj.N_agents, obj.N_goals);
            else
                obj.theta = ini_theta + rand(size(ini_theta))/10;
            end
        end
        
        function x = sample(obj)
            % convert each goal node to a distribution
            ex_theta = exp(obj.theta);
            ptheta = ex_theta./sum(ex_theta);
            % with probability eps, randomize the probabilities
            if rand < obj.eps
                ptheta = ones(obj.N_agents, obj.N_goals);
                ptheta = ptheta./sum(ptheta);
            end
            
            % sample each goal node as an allocation
            x{obj.N_agents} = [];
            for i=1:obj.N_goals
                xa = randsample(obj.N_agents,1,true,ptheta(:,i));
                x{xa}(end+1) = i;
%                 ptheta(xa,:) = 0;  % for SR-ST
            end
        end
        
        function [] = set_theta(obj, theta, std)
            % add guassian noise inspired by "ADDING GRADIENT NOISE IMPROVES LEARNING FOR VERY DEEP NETWORKS"
            if ~exist('std','var')
                std = 0;
            end
            obj.theta = reshape(theta,obj.N_agents,obj.N_goals);
            obj.theta = obj.theta + std*randn(size(obj.theta));
        end
        
        function theta = get_theta(obj)
            theta = obj.theta(:);
        end
    end
end

