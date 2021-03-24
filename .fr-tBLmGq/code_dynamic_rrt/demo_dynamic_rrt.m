%% �ð��� ���� ���ϴ� ��ֹ��� ���� �� RRT�� path planning�� �غ���.
clc;
clear all;
close all;

fprintf('Path planning using RRT in dynamic environment \n');

%% ���� ������ �Ѵ�. 
plot_stage_and_obs     = 0;   % ���� ���� ȯ���� �׷�����. 
plot_static_rrt_final  = 1;   % RRT ���� ����� �׸���. 
plot_dynamic_rrt_final = 1;   % RRT ����� dynamic�ϰ� �׷�����. 
save_video             = 1;   % ���������� �����Ѵ�. 

%% 1. ���� ������ �����Ѵ�.

% 1. Make stage
stage_struct = rrt_init_stage(10, 10);

% 2. Make static obstacles
obs_struct = rrt_init_obs();
obs_struct = rrt_add_obs(obs_struct, [0 5], [0 0 ; 3 0 ; 5 2 ; 0 2], [0 0]);
obs_struct = rrt_add_obs(obs_struct, [5 3], [0 0 ; 5 0 ; 5 2 ; 2 2], [0 0]);

% 3. Make dynamic obstacles
% obs_struct = rrt_add_obs(obs_struct, [1 1], [1 0 ; 0 1 ; -1 0 ; 0 -1], [.1 .1]);
% obs_struct = rrt_add_obs(obs_struct, [9 9], [1 0 ; 0 1 ; -1 0 ; 0 -1], -[.1 .1]);
obs_struct = rrt_add_obs(obs_struct, [1 1], [2 0 ; 0 2 ; -2 0 ; 0 -2], [.1 .1]);
obs_struct = rrt_add_obs(obs_struct, [9 9], [2 0 ; 0 2 ; -2 0 ; 0 -2], -[.1 .1]);

% 4. �������� �������� �����Ѵ�.
pos_start = [9 1];
pos_goal  = [1 9];

% 4. Ȯ���� ����.
if plot_stage_and_obs
    figure('Position', [900 200 900 700]);
    for t = 1:80
        % 1. Draw stage boundary
        clf; hold on;
        plot(stage_struct.boundary(:, 1), stage_struct.boundary(:, 2), 'r', 'LineWidth', 4);
        % 2. Draw obstacles
        for i = 1:obs_struct.nr_obs
            obs_boundary = rrt_get_obs_boundary(obs_struct, i, t);
            h_bos = fill(obs_boundary(:, 1), obs_boundary(:, 2)...
                , obs_struct.color{i} ...
                ,'LineWidth', 4);
            text(mean(obs_boundary(1:end-1, 1)), mean(obs_boundary(1:end-1, 2)) ...
                , sprintf('[%d-th]', i) ...
                , 'HorizontalAlignment','center' ...
                , 'FontSize', 15 );
        end
        % 3. Draw start and goal points
        h_start = plot(pos_start(1), pos_start(2), 'o' ...
            , 'LineWidth', 3 ...
            , 'MarkerEdgeColor','k' ...
            , 'MarkerFaceColor','g' ...
            , 'MarkerSize', 13);
        text(pos_start(1), pos_start(2), '   Start point');
        h_goal = plot(pos_goal(1), pos_goal(2), 'o'...
            , 'LineWidth', 3 ...
            , 'MarkerEdgeColor','k' ...
            , 'MarkerFaceColor','r' ...
            , 'MarkerSize', 13);
        text(pos_goal(1), pos_goal(2), '   Goal point');
        % 4. legend, title and others..
        legend([h_bos h_start h_goal], 'obastacle', 'start', 'goal', -1);
        hold off; grid on;
        title(sprintf('Operating Stage, time: %d', t), 'FontSize', 15);
        drawnow; pause(.05);
    end
end

%% 2. RRT�� �Ѵ�.

% 1. RRT�� �ʱ�ȭ�Ѵ�.
rrt_struct = rrt_init_rrt(pos_start, pos_goal);

% 2. �̰� �����̴� ��ֹ��� ó���ϱ� ���� ��ƾ�̴�. (�ּ� �ð� ���� ����)
min_time_double = 0;

% 3. RRT�� �����Ѵ�.
tic;
while 1
    % ���� ������ print�� �Ѵ�. 
    if rem(rrt_struct.iter, 100) == 0
        fprintf('[%d] min_time_th: %d max_time_rrt: %d nr_node: %d \n' ...
            , rrt_struct.iter, min_time_round, rrt_struct.max_time ...
            , rrt_struct.nr_node );
    end
    
    % 1.Stage ������ ������ ���� �̰�, �ּ� �ð� ���� ���Ƿ� �̴´�. .
    if rem(rrt_struct.iter, rrt_struct.goal_freq) == 0
        rand_pos = rrt_struct.pos_goal;
    else
        rand_pos = [stage_struct.w*rand stage_struct.h*rand];
    end
    min_time_round = round( rrt_struct.max_time*rand );
    
    % 2. ���� Tree �߿��� ������ ���� ���� ����� node�� ã�´�.
    % idx_closest = rrt_search_shortest_node(rrt_struct, rand_pos);
    idx_closest = rrt_search_shortest_node(rrt_struct, rand_pos, min_time_round);
    
    % 3. ������ ã�� node���� tree�� Ȯ���ϰ�, �� �� rand_pos�� ���� ����� ���� ����.
    curr_pos  = rrt_struct.tree.pos{idx_closest};
    curr_time = rrt_struct.tree.time{idx_closest};
    shortest_next_pos  = [0 0];
    shortest_next_dist = inf;
    shortest_found     = false;
    shortest_u         = [0 0];
    for i = 1:rrt_struct.cnt_expansion
        rand_rad = 2*pi*rand();
        rand_vel = max(rrt_struct.vmax*rand, rrt_struct.vmin);
        rand_ux = rand_vel*cos(rand_rad);
        rand_uy = rand_vel*sin(rand_rad);
        rand_u  = [rand_ux rand_uy];
        
        % tree�� Ȯ���� node�� �� �� �ִ��� Ȯ���Ѵ�.
        next_pos = curr_pos + rand_u;
        if rrt_path_available(curr_pos, next_pos ...
                , stage_struct, obs_struct, curr_time)
            % ���� path�� �� �� �ִٸ�
            curr_next_dist = norm(rand_pos-next_pos);
            if curr_next_dist < shortest_next_dist
                shortest_next_dist = curr_next_dist;
                shortest_next_pos  = next_pos;
                shortest_u         = rand_u;
                shortest_found     = true;
            end
        end
    end 
    
    % 4. shortest_next_pos�� tree�� �߰��Ѵ�.
    if shortest_found
        % shortest node�� ã�Ҵٸ�, tree�� �߰��Ѵ�.
        rrt_struct = rrt_expand_node(rrt_struct, idx_closest ...
            , shortest_next_pos, shortest_u);
    end
    
    % 5. goal point�� �����ߴ��� Ȯ���Ѵ�.
    dist2goal = norm(rrt_struct.pos_goal-shortest_next_pos);
    if dist2goal < rrt_struct.threshold
        fprintf('Close enough to the Goal dist: %.1f \n', dist2goal);
        break;
    end
    
    % ���� ����
    rrt_struct.iter = rrt_struct.iter + 1;
    if rrt_struct.iter > rrt_struct.maxIter
        break;
    end
end
toc;
fprintf('RRT path found \n');

%% 3. RRT �� �ڿ������� ���󰣴�.
% 1. ó�� ��ġ�� �����Ѵ�.
nr_node = rrt_struct.nr_node;
final_time  = rrt_struct.tree.time{nr_node};
parent_idx = rrt_struct.tree.parent{nr_node};
% 2. ��θ� ������ ������ �����Ѵ�.
rrt_path        = zeros(final_time+1, 2);
rrt_path(1, :)  = pos_goal;
path_idx = 1;
% 3. �ڿ��� ���� ���󰣴�.
while 1
    path_idx   = path_idx + 1;
    curr_pos   = rrt_struct.tree.pos{parent_idx};
    parent_idx = rrt_struct.tree.parent{parent_idx};
    rrt_path(path_idx, :)  = curr_pos;
    
    if isequal(rrt_struct.tree.status{parent_idx}, 'root')
        break;
    end
end
rrt_path(end, :) = pos_start;
% 4. path�� �տ��� �ڷ� ������ �ٲ۴�.
rrt_path = flipud(rrt_path);

%% 4. RRT ���� ����� �׷�����.
if plot_static_rrt_final
    figure();hold on;
    t = 0;
    plot(stage_struct.boundary(:, 1), stage_struct.boundary(:, 2), 'r', 'LineWidth', 4);
    % 2. Draw obstacles
    for i = 1:obs_struct.nr_obs
        obs_boundary = rrt_get_obs_boundary(obs_struct, i, t);
        h_bos = fill(obs_boundary(:, 1), obs_boundary(:, 2)...
            , obs_struct.color{i} ...
            ,'LineWidth', 4);
        text(mean(obs_boundary(1:end-1, 1)), mean(obs_boundary(1:end-1, 2)) ...
            , sprintf('[%d-th]', i) ...
            , 'HorizontalAlignment','center' ...
            , 'FontSize', 15 );
    end
    % 3. Draw start and goal points
    h_start = plot(pos_start(1), pos_start(2), 'o' ...
        , 'LineWidth', 3 ...
        , 'MarkerEdgeColor','k' ...
        , 'MarkerFaceColor','g' ...
        , 'MarkerSize', 13);
    text(pos_start(1), pos_start(2), '   Start point');
    h_goal = plot(pos_goal(1), pos_goal(2), 'o'...
        , 'LineWidth', 3 ...
        , 'MarkerEdgeColor','k' ...
        , 'MarkerFaceColor','r' ...
        , 'MarkerSize', 13);
    text(pos_goal(1), pos_goal(2), '   Goal point');
    % 4. legend, title and others..
    legend([h_bos h_start h_goal], 'obastacle', 'start', 'goad', -1);
    
    % 5. �� tree�� ���ؼ� �׸���.
    for i = 1:rrt_struct.nr_node
        if isequal(rrt_struct.tree.status{i}, 'node')
            pos = rrt_struct.tree.pos{i};
            plot(pos(1), pos(2), 'bo');
            
            % ���� node�� parent node�� ���Ѵ�.
            parent_idx = rrt_struct.tree.parent{i};
            parent_pos = rrt_struct.tree.pos{parent_idx};
            parent_line = [pos ; parent_pos];
            plot(parent_line(:,1), parent_line(:,2), 'b-');
            
            % ���� node�� �ð��� �׸���.
            cur_time = rrt_struct.tree.time{i};
            text(pos(1), pos(2), sprintf('  %d', cur_time));
        end
    end
    plot(rrt_path(:,1), rrt_path(:,2), ...
        '--rs','LineWidth',2, ...
        'MarkerEdgeColor', 'k', ...
        'MarkerFaceColor', 'g', ...
        'MarkerSize', 10)
    hold off;
end

%% 5. �ð��� ���� �׷�����. (���������ε� �����Ѵ�.)
if plot_dynamic_rrt_final 
    fig_dyn_rrt = figure('Position', [900 200 900 700]);
    for t = 1:rrt_struct.tree.time{rrt_struct.nr_node}
        % 1. Draw stage boundary
        clf; hold on;
        plot(stage_struct.boundary(:, 1), stage_struct.boundary(:, 2), 'r', 'LineWidth', 4);
        % 2. Draw obstacles
        for i = 1:obs_struct.nr_obs
            obs_boundary = rrt_get_obs_boundary(obs_struct, i, t);
            h_bos = fill(obs_boundary(:, 1), obs_boundary(:, 2)...
                , obs_struct.color{i} ...
                ,'LineWidth', 4);
            text(mean(obs_boundary(1:end-1, 1)), mean(obs_boundary(1:end-1, 2)) ...
                , sprintf('[%d-th]', i) ...
                , 'HorizontalAlignment','center' ...
                , 'FontSize', 15 );
        end
        % 3. Draw start and goal points
        h_start = plot(pos_start(1), pos_start(2), 'o' ...
            , 'LineWidth', 3 ...
            , 'MarkerEdgeColor','k' ...
            , 'MarkerFaceColor','g' ...
            , 'MarkerSize', 13);
        text(pos_start(1), pos_start(2), '   Start point');
        h_goal = plot(pos_goal(1), pos_goal(2), 'o'...
            , 'LineWidth', 3 ...
            , 'MarkerEdgeColor','k' ...
            , 'MarkerFaceColor','r' ...
            , 'MarkerSize', 13);
        text(pos_goal(1), pos_goal(2), '   Goal point');
        % 4. Current robot's position
        h_robot = plot(rrt_path(t,1), rrt_path(t,2) ...
            , 's', 'LineWidth', 3 ...
            , 'MarkerEdgeColor','k' ...
            , 'MarkerFaceColor','g' ...
            , 'MarkerSize', 16);
        plot(rrt_path(1:t,1), rrt_path(1:t,2) ...
            , '-', 'LineWidth', 2 ...
            , 'Color', 'g');
        % 5. legend, title and others..
        legend([h_bos h_start h_goal h_robot] ...
            , 'obastacle', 'start', 'goad',  'robot', -1);
        hold off; grid on;
        title(sprintf('Operating Stage, time: %d', t), 'FontSize', 15);
        drawnow; pause(.05);
        
        % video ������ ���� ������ �����Ѵ�. \
        if save_video
            set(fig_dyn_rrt,'PaperPositionMode','auto')
            print (fig_dyn_rrt , '-dpng', ['pics4video/fig', num2str(t), '.png']) ;
        end
    end
end
 
if save_video 
    fprintf('Saving video ...');
    nr_frame = rrt_struct.tree.time{rrt_struct.nr_node};
    vidName = sprintf('vid/dynamic_rrt_%s.avi', datestr(datenum(clock),'yyyy-mm-dd-HH-MM-SS') );
    frmRate = 10;
    video = VideoWriter( vidName );
    video.FrameRate = ( frmRate );
    open( video );
    for i = 1:nr_frame
        img = imread( ['pics4video/fig' num2str(i) '.png']);
        img = im2double(img);
        writeVideo( video, img );
    end
    close( video );
    fprintf('... save to: %s \n', vidName);
end

%%

