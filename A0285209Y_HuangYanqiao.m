% load map
data = load('Map.mat');
map = data.Map;

% set start and goal point
start = [1, 2];
goal = [10, 10];

%get path and visualize it
path = astar(map, start, goal);

if isempty(path) == 1
    disp('No Available Path');
else
    visualize_map_and_path(map, path);
end


function path = astar(map, start, goal)
    % Define the possible moves from a grid cell
    moves = [0, 1; 0, -1; 1, 0; -1, 0; 1, 1; -1, 1; 1, -1; -1, -1];
    
    % Initialize sets and scores
    openSet = start;
    closedSet = [];
    gScore = inf(size(map));
    gScore(start(1), start(2)) = 0;
    fScore = inf(size(map));
    fScore(start(1), start(2)) = heuristic(start, goal);
    
    % Store the path information
    previous = zeros(size(map, 1), size(map, 2), 2);
    
    while ~isempty(openSet)
        % Find the node in openSet with the lowest fScore value
        minF = inf;
        current = [];
        for i = 1:size(openSet, 1)
            node = openSet(i, :);
            if fScore(node(1), node(2)) < minF
                minF = fScore(node(1), node(2));
                current = node;
            end
        end
        currentRow = current(1);
        currentCol = current(2);
       
        
        % If the goal is reached, return the path
        if isequal(current, goal)
            path = getPath(previous, current);
            return;
        end
        
        % Update sets and scores
        openSet(openSet(:, 1) == currentRow & openSet(:, 2) == currentCol, :) = []; %Open.pop
        closedSet = [closedSet; current]; %Closed.push
        
        % Check neighbors
        for i = 1:size(moves, 1)
            neighbor = current + moves(i, :);

            % Skip if neighbor is out of bounds
            if neighbor(1) < 1 || neighbor(1) > size(map, 1) || neighbor(2) < 1 || neighbor(2) > size(map, 2)
                continue; 
            end

            % Skip if neighbor is an obstacle or already evaluated
            if map(neighbor(1), neighbor(2)) == 1 || ismember(neighbor, closedSet, 'rows')
                continue; 
            end

            
            if i > 4
                %The diagonally travelling cost is 1.4 （close to root 2）
                newScore = gScore(currentRow, currentCol) + 1.4;
            else
                newScore = gScore(currentRow, currentCol) + 1;
            end
            
            
            if newScore < gScore(neighbor(1), neighbor(2))
                previous(neighbor(1), neighbor(2), :) = current;
                gScore(neighbor(1), neighbor(2)) = newScore;
                fScore(neighbor(1), neighbor(2)) = newScore + heuristic(neighbor, goal);
                if ~ismember(neighbor, openSet, 'rows')
                    openSet = [openSet; neighbor];
                end
            end
        end
    end
    
    % Return an empty path if no path is found
    path = [];
end

function dis = heuristic(node, goal)
    % Euclidean distance
    dis = sqrt(sum((node - goal).^2));
end

function path = getPath(privious, current)
    % get the path from the goal to start point
    path = current;
    while any(privious(current(1), current(2), :))
        current = squeeze(privious(current(1), current(2), :))';
        path = [current; path];
    end
end

function visualize_map_and_path(map, path)
    figure; 
    imagesc(1 - map); % Mapping to show obstacles (1) as black and free space (0) as white
    colormap(gray);
    hold on; 

    % draw the path
    plot(path(:,2), path(:,1), 'r-', 'LineWidth', 2);
    plot(path(1,2), path(1,1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % start point color is green
    plot(path(end,2), path(end,1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % goal point color is red

    axis equal;
    grid on;
    xlabel('X');
    ylabel('Y');
    xlim([1 10]);
    ylim([1 10]);
    title('Map and Path');
end





