% Define Environment
boundary = [0, 0; 10, 0; 10, 10; 0, 10];
obstacle1 = [3, 0; 7, 0; 7, 4; 3, 4];  % Lower bridge
obstacle2 = [3, 6; 7, 6; 7, 10; 3, 10]; % Upper bridge
obstacle3 = [0, 4; 2, 4; 2, 6; 0, 6]; % Left vertical bar
obstacle4 = [8, 4; 10, 4; 10, 6; 8, 6]; % Right vertical bar

% Start and Goal Position
start = [1, 5];
goal = [9, 7];

% Visualization
figure;
hold on;
fill(obstacle1(:,1), obstacle1(:,2), [0.7, 0.7, 0.7]);
fill(obstacle2(:,1), obstacle2(:,2), [0.7, 0.7, 0.7]);
fill(obstacle3(:,1), obstacle3(:,2), [0.7, 0.7, 0.7]);
fill(obstacle4(:,1), obstacle4(:,2), [0.7, 0.7, 0.7]);
plot(boundary([1:end 1],1), boundary([1:end 1],2), 'k-');
plot(start(1), start(2), 'ro', 'LineWidth', 2);
plot(goal(1), goal(2), 'go', 'LineWidth', 2);
axis equal;
grid on;

% Path Planning and Navigation Logic (similar as above)
% ... Rest of the code remains similar ...
stepSize = 0.1;
robotPos = start;
hitPoint = [];
followBoundary = false;

while norm(robotPos - goal) > 0.1
    if ~followBoundary
        direction = (goal - robotPos) / norm(goal - robotPos);
        nextPos = robotPos + stepSize * direction;
        
        if inpolygon(nextPos(1), nextPos(2), obstacle1(:,1), obstacle1(:,2)) || ...
           inpolygon(nextPos(1), nextPos(2), obstacle2(:,1), obstacle2(:,2))
            followBoundary = true;
            hitPoint = robotPos;
            direction = [-direction(2), direction(1)]; % Rotate 90 degrees
            disp('Obstacle hit! Following the boundary.');
        end
    else
        direction = (robotPos - hitPoint) / norm(robotPos - hitPoint);
        direction = [-direction(2), direction(1)]; % Rotate 90 degrees
        nextPos = robotPos + stepSize * direction;
        
        % Check if the robot can move directly to the goal
        lineToGoal = [robotPos; goal];
        [xi, yi] = polyxpoly(lineToGoal(:,1), lineToGoal(:,2), obstacle1(:,1), obstacle1(:,2));
        [xi2, yi2] = polyxpoly(lineToGoal(:,1), lineToGoal(:,2), obstacle2(:,1), obstacle2(:,2));
        
        if isempty(xi) && isempty(xi2) && norm(nextPos - hitPoint) > stepSize
            followBoundary = false;
            disp('Clear path to goal detected, resuming direct navigation.');
        end
    end
    
    robotPos = robotPos + stepSize * direction;
    plot(robotPos(1), robotPos(2), 'b.');
    pause(0.05);
end
disp('Goal reached!');
title('Robot Navigation Path with Bug Algorithm');
