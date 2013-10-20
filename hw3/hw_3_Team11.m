%HW2 - Team 11%

%Henrique Maia - htm2104

%Kira Whitehouse - kbw2116

%Emily Lemonier - eql2001

% Team 11 - The Pumpkin Widgets / Roomba Whisperers


function hw_3_Team11(serPort)
    
    %gather travels
    [x_scans, y_scans, angle_scans, ~] = detect_room_boundary(serPort, 10000); 
    [~, dim] = size(x_scans);
 
    min_x = abs(min(x_scans));
    min_y = abs(min(y_scans));
    %modify x,y positions s.t. all are positive values
    for i = 1:dim
        x_scans(i) = x_scans(i) + min_x + 1;
        y_scans(i) = y_scans(i) + min_y + 1;
    end
    
    %meters
    robot_diameter = 0.34306;
    
    % create 2D map filled with zeros
    % 0 = we have not traversed this space
    % 1 = there is no object/wall/obstacle in this space
    % -1 = there is an object/wall/obstacle in this space
    big_mama_x = ceil(max(x_scans)/robot_diameter);
    big_mama_y = ceil(max(y_scans)/robot_diameter);
    MAP = zeros(big_mama_y, big_mama_x);

    %current position is clear (void of obstacles)
    %MAP(cur_x, cur_y) = 1;
    %positions we have traversed are clear (void of obstacles)
    for i = 1:dim
        x_pos = round(x_scans(i)/ robot_diameter);
        y_pos = round(y_scans(i)/ robot_diameter);

        MAP(y_pos, x_pos) = 1;
    end
        
    %for each row move from left and from right and fill with -1s
    for i = 1:big_mama_x
        j = 1;
        while (j <= big_mama_y) && (MAP(j, i) ~= 1)
            MAP(j, i) = -1;
            j = j + 1;
        end
        
        j = big_mama_y;
        while (j > 0) && (MAP(j, i) ~= 1)
            MAP(j, i) = -1;
            j = j - 1;
        end 
    end
    
    
    %for each column move from top and from bottom and fill with -1s
    for j = 1:big_mama_y
        i = 1;
        while (i <= big_mama_x) && (MAP(j, i) ~= 1)
            MAP(j, i) = -1;
            i = i + 1;
        end
        
        i = big_mama_x;
        while (i > 0) && (MAP(j, i) ~= 1)
            MAP(j, i) = -1;
            i = i - 1;
        end
    end
    
    disp(MAP);
    
    %set current position
    cur_x = round(x_scans(end)/ robot_diameter);
    cur_y = round(y_scans(end)/ robot_diameter);
    
    %current angle
    cur_a = angle_scans(end);
    
    disp('current positionx, positiony, angle');
    disp(cur_x);
    disp(cur_y);
    disp(cur_a * 180/pi);
        
    dest_x = 1;
    dest_y = 1;
    
    %while zeros are still in the map
    %CHANGE TO WHILE
    if contains_zeros(MAP)
        
        %find a zero
        for i = 1:big_mama_x
            for j = 1:big_mama_y
                if(MAP(j, i) == 0)
                    dest_x = i;
                    dest_y = j;
                end
            end
        end
        
        disp(dest_x);
        disp(dest_y);
        
        
        %calculate angle from current position to zero
        angle = calculate_angle(cur_a, cur_x, cur_y, dest_x, dest_y);
        turnAngle(serPort, 0.1, angle);
        
        disp(angle);

        %calculate angle from current position to zero
        distance = calculate_distance(cur_x, cur_y, dest_x, dest_y);

        disp(distance);
        
        %use bug 2 to move forward distance to zero
        %update current position and map while moving
        %if we can't reach the zero position, mark zero position with -1
        [MAP, cur_x, cur_y, cur_a, success] = bug_2(serPort, distance, MAP, cur_x, cur_y, cur_a);
        disp(MAP);
        disp(cur_x);
        disp(cur_y);
        disp(cur_a);
        disp(success);
        MAP(dest_y, dest_x) = success;
        disp(MAP);
        
    end
end

function zeros = contains_zeros(map)
    for r = 1:size(map, 1)
        for c = 1:size(map, 2)
            if (map(r, c) == 0)
                zeros = true;
                return;
            end
        end
    end
end
 

function dist = calculate_distance(start_x, start_y, goal_x, goal_y)
    dist = sqrt((goal_y - start_y)^2 + (goal_x - start_x)^2);
    dist = dist * 0.34306;
end

function angle = calculate_angle(start_angle, start_x, start_y, goal_x, goal_y)
    if(start_y == goal_y)
        if(start_x > goal_x)
            angle = pi;
        else
            angle = 0;
        end
    elseif(start_x == goal_x)
        if(start_y > goal_y)
            angle = 3.0*pi/2.0;
        else
            angle = pi/2.0;
        end
        return
    else
        delta_y = goal_y - start_y;
        delta_x = goal_x - start_x;
        sign_y = delta_y/abs(delta_y);
        sign_x = delta_x/abs(delta_x);
        
        angle = (pi/2 - (pi/2)*sign_x) + sign_y*(atan(delta_y/delta_x));
        
    end
    angle = angle - start_angle;
end


function [MAP, cur_x, cur_y, cur_a, success] = bug_2(serPort, distance, MAP, cur_x, cur_y, cur_a)
    
    [x_scans, y_scans, angles, success] = detect_room_boundary(serPort, distance);
    
    %meters
    robot_diameter = 0.34306;

    %positions we have traversed are clear (void of obstacles)
    for i = 1:size(x_scans)
        x_pos = round(x_scans(i)/ robot_diameter);
        y_pos = round(y_scans(i)/ robot_diameter);

        MAP(y_pos, x_pos) = 1;
    end
    
end


%used to trace room boundary
function [x, y, angles, success] = detect_room_boundary(serPort, goal_distance)
%=============================================================%
    % Description                                                 %
    %=============================================================%
    % This is a simple solution for HW 2 built on HW 1's solution %
    %=============================================================%
    % Poll for bump Sensors to avoid getting NaN values when the 
    % robot first hits a wall
    [BumpRight BumpLeft WheDropRight WheDropLeft WheDropCaster ...
              BumpFront] = BumpsWheelDropsSensorsRoomba(serPort); % Avoid NaaN, yum
          
    WallSensorReadRoomba(serPort);       % Reset Everything at start
    DistanceSensorRoomba(serPort);      
    AngleSensorRoomba(serPort);           
    %=============================================================%
    
    %%
    %=======================%
    % Position Declaration  %
    %=======================%
    
    % Current Position
    current_pos_normal = 0;     % Normal to M-Line path
    current_pos_tangential = 0; % Along the M-Line direction
    current_angle = 0;
    prev_proximity = inf;       % To ensure we are approaching
    
    % Hit Position Variables
    hit_pos_normal = 0;
    hit_pos_tangential = 0;
    hit_angle = 0;
    hit_proximity_to_goal = inf;
    
    %%
    %=======================%
    % Velocity Declaration  %
    %=======================%
    velocity_val = 0.2;
    angular_velocity_val = 0.1;
    %=======================%

    %=======================%
    % Distance Thresholds   %
    %=======================%
    dist_thresh_from_goal = 0.1;
    dist_thresh_from_hit  = 0.2;
    slope_thresh_for_m_line = 0.1; %%% experiment with thizz bound
    %=======================%
   
    %=======================%
    % Graph Variables       %
    %=======================%
    x = [];                 %x position of roomba
    y = [];                 %y position of roomba
    u = [];                 %x vector component of orientation
    v = [];                 %y vector component of orientation
    angles = [];
    
    %=======================%
    % Status Variable       %
    %=======================%
    status = 1;             % 1 -> Move Forward, 
                            % 2 -> Wall Follow | Haven't left the threshold of the hit point
                            % 3 -> Wall Follow | Left the threshold of the hit point
                            % 4 -> On M-Line so Turn towards Goal  
                            % 5 -> Stop at Goal Position, Success
                            % 6 -> Trapped and thus Failure
                            % 5 -> Turn around if moving away on M-Line
    %=============================================================%
    
    %% Main Loop
    while 1 
        
        %%
        %=============================================================%
        % Step 1 - Read Sensors                                       %
        %=============================================================%
        % For each loop, first read data from the robot sensor
        [BumpRight, BumpLeft, ~, ~, ~, BumpFront]  ...
            = BumpsWheelDropsSensorsRoomba(serPort);                  % Poll for Bump Sensors
        Wall = WallSensorReadRoomba(serPort);                         % Poll for Wall Sensor
        distance_temp = DistanceSensorRoomba(serPort);                % Poll for Distance delta
        angle_temp = AngleSensorRoomba(serPort)*180/pi;                      % Poll for Angle delta
        pause(0.1);
        %=============================================================%
        
        %%
        %=============================================================%
        % Step 2 - Update Odometry                                    %
        %=============================================================%
        % Keep tracking the position and angle in space
        current_angle = mod((current_angle + angle_temp + 360), 360);
        current_pos_normal = current_pos_normal + sin(current_angle*pi/180) * distance_temp;
        current_pos_tangential = current_pos_tangential + cos(current_angle*pi/180) * distance_temp;
        
        hit_angle = hit_angle + angle_temp;
        hit_pos_normal = hit_pos_normal + sin(hit_angle*pi/180) * distance_temp;
        hit_pos_tangential = hit_pos_tangential + cos(hit_angle*pi/180) * distance_temp;
        pause(0.1);
        
        %drawnow;            % not entirely sure what thizzz is ?
        %=============================================================%
        
        %%
        %=============================================================%
        % Step 2.5 - Update Information for graphing Pos + Orientation%
        %=============================================================%
        % Keep tracking the position and angle in space
        
        %if bumped
            angles(end +1) = current_angle * pi/180;
            u(end + 1) = cos(current_angle * pi/180);
            v(end + 1) = sin(current_angle * pi/180);
            x(end + 1) = current_pos_tangential;
            y(end + 1) = current_pos_normal;    
        %end
        
        pause(0.1);
        %drawnow;            % not entirely sure what thizzz is ?
        %=============================================================%        
        
        %%
        %=============================================================%
        % Step 3 - Calculate Euclidean Distances                      %
        %=============================================================%
        hit_distance = sqrt(hit_pos_normal ^ 2 + hit_pos_tangential ^ 2); % point of reference
        proximity = sqrt( (goal_distance - current_pos_tangential) ^2 + current_pos_normal^ 2); % are we there yet?
        slope_m = abs(current_pos_normal / current_pos_tangential); % used to fix orientation
        %=============================================================%


        %%
        %=============================================================%
        % Step 3.5 - Debugging                                        %
        %=============================================================%
        %display(status)
        %display(current_pos_normal)
        %display(current_pos_tangential)
        %display(proximity);
        %display(hit_proximity_to_goal);
        %display(angle_temp);
        %display(current_angle)
        %display(slope_m);

        %%
        %=============================================================%
        % Step 4 - Check Status                                       %
        %=============================================================%
        switch status
            case 1 % Move Forward, Theoretically should always be on M-Line
                if (proximity < dist_thresh_from_goal)
                    status = 5;
                elseif (slope_m > slope_thresh_for_m_line ) %fix orientation
                    status = 4;
                elseif (prev_proximity < proximity) % turn around, moving away from goal
                    status = 7;
                elseif (BumpRight || BumpLeft || BumpFront)
                    status = 2; % Change Status to Wall Follow
                    hit_angle = 0;  % Mark this as origin of Hit Coord Space
                    hit_pos_normal = 0;  % So we know if we return here
                    hit_pos_tangential = 0;  % and mark proximity in case we
                    hit_proximity_to_goal = proximity; % re-encounter the M-Line
                    bumped = true;
                end
                prev_proximity = proximity;
                SetFwdVelAngVelCreate(serPort, velocity_val, 0 );
            case 2 % Wall Follow | Haven't left the threshold of the hit point
                WallFollow(velocity_val, angular_velocity_val, BumpRight, BumpLeft, BumpFront, Wall, serPort);
                if (hit_distance > dist_thresh_from_hit)
                    status = 3;
                end
            case 3 % Wall Follow | Left the threshold of the hit point
                WallFollow(velocity_val, angular_velocity_val, BumpRight, BumpLeft, BumpFront, Wall, serPort);
                %check for M-line and closer than hit point   
                if (slope_m < slope_thresh_for_m_line && proximity < hit_proximity_to_goal)
                   status = 4;
                elseif(hit_distance < dist_thresh_from_hit)
                   status = 6; % womp
                end
            case 4 % Orient yourself on M-Line and just keep swimming            
                turnAngle(serPort, angular_velocity_val, -current_angle);
                if (330 < current_angle) || (current_angle < 30) % experiment with thizzz
                    SetFwdVelAngVelCreate(serPort, velocity_val, 0 );
                    status = 1;
                end
            case 5 % SUCCESS, Victory Spin
                fprintf('Robot stopped at Goal point\n');
                SetFwdVelAngVelCreate(serPort, 0, 0 );
                fprintf('Robot is ready for next command\n');
                
                success = 1;
                displayGraph(x, y, u, v);
                return;
            case 6 % FAILURE
                fprintf('Robot failed to reach goal\n');
                SetFwdVelAngVelCreate(serPort, 0, 0 );
                fprintf('[ERROR] :: DELETING FOLLOWING LAWS\n');
                fprintf('A robot may not injure a human being\n');
                fprintf('or, through inaction, allow a human being\n');
                fprintf('to come to harm.\n\n');
                fprintf('A robot must obey the orders given to it by\n');
                fprintf('human beings, except where such orders would\n');
                fprintf('conflict with the First Law.\n\n');
                fprintf('A robot must protect its own existence\n');
                fprintf('as long as such protection does not\n');
                fprintf('conflict with the First or Second Law.\n');
                displayGraph(x, y, u, v);
                success = -1;
               
                return;
            case 7 % passed the goal
                SetFwdVelAngVelCreate(serPort, 0, 0 );
                turnAngle(serPort, angular_velocity_val, 180); % not sure if complete solution, but has worked
                status = 1;
        end
    end
end


%%
% Wall Follow Function
function WallFollow(velocity_val, angular_velocity_val, BumpRight, BumpLeft, BumpFront, Wall, serPort)

    % Angle Velocity for different bumps
    av_bumpright =  4 * angular_velocity_val;
    av_bumpleft  =  2 * angular_velocity_val;
    av_bumpfront =  3 * angular_velocity_val;
    av_nowall    = -4 * angular_velocity_val;
    
    if BumpRight || BumpLeft || BumpFront
        v = 0;                              % Set Velocity to 0
    elseif ~Wall
        v = 0.25 * velocity_val;            % Set Velocity to 1/4 of the default
    else
        v = velocity_val;                   % Set Velocity to the default
    end

    if BumpRight
    av = av_bumpright;                      % Set Angular Velocity to av_bumpright
    elseif BumpLeft
        av = av_bumpleft;                   % Set Angular Velocity to av_bumpleft
    elseif BumpFront
        av = av_bumpfront;                  % Set Angular Velocity to av_bumpfront
    elseif ~Wall
        av = av_nowall;                     % Set Angular Velocity to av_nowall
    else
        av = 0;                             % Set Angular Velocity to 0
    end
    SetFwdVelAngVelCreate(serPort, v, av );
    pause(0.1);
end

%%
%Graph Function
function displayGraph(x_position, y_position, x_angle, y_angle)
    figure;
    quiver(x_position, y_position, x_angle, y_angle);
    title('Position and Orientation of Robot Relative to Initial State.');
    xlabel('X Position');
    ylabel('Y Position');
end