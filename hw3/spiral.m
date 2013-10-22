function spiral(port)
%Spiral around room in rectalinear fashion (all angles 90 degrees)

    %setup sensors
    WallSensorReadRoomba(port);      
    DistanceSensorRoomba(port);      
    AngleSensorRoomba(port); 
    [BumpRight, BumpLeft, ~, ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(port);
    
    %setup timer
    tic;
    time_limit = 120;
    timer = toc;
    
    %roomba info
    robot_diameter = 0.34306;
    
    %map/positioning info
    big_mama = 10;
    map = zeros(big_mama, big_mama);
    robot_x = big_mama/2;
    robot_y = big_mama/2;
    robot_angle = 0;

    
    %while we haven't run out of time
    while timer < time_limit
        
        %move forward one robot diameter
        SetFwdVelAngVelCreate(port, 0.1, 0 );
        dist = DistanceSensorRoomba(port); 
        while dist < robot_diameter
            dist = dist + DistanceSensorRoomba(port);
            pause(0.1);
        end
        SetFwdVelAngVelCreate(port, 0, 0);
        
        %update bumps
        [BumpRight, BumpLeft, ~, ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(port);
        
        %if we hit something, mark as obstacle
        %HEADS UP: these are untested --> i have been testing the code with
        %an empty simulator; no walls/obstacles = no bumps to try out!
        %we need to circle the obj here and update the map
        if BumpRight
            right_x = sin_d(robot_angle);
            right_y = -cos_d(robot_angle);
            map(robot_y + right_y, robot_x + right_x) = -1; 
        elseif BumpLeft
            left_x = -sin_d(robot_angle);
            left_y = -cos_d(robot_angle);
            map(robot_y + left_y, robot_x + left_x) = -1; 
        elseif BumpFront
            map(robot_y, robot_x) = -1;
            
        %if we didn't hit anything
        else
            %no obstacles in square we just traversed
            map(robot_y, robot_x) = 1;
                      
            %update our last position
            robot_x = robot_x + cos_d(robot_angle);
            robot_y = robot_y - sin_d(robot_angle);
      
            %check if we should move foward one more square or turn
            left_x = -sin_d(robot_angle);
            left_y = -cos_d(robot_angle);
 
            %error checking
            fprintf('robot position x: %d   y: %d   ang: %d\n', robot_x, robot_y, robot_angle);
            fprintf('check to our left, x: %d   y: %d\n', left_x, left_y);
            
            %if map to our left is untraversed
            if map(robot_y + left_y, robot_x + left_x) == 0
                %disp('turning 90 degrees');
                
                %turn angle = 90 degrees
                SetFwdVelAngVelCreate(port, 0, 0.2);
                angle = AngleSensorRoomba(port);
                while angle < pi/2.0
                    angle = angle + abs(AngleSensorRoomba(port));
                    pause(0.1);
                end
                SetFwdVelAngVelCreate(port, 0, 0);
                robot_angle = robot_angle + 90;
                
            end
            
        end
        
        %update timer
        timer = toc;
    end
    
    %display map of obstacles
    disp(map);
    
end

%computes cosine given degrees
function c = cos_d(degrees)
    c = round(cos(degrees * pi/180));
end

%computes sine given degrees
function s = sin_d(degrees)
    s = round(sin(degrees * pi/180));
end

