%HW2 - Team 11%

%Henrique Maia - htm2104

%Kira Whitehouse - kbw2116

%Emily Lemonier - eql2001

% Team 11 - The Pumpkin Widgets

function hw_2_Team11(serPort)

    global Goal td Angle ClosestX ClosestY;
    td = .05;   % overwrite/increase default td so functions crash less
    
    SetDriveWheelsCreate(serPort, 0, 0) % Stop collaborate and listen.
    DistanceSensorRoomba(serPort);  %reset the distance sensor
    AngleSensorRoomba(serPort);     %reset the angle sensor
    
    Goal = 0;
    Angle = 0;                      %always start facing goal
    ClosestX = inf;
    ClosestY = inf;

    while ~Goal % will be set to true by Goal Reached in Odom    
        
        turnToGoal(serPort);

        moveToGoal(serPort);
        
        wallFollow(serPort);
        
    end

    SetDriveWheelsCreate(serPort, 0, 0);  % Full Stop
    display('FINISHED');                  % Home Sweet Home
end

function turnToGoal(serPort)
    global Angle Exit;
    Exit = 0;
    
    while Angle ~= 0
        %turn SLOWLY until global angle is zero
        pause(0.1);
    end

end

function moveToGoal(serPort)
    global Goal;
    
    %Hit = check_if_Hit
    while ~Goal && ~Hit && ~Exit
        %move forward
        pause(0.1);
        %Hit = check_if_Hit
        %%%%%%%stop? -- not needed since taken care of in Odom?
        %get info on current position
        %pass info to Odom (check Goal) (will call stop if on M-Line)
        pause(0.1);
    end
end

function Odom
    global Goal Angle ClosestX ClosestY DistanceToGoal Exit;
    
    %calculate if on M-Line
    
    if on the M-Line 
        SetDriveWheelsCreate(serPort, 0, 0) % Stop
        %DistanceToGoal = calculate distance to goal
        
        if DistanceToGoal == 0
            display('SUCCESS');
            Goal = 1;
        elseif %closer than last hit coordinate
            Exit = 1;
        elseif %BACK at closest hit point (make sure doesnt trigger at the original hit at this point)
            display('FAILURE');
            Goal = 1; % Go home, you're drunk
        end
    end
        

end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Old Code from hw 1 below
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

function wallFollow(serPort)
    global Goal Exit;

    avg_count = 3; %how many polls of the sensor to make
    d = 0;
    searchingForWall = 0;
    MIN_WALL_STRENGTH = 15; % minimal wall strength 
    
    while ~Goal && ~Exit

        [BumpRight, ~, ~, ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        wallStrength = 0;
        if(~isnan(BumpRight) || ~isnan(BumpFront))
            if (BumpFront)
                display('Bump Front')
                SetDriveWheelsCreate(serPort, 0, 0);    
                turnAngle(serPort, 0.1, 30); %turn away from front bump and locate wall
                display('Survey')                
                Survey(serPort)
                %update odometry
                UpdateOdometry(AngleSensorRoomba(serPort), DistanceSensorRoomba(serPort));

                wallStrength = AnalogWallSensorReadRoomba(serPort);
            end
            if (BumpRight)
%                 display(BumpRight)
%                 SetDriveWheelsCreate(serPort, 0, 0);
                SetDriveWheelsCreate(serPort, -0.1, 0.1);
                pause(0.1);
                Survey(serPort)
            end
            
        end
        max = 0;
        i = 0;
        while i < avg_count
            SetDriveWheelsCreate(serPort, 0, 0);
            pause(0.1)
            wallStrength = AnalogWallSensorReadRoomba(serPort); 
            pause(0.1)
            if(isnan(wallStrength))
                i = i - 1;
                continue;
            end
            if max < wallStrength
                max = wallStrength;
            end
            i = i + 1;
        end
        wallStrength = max;
        
        display(wallStrength)
        SetDriveWheelsCreate(serPort, 0, 0);
        pause(0.05)
       
        if(wallStrength > MIN_WALL_STRENGTH)
            SetDriveWheelsCreate(serPort, 0.15, 0.15); 
            pause(0.2)
            display('onward');
        else
            SetFwdVelRadiusRoomba(serPort, 0.1, -0.1);
            display('lost wall sensor');
            UpdateOdometry(AngleSensorRoomba(serPort), DistanceSensorRoomba(serPort));

        end
        
        pause(0.1);
    end

end

%Survey in a circle for the strongest wall-IR-sensor reading
%Should align itself orthogonal to wall once completed
%If it cannot find a wall it spirals in search of one
function Survey(serPort)
    prevStrength = 0;
    w_speed = 0.05;
    turn_count = 0;
    while turn_count < 30
        SetDriveWheelsCreate(serPort, w_speed, -w_speed);   % turn away from wall
        pause(0.1);
        SetDriveWheelsCreate(serPort, 0, 0);                % Stop for steady signal
        pause(0.1);
        wallStrength = AnalogWallSensorReadRoomba(serPort); % Scan for signal
        pause(0.1)
        if(prevStrength >= wallStrength && prevStrength > 10) % if signal was valid and decreases
            break;                                            %stop scanning
        end
        if (wallStrength > prevStrength + 4) % if slight improvement
            prevStrength = wallStrength;     % accept new value
        end
        turn_count = turn_count + 1;
        pause(0.1)
    end
    if(turn_count == 30)                            % if you went full circle and never found a signal
        SetFwdVelRadiusRoomba(serPort, 0.1, -0.15); % spin around to find the wall
        display('find the wall');        
    else
        SetDriveWheelsCreate(serPort, -w_speed, w_speed); % pivot back once to return to parallel
        pause(0.2)
        SetDriveWheelsCreate(serPort, 0, 0);
    end
end


function UpdateOdometry(angle, distance)
    global TotalX TotalY TotalAngle NetDistance;
    
    angle = angle * 180 / pi;
    disp('angle before');
    disp(angle);
    disp('distance');
    disp(distance);
    
    if(angle < -180)
        angle = angle + 360;
    end

    TotalAngle = TotalAngle + angle;   

    TotalX = TotalX + (cos(TotalAngle * pi/180)*distance)*100;
    TotalY = TotalY + (sin(TotalAngle * pi/180)*distance)*100; 
    
    fprintf('x: %8.4f\ny: %8.4f\na: %6.3f\n', TotalX, TotalY, TotalAngle);
    absAng = abs(abs(TotalAngle) - 360);
    if (absAng < 15)
        if(abs(TotalX) < 80 && abs(TotalY) < 80)
            NetDistance = 0;
        end
    end
     
end
