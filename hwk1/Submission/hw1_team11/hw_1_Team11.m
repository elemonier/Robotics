%HW1 - Team 11%

%Henrique Maia - htm2104

%Kira Whitehouse - kbw2116

%Emily Lemonier - eql2001

% Team 11 - The Roomba Whisperers

% We have commented throughout the code
% so that our accomodations for edge cases
% and other common issues may be best understood
% Feel free to reach out for questions or comments

function hw_1_Team11(serPort)

    global NetDistance td TotalX TotalY TotalAngle;
    td = .05;   % overwrite/increase default td so functions crash less
    
    DistanceSensorRoomba(serPort);  %reset the distance sensor
    AngleSensorRoomba(serPort);     %reset the angle sensor
    SetDriveWheelsCreate(serPort, 0.2, 0.2); % Go forth and conquer!
    
    x = 0.0;    
    while true % searching for initial hit
        [BumpRight, BumpLeft, ~, ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        pause(0.1)
        if(~isnan(BumpRight) || ~isnan(BumpLeft) || ~isnan(BumpFront)) % error checking
            if (BumpRight || BumpLeft || BumpFront)
                display('Bump in the Night');
                SetDriveWheelsCreate(serPort, 0, 0) % Stop collaborate and listen.
                x = DistanceSensorRoomba(serPort);  % grab initial dist to wall from start
                if BumpLeft
                    turnAngle(serPort, 0.1, 40); %arbitrary helpful first turn
                elseif BumpRight
                    disp('bump right');
                    SetDriveWheelsCreate(serPort, -0.1, 0.1);
                elseif BumpFront
                    disp('bump front');
                    turnAngle(serPort, 0.1, 30);
                end                
                NetDistance = inf;
                pause(0.1)
                break
            end
        end
    end

    Survey(serPort) % align orthogonal to wall 
    
    DistanceSensorRoomba(serPort); %reset the distance sensor
    a = AngleSensorRoomba(serPort);    %grab initial angle from start
    
    TotalX = 0;
    TotalY = 0;
    TotalAngle = 0;
    
    SenseSpeedAverage(serPort);    %Embark on a journey
    
    DistanceSensorRoomba(serPort); %reset the distance sensor
    % Return to where you came from    
    turnAngle(serPort, 0.1, 360 - (180*a/pi))
    dist = 0;
    while dist < x;
        SetDriveWheelsCreate(serPort, 0.15, 0.15);
        dist = DistanceSensorRoomba(serPort);
    end
    SetDriveWheelsCreate(serPort, 0, 0);  % Full Stop
    display('FINISHED');                  % Home Sweet Home
end

% Move forward and pass values to odometry
% Takes a max value of wall-sensor for many polls
function SenseSpeedAverage(serPort)
global NetDistance;

    avg_count = 3; %how many polls of the sensor to make
    d = 0;
    searchingForWall = 0;
    MIN_WALL_STRENGTH = 15; % minimal wall strength 
    
    while NetDistance ~= 0  % NetDistance gets

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
