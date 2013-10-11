function distTravel= FollowWall(serPort)
% Simple autonomous wall following program for the iRobot Create.
%
% For the physical Create only, it is assumed that the function call
% serPort= RoombaInit(comPort) was done prior to running this program.
% Calling RoombaInit is unnecessary if using the simulator.
%
% Input:
% serPort - Serial port object, used for communicating over bluetooth
%
% Output:
% distTravel - distance travelled from start
%

    % Set constants for this program
    maxFwdVel= 0.2;     % Max allowable forward velocity
    
    % Struct for storing data sent or recieved from robot
    robotData= struct('contactDir', BumpDirection.None, ...
        'maxFwdVel', maxFwdVel, 'fwdVel', 0, 'turnRad', 0, ...
        'wallFollowDir', BumpDirection.Right, 'totalDist', 0,...
        'distDiff', 0, 'heading', 0, 'initWallHit', [0,0],...
        'coord', [0,0], 'wallSense', false);
    
    % Start robot moving forward until contact
    SetDriveWheelsCreate(serPort, robotData.maxFwdVel,...
        robotData.maxFwdVel)
    disp('travel to contact')
    while robotData.contactDir == BumpDirection.None
        [robotData.contactDir, robotData.wallSense]= WallAndBumpSensors(serPort);
        pause(0.1)
    end
    
    % Stop the robot
    SetDriveWheelsCreate(serPort, 0, 0)
    disp('hit wall and stopped')
    
    pause(1)
    
    % Record location
    robotData= updateCoord(serPort, robotData, false);
    robotData.initWallHit= robotData.coord;
    
    % Reset the robot for good wall following angle
    robotData= setupFollowAngle(serPort, robotData);
    
%     while true
%         % Follow the wall
%         robotData= straightWallFollow(serPort, robotData);
%         
%         % Resolve why the wall following was interrupted and find the
%         % next wall
%         robotData= resolveDisruption(serPort, robotData);
%         pause(0.1)
%     end
        
    distTravel= 0;
end

function robotData= setupFollowAngle(serPort, robotData)
    disp('setup angle')
    counterClockwise= eps;
    clockwise= -eps;
    % Default radius setting for in place counter-clockwise turn
    robotData.turnRad= counterClockwise;
    % Low forward velocity for in place turn
    robotData.fwdVel= 0.003;
    
    % Check for previous wall follwing, or a bump to the left from the
    % start, and assign other values than the default turn direction
    if robotData.wallFollowDir == BumpDirection.Right
        robotData.turnRad= counterClockwise;
    elseif robotData.wallFollowDir == BumpDirection.Left
        robotData.turnRad= clockwise;
    elseif robotData.contactDir == BumpDirection.Left
        robotData.turnRad= clockwise;
    end
    
    disp('turn away from contact') 
    SetFwdVelRadiusRoomba(serPort, robotData.fwdVel, robotData.turnRad)
    while robotData.contactDir ~= BumpDirection.None
        [robotData.contactDir, robotData.wallSense]= WallAndBumpSensors(serPort);
        pause(0.1)       
    end
    % Stop the robot
    SetDriveWheelsCreate(serPort, 0, 0)
    
    disp('turn to contact')
    robotData.turnRad= -robotData.turnRad;
    SetFwdVelRadiusRoomba(serPort, robotData.fwdVel, robotData.turnRad)
    while robotData.contactDir == BumpDirection.None
        [robotData.contactDir, robotData.wallSense]= WallAndBumpSensors(serPort);
        pause(0.1)
    end
    disp('angle set and stopping')
    % Stop the robot
    SetDriveWheelsCreate(serPort, 0, 0)
    
    robotData.wallFollowDir= robotData.contactDir;
end

function robotData= straightWallFollow(serPort, robotData)
    [robotData.contactDir, robotData.wallSense]= WallAndBumpSensors(serPort);
    
    if robotData.contactDir == BumpDirection.None
        warning('No bump direction in wall follow call')
    else
        disp('following wall')
        SetDriveWheelsCreate(serPort, robotData.maxFwdVel,...
            robotData.maxFwdVel)
        while robotData.wallSense
            [robotData.contactDir, robotData.wallSense]= WallAndBumpSensors(serPort);
            
            % Add stuck abort code here
            
            pause(0.1)
        end
                    
        disp('wall follow disrupted, stopping')
        SetDriveWheelsCreate(serPort, 0, 0)
        % Update coordinates
        robotData= updateCoord(serPort, robotData, false);
    end
end

function robotData= resolveDisruption(serPort, robotData)
    if robotData.contactDir == BumpDirection.None
        disp('lost contact')
        robotData= turnCorner(serPort, robotData);
    elseif robotData.contactDir == BumpDirection.Front ||...
            robotData.contactDir ~= robotData.wallFollowDir
        disp('contact front')
        robotData= setupFollowAngle(serPort, robotData);
    else    % Stuck condition or incorrect angle
        disp('stuck or angle incorrect, correcting')
        robotData= setupFollowAngle(serPort, robotData);
    end
    
    % Fringe case: why is this necessary?
    if robotData.contactDir == BumpDirection.None
        warning('correct angle not set for wall following')
        if robotData.wallFollowDir == BumpDirection.Right
            robotData.turnRad= eps;
        elseif robotData.wallFollowDir == BumpDirection.Left
            robotData.turnRad= -eps;
        end
        robotData.fwdVel= .001;
        
        disp('regaining wall') 
        SetFwdVelRadiusRoomba(serPort, robotData.fwdVel, robotData.turnRad)
        while ~robotData.wallSense
            [robotData.contactDir, robotData.wallSense]= WallAndBumpSensors(serPort);
            pause(0.1)       
        end
        disp('wall found, stopping')
        SetDriveWheelsCreate(serPort, 0, 0)
    end
end

function robotData= turnCorner(serPort, robotData)
    counterClockwise= .1;
    clockwise= -.1;
    
    if robotData.wallFollowDir == BumpDirection.Right
        robotData.turnRad= clockwise;
    else
        robotData.turnRad= counterClockwise;
    end
    turnRad= robotData.turnRad; % Retain turn radius
    robotData.fwdVel= maxVel(robotData.turnRad);
    
    disp('round corner to contact')
    SetFwdVelRadiusRoomba(serPort, robotData.fwdVel, robotData.turnRad)
    while robotData.contactDir == BumpDirection.None
        [robotData.contactDir, robotData.wallSense]= WallAndBumpSensors(serPort);
        pause(0.1)
    end
    
    disp('found wall and stopped')
    SetDriveWheelsCreate(serPort, 0, 0)
    
    robotData= setupFollowAngle(serPort, robotData);
    
    % Update coordinates
    robotData.turnRad= turnRad;
    robotData= updateCoord(serPort, robotData, true);
end

function robotData= updateCoord(serPort, robotData, wasTurn)
    % Compute new coordinates
%     disp(' ')
    distDiff= DistanceSensorRoomba(serPort);
    
    if wasTurn % Find chord length
%         disp(['turn rad: ', robotData.turnRad])
        turnAngle= distDiff/2*pi*abs(robotData.turnRad);
        distDiff= 2*abs(robotData.turnRad)*sin(turnAngle/2);
    end
    
    robotData.totalDist= robotData.totalDist + distDiff;
    robotData.heading= robotData.heading + AngleSensorRoomba(serPort);
    
    if robotData.heading > pi
        robotData.heading= robotData.heading - 2*pi;
    elseif robotData.heading <= -pi
        robotData.heading= robotData.heading + 2*pi;
    end
    
    dX= distDiff * cos(robotData.heading);
    dY= distDiff * sin(robotData.heading);
    robotData.coord(1)= robotData.coord(1) + dX;
    robotData.coord(2)= robotData.coord(2) + dY;
    
%     disp(['distdiff: ', num2str(distDiff)])
%     disp(['totalDist: ', num2str(robotData.totalDist)])
%     disp(['heading: ', num2str(robotData.heading)])
%     disp(['updated coord: ', num2str(robotData.coord(1)), ' ',...
%         num2str(robotData.coord(2))])
%     disp(' ')
end

function v= maxVel(turnRadius)
% Calculate the maximum allowable angular velocity from the linear velocity
%
% Input:
% v - Forward velocity of Create (m/s)
%
% Output:
% w - Angular velocity of Create (rad/s)
    % Robot constants
    maxWheelVel= 0.5;   % Max linear velocity of each drive wheel (m/s)
    robotRadius= 0.2;   % Radius of the robot (m)
    
    turnRadius= abs(turnRadius);
    
    if isinf(turnRadius)
        turnRadius= 32.768;
    elseif turnRadius == eps
        turnRadius= 0.001;
    elseif turnRadius > 2
        turnRadius= 2;
    elseif turnRadius == 0
        turnRadius= 0.001;
    end
    
    v= (maxWheelVel * turnRadius)/(robotRadius + turnRadius);
end