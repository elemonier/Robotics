% Kira Whitehouse
% kbw2116

function kirasVersion(port)

%Initialize variables
v= 0.3;                     % Forward velocity (m/s)
w= 0;                       % Angular velocity (rad/s)
totalDistanceTraveled = 0;  % Distance meter
totalAngleTurned = 0;       % Angle meter
goalX = 0;                  % Initial X position
goalY = 0;                  % Initial Y position
timeout = 1200;             % Time until program exits


%Move robot forwards until we hit something
SetFwdVelAngVelCreate(port, v, w);
pause(0.1);

%Check bump sensors
[BumpRight, BumpLeft, ~, ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(port);
bumped = 0;
while isnan(BumpRight) || isnan(BumpLeft) || isnan(BumpFront)
    bumped = 0;
end

bumped= BumpRight || BumpLeft || BumpFront;

while ~bumped
    [BumpRight, BumpLeft, ~, ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(port);
    bumped= BumpRight || BumpLeft || BumpFront;
    pause(0.1);
end

disp('bumped');
SetFwdVelAngVelCreate(port, 0, 0);
pause(0.1);

if BumpRight
    disp('right');
    rotate(port, 0.2);
elseif BumpLeft
    disp('left');
    rotate(port, -0.2);
elseif BumpFront
    disp('front');
    rotate(port, 0.2);
end

% Reset angular and linear odometry
AngleSensorRoomba(port);     
DistanceSensorRoomba(port);

while (goalX ~= 0 && goalY ~= 0 && timeout > 0) || (totalDistanceTraveled == 0) 
    SetFwdVelAngVelCreate(port, 0.1, 0);
    pause(0.1);
    
    Wall = WallSignalRoomba(port);
    [BumpRight, BumpLeft, ~, ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(port);
    bumped= BumpRight || BumpLeft || BumpFront;

    if (Wall < 20)
            disp('veered');
            SetFwdVelAngVelCreate(port, 0.0, 0);
            SetFwdVelRadiusRoomba(port, 0.08, -0.1);

        angle = 0;
        %1.5 is ~ pi/2; sets maximum bound on veer rotation
        while (Wall < 20) && ~bumped && (angle < 1.2)
            pause(0.1);
            Wall = WallSignalRoomba(port);
            [BumpRight, BumpLeft, ~, ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(port);
            bumped= BumpRight || BumpLeft || BumpFront;
            angle = angle + abs(AngleSensorRoomba(port));
            disp(angle);
        end
        SetFwdVelAngVelCreate(port, 0.0, 0);
        disp('done turning after veer');
    end
    
    if BumpFront
        disp('bump front inside loop');
        rotate(port, 0.2);
        disp('done rotating after front bump');
            
    end    
end 
%}
%{
% Stop robot motion
v= 0;
w= 0;
SetFwdVelAngVelCreate(port,v,w);
%}
% If you call RoombaInit inside the control program, this would be a
% good place to clean up the serial port with...
% fclose(port)
% delete(port)
% clear(port)
% Don't use these if you call RoombaInit prior to the control program
end


%rotate clockwise or counterclockwise
%input direction is boolean, velocity is signed angular rotation
function rotate(port, angularVelocity)

%read wall sensor
Wall = WallSignalRoomba(port);
disp(Wall);

%turn until we sense a wall on our right
while (Wall < 30)
	SetFwdVelAngVelCreate(port, 0, angularVelocity);
	pause(0.1);
	Wall = WallSignalRoomba(port);
end



%zero angular velocity
SetFwdVelAngVelCreate(port, 0, 0);
pause(0.1);

end



function w= v2w(v)
% Calculate the maximum allowable angular velocity from the linear velocity
%
% Input:
% v - Forward velocity of Create (m/s)
%
% Output:
% w - Angular velocity of Create (rad/s)
    
    % Robot constants
    maxWheelVel= 0.49;   % Max linear velocity of each drive wheel (m/s)
    robotRadius= 0.2;   % Radius of the robot (m)
    
    % Max velocity combinations obey rule v+wr <= v_max
    w= (maxWheelVel-v)/robotRadius;
end