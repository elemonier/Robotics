function CircumAmbulateWall(serPort)%TODO -- change finalRad to desired output

    global Alpha NetDistance To DirectionTo;
    
    SetFwdVelRadiusRoomba(serPort, 0.2, 2); %     -Move forward until any bump
    while true
        [BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        if (BumpRight || BumpLeft || BumpFront)
            SetFwdVelRadiusRoomba(serPort, 0, 2); %Stop
            if(BumpRight)
                To = 'R';
                DirectionTo = -1;
            else
                To = 'L';
                DirectionTo = 1;
            end
            DistanceSensorRoomba(serPort); %reset the distance sensor
            NetDistance = inf;
            break
        end
        pause(0.1)
    end
    display('Variables Set');
    to= BumpingTo(serPort);
    while to == 1
        turnAngle(serPort, 0.2, -DirectionTo*10);
        to= BumpingTo(serPort);
        pause(0.2)
    end
    while to == 0
        turnAngle(serPort, 0.2, DirectionTo*2);
        to= BumpingTo(serPort);
        pause(0.2)
    end
    AngleSensorRoomba(serPort); %reset the angle sensor
    Alpha = 0;
    
    Drag(serPort);
    
    SetFwdVelRadiusRoomba(serPort, 0, 2); % Full Stop
    display('FINISHED');
end

function Coordinates(serPort, deltaDist, deltaAngle)
%     display(deltaDist);
%     display(deltaAngle);
%     persistent TotalX TotalY TotalAngle;
%     global NetDistance;
% 
%     if(TotalAngle)
%     TotalAngle = TotalAngle + deltaAngle;
%     
%     TotalX = TotalX + cos(TotalAngle)*deltaDist;
%     TotalY = TotalY + sin(TotalAngle)*deltaDist;
%     
%     if(TotalX == 0 && TotalY == 0) %or close enough
%         NetDistance = 0;
%     end

%     Alpha = Alpha + deltaAngle;
%     
%     if(angle == 0)
%         NetDistance = NetDistance + (prevAngle * deltaDist); %fix Me
%     end
%     prevAngle = Alpha;
end

function Drag(serPort)
    global DirectionTo NetDistance;
    %tStart= tic;        % Time limit marker
    %maxDuration = 1;
    display('DRAGGING');
    while NetDistance ~= 0
        to= BumpingTo(serPort);
        if to > 0
            dist = DistanceSensorRoomba(serPort);
            display(dist);
            Coordinates(serPort, dist, AngleSensorRoomba(serPort));            
            if to == 2 || dist == 0 %front hit
                SetFwdVelRadiusRoomba(serPort, 0, 2); %stop
                turnAngle(serPort, 0.2, -DirectionTo*15);
                Reset(serPort);                         %fix direction
                pause(0.1)
            end
            SetFwdVelRadiusRoomba(serPort, 0.2, 2); %drive forward
            pause(.1)
        else %lost the wall
            while to == 0 %spiral until hit
                SetFwdVelRadiusRoomba(serPort, 0.1, DirectionTo*pi/24); %complete guess
                %its possible if it spirals too much to hit the wrong
                %bumper here and cause problems with to and away logic
                to= BumpingTo(serPort);
                pause(0.1)
            end
            Reset(serPort);
        end
        pause(0.15)
    end

end

function to= BumpingTo(serPort)
    global To;
    to = 0;
    [BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
    if To == 'R' && BumpRight
        to = 1;
    elseif To == 'L' && BumpLeft
        to = 1;
    elseif BumpFront
        to = 2;   
    end
    display(to);

end

function Reset(serPort)
    global DirectionTo;
    SetFwdVelRadiusRoomba(serPort, 0, inf); %Stop
    to= BumpingTo(serPort);
    while to > 0
        turnAngle(serPort, 0.2, -DirectionTo*10);
        to= BumpingTo(serPort);
        pause(0.1)
    end
    while to == 0
        turnAngle(serPort, 0.2, DirectionTo*2);
        to= BumpingTo(serPort);
        pause(0.1)
    end
%     Do we want to pass zero distance here?
    Coordinates(serPort, 0, AngleSensorRoomba(serPort));
end

% Briefly pause to avoid continuous loop iteration
%  pause(0.1)

% make roomba curve ever greater if it keeps losing wall within small timeframe
%edge cases in code, when it checks position to stop, if we start at a turn
%or place where it has to reset, will we come to a full stop?
%most of the turns/resets will be 90 degrees in cross shape...
