function CircumAmbulateWall(serPort)%TODO -- change finalRad to desired output

    global Angle NetDistance To DirectionTo TotalX TotalY;
    
    SetDriveWheelsCreate(serPort, 0.3, 0.3); %     -Move forward until any bump
    while true
        [BumpRight, BumpLeft, ~, ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        if(~isnan(BumpRight) || ~isnan(BumpLeft) || ~isnan(BumpFront))
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
                TotalX = 0.0;
                TotalY = 0.0;
                break
            end
        end
        pause(0.1)
    end
    display('Variables Set');
    to= BumpingTo(serPort);
    while to > 0 
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
    pause(.2)
    Angle = 0.0;
    
    Drag(serPort);
%     test(serPort);
    
    SetDriveWheelsCreate(serPort, 0, 0); % Full Stop
    display('FINISHED');
end

function test(serPort)
    global DirectionTo;

AngleSensorRoomba(serPort); %reset
display(AngleSensorRoomba(serPort)*180/pi);
turnAngle(serPort, 0.2, DirectionTo*70)
display(AngleSensorRoomba(serPort)*180/pi);

end

function Coordinates(serPort, deltaDist)
    global NetDistance Angle TotalX TotalY;
%     display(deltaDist);
    
    deltaAngle = AngleSensorRoomba(serPort)*180/pi;
    display(deltaAngle);
    display(Angle)
   
    if(deltaDist > 0)

        TotalX = TotalX + (cos(Angle*pi/180)*deltaDist)*100;
        TotalY = TotalY + (sin(Angle*pi/180)*deltaDist)*100;            
        absAng = abs(abs(Angle) - 360);
        if (absAng < 10)
            
            if(TotalX < 100 && TotalX > -100) % or close enough
                if(TotalY < 100 && TotalY > -100)
                    SetDriveWheelsCreate(serPort, 0, 0); % Full Stop
                    NetDistance = 0;
                end
            end
        end

    end
    
    Angle = Angle + deltaAngle;
    
    display(TotalX)
    display(TotalY)

end

function Drag(serPort)
    global DirectionTo NetDistance;
    %tStart= tic;        % Time limit marker
    %maxDuration = 1;
    display('DRAGGING');
    AngleSensorRoomba(serPort);
    while NetDistance ~= 0
        to= BumpingTo(serPort);
        if to > 0
            dist = DistanceSensorRoomba(serPort);
            Coordinates(serPort, dist); %angle should be zero if straight wall
            pause(0.1)
            if to == 2 || dist == 0 %front hit
                SetDriveWheelsCreate(serPort, 0.1, 0.1); %stop
                turnAngle(serPort, 0.2, -DirectionTo*25);
                pause(0.1)
                Reset(serPort);                         %fix direction
            end
            SetDriveWheelsCreate(serPort, 0.2, 0.2); %drive forward
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
    to = 0; % not touching
    [BumpRight, BumpLeft, ~, ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
    if To == 'R' && BumpRight
        to = 1; %dragging correct wall
    elseif To == 'L' && BumpLeft
        to = 1; %dragging correct wall
    elseif BumpFront
        to = 2; %front hit
    end
%     display(to);

end

function Reset(serPort)
    global DirectionTo;
%     SetDriveWheelsCreate(serPort, 0, 0); %Stop
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
    Coordinates(serPort, 0);
end

% Briefly pause to avoid continuous loop iteration
%  pause(0.1)

% make roomba curve ever greater if it keeps losing wall within small timeframe
%edge cases in code, when it checks position to stop, if we start at a turn
%or place where it has to reset, will we come to a full stop?
%most of the turns/resets will be 90 degrees in cross shape...
