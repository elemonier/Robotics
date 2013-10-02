function CircumAmbulateWall(serPort)


%distance + angle taking at a specific point - make dependent on wall
%sensor
%make simulation dependent on wall sensor
%realign - take anglesensor, not distance sensor
%when looses bump - take distance

    global Angle NetDistance To DirectionTo TotalX TotalY grossDistanceTraveled;
    grossDistanceTraveled = 0;
    
    %get initial start angle + distance away from wall
    getInitialCoordinates(serPort, 0);

    %Move forward until any bump
    SetDriveWheelsCreate(serPort, 0.3, 0.3); 
    while true
        [BumpRight, BumpLeft, ~, ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        if(~isnan(BumpRight) || ~isnan(BumpLeft) || ~isnan(BumpFront))
            if (BumpRight || BumpLeft || BumpFront)
                
                %Stop
                SetFwdVelRadiusRoomba(serPort, 0, 2); 
                if(BumpRight)
                    To = 'R';
                    DirectionTo = -1;
                else
                    To = 'L';
                    DirectionTo = 1;
                end
                
                %DistanceSensorRoomba(serPort); reset the distance sensor
                %commented out because we are now resetting distance +
                %anglen sensors in getInitialCoordinates
                
                getInitialCoordinates(serPort, 1);
                NetDistance = inf;
                TotalX = 0.0;
                TotalY = 0.0;
                break
            end
        end
        pause(0.1)
    end
    display('Variables Set');
    
    %indicates dragging or hit
    to = BumpingTo(serPort);
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
    getInitialCoordinates(serPort, 2); 
    %AngleSensorRoomba(serPort); %reset the angle sensor - commented out,
    %done in get initialCoordinates
    pause(.2)
    Angle = 0.0;
    
    Drag(serPort);
    %test(serPort);
    
    % Full Stop
    SetDriveWheelsCreate(serPort, 0, 0); 
    SetFwdVelAngVelCreate(serPort, 0, 0);
    display('FINISHED');
    %return robot to 'true' starting point
    getInitialCoordinates(serPort, 3);
end

function test(serPort)
    global DirectionTo;

AngleSensorRoomba(serPort); %reset
display(AngleSensorRoomba(serPort)*180/pi);
turnAngle(serPort, 0.2, DirectionTo*70)
display(AngleSensorRoomba(serPort)*180/pi);

end

function Coordinates(serPort, deltaDist)
    global NetDistance Angle TotalX TotalY grossDistanceTraveled;
    
    %display(deltaDist);
    deltaAngle = AngleSensorRoomba(serPort)*180/pi;
    
    grossDistanceTraveled = grossDistanceTraveled + abs(deltaDist)*100;
   
    if(deltaDist > 0)

        TotalX = TotalX + (cos(Angle*pi/180)*deltaDist)*100;
        TotalY = TotalY + (sin(Angle*pi/180)*deltaDist)*100;  
        Angle = mod(Angle + deltaAngle, 360);
        absAng = abs(mod(Angle, 360));
        
        %widened absAngle range from 10 to 30
        if(grossDistanceTraveled > 200)
            if (absAng > 330 || absAng < 30)
                if(TotalX < 100 && TotalX > -100) % or close enough
                    if(TotalY < 100 && TotalY > -100)
                        SetDriveWheelsCreate(serPort, 0, 0); % Full Stop
                        NetDistance = 0;
                    end
                end
            end
        end

    end
    
    display('CURRENT COORDS:');
    display('grossDistanceTraveled, Angle, TotalX, TotalY');
    display(grossDistanceTraveled);
    display(Angle);
    display(TotalX);
    display(TotalY);

end

function Drag(serPort)
    global DirectionTo NetDistance;
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
                to = BumpingTo(serPort);
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
%     Do we want to pass zero distance here? - Em: yes.
    Coordinates(serPort, 0);
end

function getInitialCoordinates(serPort, initHit)
    global startDistance startAngle;
    if initHit == 0
        DistanceSensorRoomba(serPort);
        AngleSensorRoomba(serPort);
    elseif initHit == 1
        startDistance = DistanceSensorRoomba(serPort);
    elseif initHit == 2
        startAngle = AngleSensorRoomba(serPort);
        disp('INIT COORDS');
        disp('dist:');
        disp(startDistance);
        disp('angle:');
        disp(startAngle);
    elseif initHit == 3
        disp('moving angle dist');
        disp(360-startAngle);
        disp(abs(startDistance));
        turnAngle(serPort, .1, 360-startAngle);
        travelDist(serPort, .1, abs(startDistance)); 
        SetFwdVelAngVelCreate(serPort, 0, 0);
    end
    

end


% make roomba curve ever greater if it keeps losing wall within small timeframe
%edge cases in code, when it checks position to stop, if we start at a turn
%or place where it has to reset, will we come to a full stop?
%most of the turns/resets will be 90 degrees in cross shape...
