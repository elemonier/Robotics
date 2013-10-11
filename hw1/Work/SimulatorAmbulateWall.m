function SimulatorAmbulateWall_check(serPort)

%change it to only going around one way
%distance + angle taking at a specific point - make dependent on wall
%sensor
%make simulation dependent on wall sensor
%realign - take anglesensor, not distance sensor
%when looses bump - take distance

    global NetDistance To DirectionTo grossDistanceTraveled;
    global setInitAng;
    setInitAng = 2;
    grossDistanceTraveled = 0;
    %get zero point start angle + distance away from wall
    getInitialCoordinates(serPort, 0);
    disp('ZERO POINT OF INITIAL COORDS');
    %Initial Motion: Move forward until any bump
    SetDriveWheelsCreate(serPort, 0.3, 0.3);
    while true
        [BumpRight, BumpLeft, ~, ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        if(~isnan(BumpRight) || ~isnan(BumpLeft) || ~isnan(BumpFront))
            if (BumpRight || BumpLeft || BumpFront)
                
                %Stop
                SetFwdVelRadiusRoomba(serPort, 0, 2); 
                
                %assume everything bumps right so as to follow wall +
                %allows for optional implementation of wallSensor
                DirectionTo = -1;
                To = 'R';
             
                %Resets Angle + Distance Sensors, gets the change from 
                %REAL starting point (ie off of wall)
                getInitialCoordinates(serPort, 1);
                NetDistance = inf;
                break
            end
        end
        pause(0.1);
    end
    
    display('All Variables Set');
    
    %starts loop now to drive + turn
    Drag(serPort);
    
    % Full Stop
    SetDriveWheelsCreate(serPort, 0, 0); 
    SetFwdVelAngVelCreate(serPort, 0, 0);
    display('FINISHED');
    %return robot to 'true' starting point
    getInitialCoordinates(serPort, 3);
end

function test(serPort)
    global DirectionTo;

%AngleSensorRoomba(serPort); %reset
display(AngleSensorRoomba(serPort)*180/pi);
turnAngle(serPort, 0.2, DirectionTo*70)
display(AngleSensorRoomba(serPort)*180/pi);

end

%Coordinates evaluates current position of robot relative to location of
%initial hit
%arguments:
%   check = 'dist' when only reevaluate distance
%   check = 'angle' when only reevaluate angle
%   check = 'both' when reevaluate angle + distance
function Coordinates(serPort, d, check)
    global NetDistance Angle TotalX TotalY grossDistanceTraveled;
    
    deltaDist = d;
    %deltaDist =abs(DistanceSensorRoomba(serPort));
    deltaAngle = AngleSensorRoomba(serPort)*180/pi;
    grossDistanceTraveled = grossDistanceTraveled + abs(deltaDist);
    
    %make ~90 degree asumption to account for a ~90 deg angle
    disp('DELTA ANGLE');
    disp(deltaAngle);
    disp('ANGLE');
    disp(Angle);
    if(83 < deltaAngle && 97 > deltaAngle)
        deltaAngle = 90;
    elseif (-97 < deltaAngle && -83 > deltaAngle)
        deltaAngle = -90;
    end
    
    
    %only adjust distance
    if strcmp(check, 'dist')
        TotalX = TotalX + (cos(Angle*pi/180)*deltaDist)*100;
        TotalY = TotalY + (sin(Angle*pi/180)*deltaDist)*100; 
    %only adjust angle
    elseif strcmp(check, 'angle')
        if(abs(Angle+deltaAngle) > 360)
            if(Angle < 0)
                Angle = -1 * mod(abs(Angle + deltaAngle), 360);
            elseif (Angle > 0)
                Angle = mod(Angle + deltaAngle, 360);
            end
        else
            Angle = Angle + deltaAngle;
        end
    %adjust both distance + angle
    elseif strcmp(check, 'both')
        if(abs(Angle+deltaAngle > 360))
            Angle = mod(Angle + deltaAngle, 360);
        else
            Angle = Angle + deltaAngle;
        end
        TotalX = TotalX + (cos(Angle*pi/180)*deltaDist)*100;
        TotalY = TotalY + (sin(Angle*pi/180)*deltaDist)*100;  
    end
    %Checking if current location = start location
    if(grossDistanceTraveled > 2)
    	if(abs(Angle) > 330 || abs(Angle) < 35)
        	if(TotalX < 60 && TotalX > -60) % or close enough
            	if(TotalY < 100 && TotalY > -100)
                	SetDriveWheelsCreate(serPort, 0, 0); %Full Stop
                    NetDistance = 0; %Net distance = 0 means exit drag loop
                end
            end
        end
    end
    
    display('CURRENT COORDS: Angle, TotalX, TotalY');
    display(Angle);
    display(TotalX);
    display(TotalY);

end


function Drag(serPort)
    global DirectionTo NetDistance;
    display('DRAG');
    
    %reset angle
    %AngleSensorRoomba(serPort);
    %loop for robot to continue
    while NetDistance ~= 0
        to = BumpingTo(serPort);
        disp('TO ITEM');
        disp(to);
        dist = DistanceSensorRoomba(serPort);
        if to > 0
            d = DistanceSensorRoomba(serPort);
            dist = dist + d;
            disp('DISTANCE');
            disp(dist);
            %Coordinates(serPort, dist, 'dist'); %angle should be zero if straight wall
            %pause(0.1);
            %do we need this dist check?? If not- lets take it out if we
            %can. That will streamline coordinates calculation.
            if to == 2 || dist == 0 %front hit
                disp('FRONT HIT.');
                SetDriveWheelsCreate(serPort, 0.1, 0.1); %stop
                %AngleSensorRoomba(serPort);
                turnAngle(serPort, 0.2, -DirectionTo*25);
                pause(0.1);
                Reset(serPort);
                %fix direction
            end
            
            if to == 1 && dist == 0
                disp('caught in angle');
                Reset(serPort);
            end
            %add dist here?
            %dist = dist + DistanceSensorRoomba(serPort);
            Coordinates(serPort, dist, 'dist');
            SetDriveWheelsCreate(serPort, 0.2, 0.2); %drive forward
            pause(0.1);
        else %lost the wall
            while to == 0 %spiral until hit
                SetFwdVelRadiusRoomba(serPort, 0.1, DirectionTo*pi/24); %complete guess
                %its possible if it spirals too much to hit the wrong
                %bumper here and cause problems with to and away logic
                to = BumpingTo(serPort);
                pause(0.1);
            end
            print('LOST WALL');
            %AngleSensorRoomba(serPort);
            Reset(serPort);
        end
        pause(0.15);
    end

end

%To returns direction of hit
%   = 0 - not touching
%   = 1 - dragging
%   = 2 - front hit
function to = BumpingTo(serPort)
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

end

%Navigate around angle
%Assime distance traveled is 0, etc.
function Reset(serPort)
    global DirectionTo setInitAng TotalX TotalY Angle;
    
    %SetDriveWheelsCreate(serPort, 0, 0); %Stop
    to = BumpingTo(serPort);
    while to > 0
        turnAngle(serPort, 0.2, -DirectionTo*10);
        to= BumpingTo(serPort);
        pause(0.1);
    end
    while to == 0
        turnAngle(serPort, 0.2, DirectionTo*2);
        to= BumpingTo(serPort);
        pause(0.1);
    end
    %pass zero distance
    if(setInitAng == -1)
        disp('RESETTING ANGLE');
        Coordinates(serPort, 0, 'angle');
    else
        %disp('FIRST RESET');
        getInitialCoordinates(serPort, setInitAng);
        AngleSensorRoomba(serPort);
        DistanceSensorRoomba(serPort);
        TotalX = 0.0;
        TotalY = 0.0;
        Angle = 0.0;
    end
    setInitAng = -1;
end

function getInitialCoordinates(serPort, initHit)
    global startDistance startAngle;
    %Robot just started moving, set everything to 0
    
    %you are at initial starting point
    if initHit == 0
        DistanceSensorRoomba(serPort);
        AngleSensorRoomba(serPort);
    
        %you just hit wall - evaluate distance traveled
    elseif initHit == 1
        startDistance = DistanceSensorRoomba(serPort);
        disp('INIT COORDS');
        disp('dist:');
        disp(startDistance);
    %you are aligned to a place where you will start 'drag'; evaluate 
    elseif initHit == 2
        startAngle = AngleSensorRoomba(serPort)*180/pi;
        disp('INIT COORDS');
        disp('angle:');
        disp(startAngle);
    
    elseif initHit == 3
        disp('moving angle dist');
        disp(abs(startDistance));
        %travelDist(serPort, .1, -.1);
        turnAngle(serPort, 0.1, startAngle);
        pause(0.1);
        disp('done turning.');
        travelDist(serPort, .1, abs(startDistance)); 
        pause(0.1);
        disp('done traveling.');
        SetFwdVelAngVelCreate(serPort, 0, 0);
    end
    

end

% make roomba curve ever greater if it keeps losing wall within small timeframe
%edge cases in code, when it checks position to stop, if we start at a turn
%or place where it has to reset, will we come to a full stop?
%most of the turns/resets will be 90 degrees in cross shape...