function WallSensorKillerDestroyer(serPort)%TODO -- change finalRad to desired output

    global Alpha NetDistance keepGoing td;
    td = .05;
    
    SetDriveWheelsCreate(serPort, 0.2, 0.2);
    while true
        Bump = BumpSensorsRoomba(serPort);
        if Bump ~= BumpDirection.None
            SetFwdVelRadiusRoomba(serPort, 0, 2); %Stop
            if Bump == BumpDirection.Left
                turnAngle(serPort, 0.1, 40);
            end
            DistanceSensorRoomba(serPort); %reset the distance sensor
            NetDistance = 1;
            keepGoing = 1;
            break
        end
        
        pause(0.5)
    end
    display('Made Contact');

    AngleSensorRoomba(serPort); %reset the angle sensor
    Alpha = 0;
    
     Survey(serPort)    
     SenseSpeedAverage(serPort);

    SetFwdVelRadiusRoomba(serPort, 0, 2); % Full Stop
    display('FINISHED');
end

function SenseSpeed(serPort)
global keepGoing;

    if(isnan(keepGoing))
        keepGoing = 1;
    end

    while keepGoing

        [wallStrength, Bump] = WallAndBumpSensors(serPort);
        display(wallStrength)
        SetDriveWheelsCreate(serPort, 0, 0); 
        pause(1)
        
        if(Bump ~= BumpDirection.None)
            SetDriveWheelsCreate(serPort, 0, 0);
            turnAngle(serPort, 0.1, 40);
            display('Reseting')
            Reset(serPort, Bump)
        end
        if(wallStrength > 39 && wallStrength < 130)
            SetDriveWheelsCreate(serPort, 0.3, 0.3);
            display('onward');
                    pause(0.1);

        elseif (wallStrength < 40 && wallStrength > 15)
            SetDriveWheelsCreate(serPort, 0.2, 0.3);
            display('veer right');
                    pause(0.1);

        elseif (wallStrength > 129)
            SetDriveWheelsCreate(serPort, 0.3, 0.2);
            display('veer left');
                    pause(0.1);

        elseif (wallStrength < 16)
            SetFwdVelRadiusRoomba(serPort, 0.05, -0.1);
            display('lost wall sensor');
            
        end
        
        pause(0.1);
    end

end

function SenseSpeedAverage(serPort)
global keepGoing;

    if(isnan(keepGoing))
        keepGoing = 1;
    end
    avg_count = 1;
    
    while keepGoing
       
        Bump = BumpSensorsRoomba(serPort);
        wallStrength = 0;
        if(Bump ~= BumpDirection.None)
            display(Bump)
            SetDriveWheelsCreate(serPort, 0, 0);
            turnAngle(serPort, 0.1, 30);
            display('Survey')
            Survey(serPort)
            wallStrength = WallSignalRoomba(serPort);        
        else
            avg = 0;
            i = 0;
            while i < avg_count
                wallStrength = WallSignalRoomba(serPort);        
                if(isnan(wallStrength))
                    i = i - 1;
                    continue;
                end
                avg = avg + wallStrength;
                i = i + 1;
            end
            wallStrength = avg /i;
        end
        display(wallStrength)
       
        SetDriveWheelsCreate(serPort, 0, 0);
        pause(1)
       
        if(wallStrength > 15 ) %&& wallStrength < 160)
            SetDriveWheelsCreate(serPort, 0.15, 0.15);    
            display('onward');
%         elseif (wallStrength < 40 && wallStrength > 15)
%             SetDriveWheelsCreate(serPort, 0.1, 0.15);
%             display('veer right');
%         elseif (wallStrength > 159)
%             SetDriveWheelsCreate(serPort, 0.15, 0.1);
%             display('veer left');
        elseif (wallStrength < 16)
            SetFwdVelRadiusRoomba(serPort, 0.1, -0.1);
            display('lost wall sensor');
        end
        
        pause(0.1);
    end

end

function Survey(serPort)
    prevStrength = 0;
    w_speed = 0.05;
    turn_count = 0;
    while turn_count < 30
        SetDriveWheelsCreate(serPort, w_speed, -w_speed);
        SetDriveWheelsCreate(serPort, 0, 0);
        wallStrength = WallSignalRoomba(serPort);
        display(wallStrength)
        pause(0.2)
        if(prevStrength >= wallStrength && prevStrength > 10)
            break;
        end
        if (wallStrength > prevStrength + 4)
            prevStrength = wallStrength;
        end
        turn_count = turn_count + 1;
        display(turn_count)
    end
    if(turn_count == 30)
        SetFwdVelRadiusRoomba(serPort, 0.1, -0.1);
        display('lost wall sensor');        
    else
        SetDriveWheelsCreate(serPort, -w_speed, w_speed);
        SetDriveWheelsCreate(serPort, 0, 0);    
    end
   
end

function Reset(serPort)
    [wallStrength, Bump] = WallAndBumpSensors(serPort);

    while (wallStrength < 20 && Bump ~= BumpDirection.Front)
        display('turning more');
        SetFwdVelRadiusRoomba(serPort, 0.1, 0.1);
        pause(0.2)
        [wallStrength, Bump] = WallAndBumpSensors(serPort);
        display(wallStrength)
    end

    display('Done with Reset')
end

% Briefly pause to avoid continuous loop iteration
%  pause(0.1)
%make it survey its surroundings and use a percentage
% make roomba curve ever greater if it keeps losing wall within small timeframe
%edge cases in code, when it checks position to stop, if we start at a turn
%or place where it has to reset, will we come to a full stop?
%most of the turns/resets will be 90 degrees in cross shape...
