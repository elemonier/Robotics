

function FollowWall(serPort)

    %Variable Setup
    transVel = .2;
    rotVel = 0;
    home = 0;
    firstHome = 1;
    
    %zero values of angle, radius

    hitDir = findWall(serPort, transVel);
    disp('hitDir');
    disp(hitDir);
    
    while(~home)
        alignWallIRSensor(serPort); 
        home = evaluateLocation(serPort, firstHome);
        firstHome = 0;
        moveForward(serPort);
        home = evaluateLocation(serPort, firstHome);
        rotateToWall(serPort);
        home = evaluateLocation(serPort, firstHome);
    end

end


%Functions
function hitDir = findWall(serPort, transVel)
    SetDriveWheelsCreate(serPort, transVel, transVel);
    wallSensors = readWallSensors(serPort);
    disp(wallSensors);
    wallSense = 0;
    
    if(isnan(wallSensors(1)) || isnan(wallSensors(2)) || isnan(wallSensors(3)))
        wallSense = 0;
    end
    
    if( wallSensors(1) || wallSensors(2) || wallSensors(3))
        wallSense = 1;
    end
    
    while ~wallSense
        wallSensors = readWallSensors(serPort);
        disp(wallSensors);
        
        if( wallSensors(1) || wallSensors(2) || wallSensors(3) )
            wallSense = 1;
        else
            wallSense = 0;
        end
        
        pause(0.2);
    end           
    SetDriveWheelsCreate(serPort, 0, 0);
    hitDir = wallSensors;
    
    disp('found initial hit');
end


function wallDirection = wallSensor(serPort)

    [contactDir, wallSense]= WallAndBumpSensors(serPort);
    wallDirection = wallSense;
    

end

function wallSensors = readWallSensors(serPort)

    [BumpRight,BumpLeft,~,~,~,BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
    wallSensors = [BumpRight, BumpLeft, BumpFront];
    
end

function home = evaluateLocation(serPort, firstHit)
    global initAngle initRadius;
    global angl radius tempAngl;
    global angleBool xBool yBool;
    global initX initY;
    global x y;
   
    %if first hit, set initial radius & angle
    if firstHit
        %set global variables
        initAngle = 0;
        initX = 0;
        initY = 0;
        
        angl = 0;
        x = 0;
        y = 0;
        
        xBool = 0;
        yBool = 0;
        angleBool = 0;
        
        %reset distance + angle
        AngleSensorRoomba(serPort);
        DistanceSensorRoomba(serPort);
    else
        
        tempAngl = AngleSensorRoomba(serPort);
        angl = angl + AngleSensorRoomba(serPort);
        radius = DistanceSensorRoomba(serPort);
        
        
        x = x + radius * cos(tempAngl);
        y = y + radius * sin(tempAngl);
        
        disp('angle');
        disp(angl);
        disp('x');
        disp(x);
        disp('y');
        disp(y);
        
        %checking for angle, x, y hit
        
        if ( angl >= -.5 && angl <=  .5)
            angleBool = 1;
            disp('angleBool = 1');
        else
            angleBool = 0;
        end
        
        if ( x >= -.5  && x <=  .5)
            xBool = 1;
            disp('XBool = 1');
        else
            xBool = 0;
        end
        
        if ( y >=  -.5  && y <= .5)
            yBool = 1;
            disp('YBool = 1');
        else
            yBool = 0;
        end
    end
    
    if ( angleBool && xBool && yBool)
        home = 1;
    else
        home = 0;
    end
                  
end

function alignWallIRSensor(serPort)
    disp('align wall ir sensor.');
    wallSensors = readWallSensors(serPort);
    while(wallSensors(1) || wallSensors(2) || wallSensors(3))
        turnAngle(serPort, .05, .5);
        wallSensors = readWallSensors(serPort);
        pause(0.1);
    end
    turnAngle(serPort, .05, .3);
    stop(serPort);
end

function moveForward(serPort)    
    wall = WallSignalRoomba(serPort);
    wallSensors = readWallSensors(serPort);
    disp('wallstrength');
    disp(wall);
    while(wall > 0)
        SetDriveWheelsCreate(serPort, .1, .1);
        disp('wallstrength');
        wall = WallSignalRoomba(serPort);
        disp(wall); 
        
        if(wallSensors(1) || wallSensors(2) || wallSensors(3))
            alignAWallIRSensor(serPort);
        end
        pause(0.1);
    end
    stop(serPort);
end   

function rotateToWall(serPort)
    disp('in rotate To Wall');
    wallSensors = readWallSensors(serPort);
    disp(wallSensors);
    bumped = 0;
    
    if(isnan(wallSensors(1)) || isnan(wallSensors(2)) || isnan(wallSensors(3)))
        bumped = 0;
    elseif(~wallSensors(1) || ~wallSensors(2) || ~wallSensors(3))
        bumped = 0;
    else
        bumped = 1;
    end
    
    while ~bumped
        SetFwdVelAngVelCreate(serPort, .05, -.15);
        wallSensors = readWallSensors(serPort);
        disp(wallSensors);
        
        if(isnan(wallSensors(1)) || isnan(wallSensors(2)) || isnan(wallSensors(3)))
            bumped = 0;
        elseif(~wallSensors(1) && ~wallSensors(2) && ~wallSensors(3))
            bumped = 0;
        else
            bumped = 1;
        end
    
        pause(0.1);
    end
    stop(serPort);
end
    

function stop(serPort)
    SetDriveWheelsCreate(serPort, 0, 0);
    SetFwdVelAngVelCreate(serPort, 0, 0);
end
