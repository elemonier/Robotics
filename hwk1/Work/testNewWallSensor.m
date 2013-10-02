%function which tests whether AnalogWallSensorReadRoomba is working
%included actual function for convenience.
%gives fwrite error. Trying to figure it out but thought I would push
%to see if you guys could help troubleshoot!
%-em

%commenting out lines 17, 18 makes it work.


function testNewWallSensor(serPort)
    SetFwdVelAngVelCreate(serPort, .2, 0);
    [BumpRight,BumpLeft,~,~,~,BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
    bumped = BumpRight || BumpLeft || BumpFront;
    
    while ~bumped
        SetFwdVelAngVelCreate(serPort, .2, 0);
        %wall_sensor = AnalogWallSensorReadRoomba(serPort);
        %disp(wall_sensor);
        [BumpRight,BumpLeft,~,~,~,BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        bumped = BumpRight || BumpLeft || BumpFront; 
        pause(0.1);
    end
    
    %[BumpRight,BumpLeft,~,~,~,BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
    %bumped = BumpRight || BumpLeft || BumpFront; 
    %
    %while bumped
    %    turnAngle(serPort, .1, pi/10);
    %    [BumpRight,BumpLeft,~,~,~,BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
    %    bumped = BumpLeft || BumpFront; 
    %end 
    

end


function [wall_sensor]   = AnalogWallSensorReadRoomba(serPort)
    try
        set(serPort,'timeout',0.01);
        %Flush buffer
        N = serPort.BytesAvailable();
        while(N~=0) 
            fread(serPort,N);
            N = serPort.BytesAvailable();
        end
    catch
    end
    %% Get (142) Wall Reading(8) data fields
    fwrite(serPort, [142 27]);
    wall_sensor = fread(serPort, 1, 'uint16');
end