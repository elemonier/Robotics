 function [wall_sensor] = AnalogWallSensorReadRoomba(serPort)
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