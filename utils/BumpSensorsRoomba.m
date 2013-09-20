function [BumpDir] = BumpSensorsRoomba(serPort)

BumpDir= BumpDirection.None;

try
%Flush Buffer    
N = serPort.BytesAvailable();
while(N~=0) 
fread(serPort,N);
N = serPort.BytesAvailable();
end

warning on
global td


% Send command for packet 7
fwrite(serPort, [142]);  fwrite(serPort,7); 

BmpWheDrps = dec2bin(fread(serPort, 1),8);
BumpRight = bin2dec(BmpWheDrps(end));
BumpLeft = bin2dec(BmpWheDrps(end-1));
WheDropRight = bin2dec(BmpWheDrps(end-2));
WheDropLeft = bin2dec(BmpWheDrps(end-3));
WheDropCaster = bin2dec(BmpWheDrps(end-4));

if BumpRight && BumpLeft
    BumpDir= BumpDirection.Front;
elseif BumpLeft
    BumpDir= BumpDirection.Left;
elseif BumpRight
    BumpDir= BumpDirection.Right;
else
    BumpDir= BumpDirection.None;
end


pause(td)
catch
    disp('WARNING: bump function did not terminate correctly.  Output may be unreliable.')
end