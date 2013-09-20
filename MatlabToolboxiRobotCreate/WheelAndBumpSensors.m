function [BumpDir, WallSignal] = WallAndBumpSensors(serPort)

BumpDir= BumpSensorsRoomba(serPort);
WallSignal= WallSignalRoomba(serPort);

end