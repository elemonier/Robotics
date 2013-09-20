function [WallSignal, BumpDir] = WallAndBumpSensors(serPort)


BumpDir= BumpSensorsRoomba(serPort);
WallSignal= WallSignalRoomba(serPort);

end