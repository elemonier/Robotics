function WallSensorTest(serPort)

SetDriveWheelsCreate(serPort,-0.05,0.05)

while(true)
    disp(WallAndBumpSensors(serPort))
end
end