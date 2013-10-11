create big mama array filled with -1
set currentX, currentY at center
 
head towards infinity using hw2 algorithm
check hits
while we have not hit/wall-sensored something
    distance = getDist;
    while distance < roombaLength
        distance = distance + getDist
    check hits/wall sensor
    if hit/wall sensor
        fill map in at currentX, currentY with 1
    else
        fill map in at currentX, currentY with 0
    end
    currentX = currentX + distance / roombaLength
    
    
hit something
while we have not circumnavigated an object
    update odometry, each time we move distance > roombaLen
    fill in map with currentX, Y depending on hit
    likely hits/sensoring will be on right of robot
    fill map in currentX + 1, currentY.
    
mark the circumnavigated object as outermost boundary (move big mama array into smaller one?)
while array has -1s inside and max-time-bound not expired
    move towards areas that we have not yet explored
    continue calculating odometry to get X, Y
    each time we move distance > roombaLen
    fill in map at x, y

