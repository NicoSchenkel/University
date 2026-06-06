function [finalTime, trajTimes] = calculateTrajectoryParams(taskInit, taskFinal)

timeStep = 0.2; % seconds
toolSpeed = 0.1; % m/s

distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));

% Initial & final time
initTime = 0;
finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
%timeInterval = [trajTimes(1); trajTimes(end)];


end


