function  drawTrajectory(trajTimes, rq, currentRobotJConfig)

show(robot,currentRobotJConfig,'PreservePlot',true,'Frames','on');
hold on;
for i=1:length(trajTimes)
    % Current time 
    tNow= trajTimes(i);
    % Joint values for tNow
    qCurrent = rq(:,i)
    for n = 1:length(qCurrent)
        currentRobotJConfig(n).JointPosition = qCurrent(n);
    end
    % Determining Pose 
    poseNow = getTransform(robot,currentRobotJConfig,endEffector);
    show(robot,currentRobotJConfig,'PreservePlot',false,'Frames','off');
    jointSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'r.','MarkerSize',20);
    drawnow;
end

% Add a legend and title
legend([taskSpaceMarker jointSpaceMarker], { 'Defined in Joint-Space'});
title('Manipulator Trajectories')
end