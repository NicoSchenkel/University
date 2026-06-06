function currentRobotJConfig = drawTrajectory(trajTimes, rq, robot, currentRobotJConfig, endEffector, axAnim, TWB)
% 'Parent', axAnim zwingt MATLAB, genau in das neue Fenster (figure2) zu zeichnen
show(robot, currentRobotJConfig, 'Parent', axAnim, 'PreservePlot', true, 'Frames', 'on');
hold(axAnim, 'on');

for i=1:length(trajTimes)
    % Joint values for tNow
    qCurrent = rq(:,i);
    for n = 1:length(qCurrent)
        currentRobotJConfig(n).JointPosition = qCurrent(n);
    end
    
    % Determining Pose 
        %poseNow = getTransform(robot, currentRobotJConfig, endEffector);  % ← toolOffset
                [~, ~, T] = ForwardKinematics(TWB, rad2deg(qCurrent));
             poseNow = T{8};
    % Auch hier weisen wir explizit axAnim zu, damit es im neuen Fenster bleibt
    show(robot, currentRobotJConfig, 'Parent', axAnim, 'PreservePlot', false, 'Frames', 'off');
    
    % Der plot3-Befehl muss ebenfalls wissen, dass er in axAnim zeichnen soll
    jointSpaceMarker = plot3(axAnim, poseNow(1,4), poseNow(2,4), poseNow(3,4), 'r.', 'MarkerSize', 20);
    drawnow;
    jacobian = geometricJacobian(robot,currentRobotJConfig,endEffector);


end

% Beschriftungen auf den korrekten Achsen setzen
%legend(axAnim, jointSpaceMarker, {'Defined in Joint-Space'}, 'Location', 'best');
%title(axAnim, 'Manipulator Trajectories');
jacobian - jacobiMatrix(T, TWB)




end