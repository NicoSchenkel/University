function drawFrame(T_frames)
    scale = 0.1;
    colors = {'m', 'y', 'c'};
    
    for i = 1:length(T_frames)
        Frame = T_frames{i};
        origin = Frame(1:3, 4);
        R = Frame(1:3, 1:3);
        

        for ax = 1:3
            endpoint = origin + scale * R(:, ax);
            plot3([origin(1), endpoint(1)], ...
                  [origin(2), endpoint(2)], ...
                  [origin(3), endpoint(3)], ...
                  '-', 'Color', colors{ax}, 'LineWidth', 2);
        end
        text(origin(1), origin(2), origin(3), sprintf(' F_%d', i), ...
             'Color', 'k', 'FontSize', 7);
    end
end