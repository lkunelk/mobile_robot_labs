
function lab4()
    % Aerial racing
    I = imread('Racecourse.png');
    map = im2bw(I, 0.4); % Convert to 0-1 image
    map = flipud(1-map)'; % Convert to 0 free, 1 occupied and flip.
    [M,N]= size(map); % Map size

    % Robot start position
    dxy = 0.1;
    startpos = dxy*[350; 250; 30];
    checkpoints = dxy*[440 620; 440 665];

    for i=1:100
        
        % read sensor data
        readings = measurement(map, startpos);
        
        
        % update position
        startpos = motion([1; 0; 0], startpos);
        
        % Plotting
        figure(1); clf; hold on;
        colormap('gray');
        imagesc(1-map');
        plot(startpos(1)/dxy, startpos(2)/dxy, 'ro', 'MarkerSize',10, 'LineWidth', 3);
        plot(checkpoints(:,1)/dxy, checkpoints(:,2)/dxy, 'g-x', 'MarkerSize',10, 'LineWidth', 3 );
        xlabel('North (decimeters)')
        ylabel('East (decimeters)')
        axis equal
    end
end

function [update] = motion(vel,pose)
    if(vel(1)<20 && vel(2)<20)
        heading = pose(3);
        C = [cos(heading) -sin(heading) 0; sin(heading) cos(heading) 0; 0 0 1];
        update = pose+1/5*C*vel;
        update(1:2) = update(1:2) + randn(1)*0.05;
        update(3) = update(3) + randn(1)*0.02;
    end
end

function [sensor_readings] = measurement(map,pose)
    dxy = 0.1;
    sensor_readings = zeros(69,1);
    for i = -34:34
        heading = pose(3)+i/57.3;
        C = [cos(heading) -sin(heading); sin(heading) cos(heading)];
        for j = 0.3:0.1:10
            measurement = (C*[j;0]+pose(1:2))./dxy;
            if(map(round(measurement(1)),round(measurement(2))) == 1)
                sensor_readings(i+35) = j+ randn(1)*0.05;
                
                % plot data
                plot(measurement(1), measurement(2), 'b-x', 'MarkerSize',5, 'LineWidth', 3 );
                
                break;
            end
        end
    end
end



