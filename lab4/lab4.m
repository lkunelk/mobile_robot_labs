
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
%0.1 m x 0.1 m
    for i=1:900
        
        % read sensor data
        readings = measurement(map, startpos);
        
        
        % update position
        heading = compare_guide(readings);
        [c, move] = distance_guide2(readings, 3);
        startpos = motion([c; 1*-1; heading], startpos);
        
        %check distance
        
        %change heading
        
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
%[val, index] = max(A)
% function heading_angle = simple_guide(sensor_readings)
%     [val, index] = max(sensor_readings);
%     heading_angle = -(69 * ((size(sensor_readings,1) - index) / index )) - (69/2);
% end

%% maintain a parallel direction to the wall
function heading_angle = compare_guide(sensor_reading)
    right_cum_dist = 0;
    left_cum_dist = 0;
    right_valid = 0;
    left_valid = 0;
    right_zeros = 0;
    left_zeros = 0;
    
    for i=1:34 % length(sensor_reading)
        if (sensor_reading(i) ~= 0)
            right_cum_dist = right_cum_dist + sensor_reading(i);
            right_valid = right_valid + 1;
        else
            right_zeros = right_zeros + 1;
        end
    end

    for i=36:69 % length(sensor_reading)
        if (sensor_reading(i) ~= 0)
            left_cum_dist = left_cum_dist + sensor_reading(i);
            left_valid = left_valid + 1;
        else
            left_zeros = left_zeros + 1;
        end
    end

    
    
    K_ang = 0.5;
    if right_valid > right_zeros
        right_val = (right_cum_dist / right_valid);
    else
        right_val = 20;
    end
    
    if left_valid > left_zeros
        left_val = (left_cum_dist / left_valid);
    else
        left_val = 20;
    end
    
    heading_angle = K_ang * (right_val - left_val);

end

%% maintain a distance to the wall using the middle point
function [c, move] = distance_guide1(sensor_readings, distance)
    K_dist = -0.1;
    move = 1;
    MAX_VAL = distance;
    if (sensor_readings(int16(length(sensor_readings)/2)) < 10) ... 
            || (sensor_readings(int16(length(sensor_readings)/2)) > 0.3)
        mid_read = sensor_readings(int16(length(sensor_readings)/2));
    else
        mid_read = 15;
        move = 0;
    end
    c = K_dist * (distance - mid_read);
    
%     [val, index] = max(sensor_readings);
%     heading_angle = -(69 * ((size(sensor_readings,1) - index) / index )) - (69/2);
end

function [c, move] = distance_guide2(sensor_readings, distance)
    K_dist = -0.5;
    move = 1;
    buff_reading = sensor_readings;
    for i=1:length(buff_reading)
        if (buff_reading(i) == 0)
            buff_reading(i) = 20; % change to non-zero
        end
    end
    [val, index] = min(buff_reading);
    MAX_VAL = distance;
    c = K_dist * (distance - val);
    
%     [val, index] = max(sensor_readings);
%     heading_angle = -(69 * ((size(sensor_readings,1) - index) / index )) - (69/2);
end


