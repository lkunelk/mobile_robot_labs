
function lab4()
    % Aerial racing
    I = imread('Racecourse.png');
    true_map = im2bw(I, 0.4); % Convert to 0-1 image
    true_map = flipud(1-true_map)'; % Convert to 0 free, 1 occupied and flip.
    [M,N]= size(true_map); % Map size

    map = ones(M, N) * 0.5;
    
    % Robot start position
    dxy = 0.1;
    startpos = dxy*[350; 250; 30];
    %startpos = dxy*[750; 450; 180];
    checkpoints = dxy*[440 620; 440 665];
    [readings, map] = measurement(true_map, map, startpos);
    heading = compare_guide(readings);
    %array = zeros(1,20);
    %coeff = ones(1,20);
    coeff = 1:30;
    array = ones(1,30) * heading;
    coeff = normalize(coeff,'range');
    %0.1 m x 0.1 m
    
    steps = 2000;
    trajectory = zeros(steps, 2);
    cur_pos = startpos;
    
    for i=1:steps
        
        i
        
        % read sensor data
        [readings, map] = measurement(true_map, map, cur_pos);
        
        
        % update position
        heading = compare_guide(readings);
        
        std_array = std(array);
        heading = (0.5 - std_array) * heading;
        array = array(2:end);
        array(end+1) = heading;
        % array = num2cell(cat(2, array{2:end}, heading), 1, ones(size(array)));
        heading_new = coeff * array';
        [c, move] = distance_guide2(readings, 1.5);
        if abs(c) > 5
            y_move = 0;
        else
            y_move = (((95 - (5*heading_new + 5*c))))^2*0.00001;
        end
        cur_pos = motion([c; 5*y_move*-1; heading_new], cur_pos);
        
        %check distance
        
        %change heading
        
        % save variables
        trajectory(i,:) = cur_pos(1:2);
        
        if mod(i,250) == 0
            % Plotting
            figureX = figure;
            clf;
            hold on;
            colormap('gray');
            imagesc(1-map');
            plot(startpos(1)/dxy, startpos(2)/dxy, 'ro', 'MarkerSize',5, 'LineWidth', 2);
            plot(checkpoints(:,1)/dxy, checkpoints(:,2)/dxy, 'g-x', 'MarkerSize',10, 'LineWidth', 3 );
            for j=1:2:steps
                plot(trajectory(j,1)/dxy, trajectory(j,2)/dxy, 'bo', 'MarkerSize', 1, 'LineWidth', 1);
            end
            xlabel('North (decimeters)')
            ylabel('East (decimeters)')
            axis equal
            file_name = 'picture' + string(i/250) + '.png';
            saveas(figureX,file_name);
        end
        
        if norm(cur_pos(1:2) - startpos(1:2)) < 5 && i > 1000
            break
        end
    end
   
    % Plotting
    figureX = figure;
    clf;
    hold on;
    colormap('gray');
    imagesc(1-map');
    plot(startpos(1)/dxy, startpos(2)/dxy, 'ro', 'MarkerSize',5, 'LineWidth', 2);
    plot(checkpoints(:,1)/dxy, checkpoints(:,2)/dxy, 'g-x', 'MarkerSize',10, 'LineWidth', 3 );
    for j=1:2:steps
        plot(trajectory(j,1)/dxy, trajectory(j,2)/dxy, 'bo', 'MarkerSize', 1, 'LineWidth', 1);
    end
    xlabel('North (decimeters)')
    ylabel('East (decimeters)')
    axis equal
    saveas(figureX,'final.png')
    close all;
end


function [update] = motion(vel,pose)
    if(vel(1)<20 && vel(2)<20)
        heading = pose(3);
        C = [cos(heading) -sin(heading) 0; sin(heading) cos(heading) 0; 0 0 1];
        update = pose + 1/2 * C * vel;
        update(1:2) = update(1:2) + randn(1)*0.05;
        update(3) = update(3) + randn(1)*0.02;
    end
end

function [sensor_readings, map] = measurement(true_map, map, pose)
    dxy = 0.1;
    sensor_readings = zeros(69,1);
    for i = -34:34
        heading = pose(3)+i/57.3;
        C = [cos(heading) -sin(heading); sin(heading) cos(heading)];
        for j = 0.3:0.1:10
            measurement = (C*[j;0]+pose(1:2))./dxy;
            x = round(measurement(1));
            y = round(measurement(2));
            if(true_map(x, y) == 1)
                map(x, y) = 1;
                sensor_readings(i+35) = j+ randn(1)*0.05;
                
                % plot data
                % plot(measurement(1), measurement(2), 'g-x', 'MarkerSize',3, 'LineWidth', 1 );
                
                break;
            else
                map(x, y) = 0;
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
    MAX_VAL = 5;
    
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

    
    
    K_ang = 0.1;
    if right_valid > right_zeros
        right_val = (right_cum_dist / right_valid);
    else
        right_val = 10;
    end
    
    if left_valid > left_zeros
        left_val = (left_cum_dist / left_valid);
    else
        left_val = 10;
    end
    
    heading_angle = K_ang * (right_val - left_val);

    if abs(heading_angle) > MAX_VAL
        heading_angle = abs(MAX_VAL) * sign(heading_angle);
    end
end
%% 




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


