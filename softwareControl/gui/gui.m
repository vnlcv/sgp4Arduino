% Real-Time Satellite Tracking Sky Plot in MATLAB

% Clear workspace
clear;
clc;

% Serial port configuration
port = "COM7"; % Change to your Arduino COM port
baudRate = 38400;

% Create the serial object
s = serialport(port, baudRate);
configureTerminator(s, "LF");
flush(s);

% Prepare the figure for plotting
figure;
hold on;
title('Real-Time Satellite Tracking Sky Plot');

% Set up the polar coordinates for the sky plot
theta = linspace(0, 2*pi, 361); % 0 to 360 degrees in radians
maxRadius = 90; % Maximum radius (corresponding to 0 degrees elevation)

% Draw the sky plot circle
plot(maxRadius * cos(theta), maxRadius * sin(theta), 'k'); % Outer circle for the sky plot
axis equal; % Keep the aspect ratio of the plot equal
xlim([-maxRadius maxRadius]); % Set x limits
ylim([-maxRadius maxRadius]); % Set y limits
grid on;

% Define elevation angles and labels
elevations = [0, 20, 40, 60, 80]; % Elevation from 0 to 80 degrees
elevationLabels = {'0', '20', '40', '60', '80'}; % Labels for elevations

% Draw elevation circles
for i = 1:length(elevations)
    elevation = elevations(i);
    % Calculate the radius for the circle at the given elevation
    % 0 degrees -> maxRadius, 80 degrees -> maxRadius/5 (innermost circle)
    radius = maxRadius * (1 - elevation / 90); % Adjusted for 90 degrees at center
    % Draw the circle for the elevation level
    plot(radius * cos(theta), radius * sin(theta), 'k--', 'LineWidth', 0.5);
    
    % Add elevation label (placed at the edge of the circle)
    text(radius * 1.1, 0, elevationLabels{i}, ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontWeight', 'bold');
end

% Define azimuth labels and angles (in degrees)
azimuthLabels = {'N', '30', '60', 'E', '120', '150', 'S', '210', '240', 'W', '300', '330'};
azimuthAngles = [0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330]; % Azimuth angles in degrees

% Plot azimuth labels
for i = 1:length(azimuthAngles)
    angleRad = deg2rad(azimuthAngles(i));
    % Position the labels at a distance of 1.1 * maxRadius for visibility
    text(1.1 * maxRadius * sin(angleRad), 1.1 * maxRadius * cos(angleRad), azimuthLabels{i}, ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontWeight', 'bold');
end

% Initialize data storage for azimuth and elevation
azimuths = [];
elevationsData = [];
maxPoints = 100; % Limit to the last 100 data points

% Main loop to read data from serial and update plot
while ishandle(gca) % Loop while the axes handle is valid
    % Check if data is available from serial
    if s.NumBytesAvailable > 0
        dataLine = readline(s);
        data = split(dataLine, ',');
        
        % Ensure valid data structure and length
        if numel(data) >= 2
            try
                % Convert azimuth and elevation to numbers
                azimuth = str2double(data{1});
                elevation = str2double(data{2});
                
                % Only use valid azimuth and elevation values
                if ~isnan(azimuth) && ~isnan(elevation) && elevation >= 0 && elevation <= 90
                    % Print the azimuth and elevation in the command window
                    fprintf('Azimuth: %.2f degrees, Elevation: %.2f degrees\n', azimuth, elevation);

                    % Append new data
                    azimuths = [azimuths; azimuth];
                    elevationsData = [elevationsData; elevation];
                    
                    % Limit data to maxPoints
                    if length(azimuths) > maxPoints
                        azimuths = azimuths(end-maxPoints+1:end);
                        elevationsData = elevationsData(end-maxPoints+1:end);
                    end
                    
                    % Clear previous plot data
                    cla; % Clear axes
                    
                    % Redraw the sky plot circle and elevation circles again
                    plot(maxRadius * cos(theta), maxRadius * sin(theta), 'k'); % Outer circle
                    
                    % Redraw elevation circles and labels
                    for i = 1:length(elevations)
                        elev = elevations(i);
                        radius = maxRadius * (1 - elev / 90); % Adjusted for 90 degrees at center
                        plot(radius * cos(theta), radius * sin(theta), 'k--', 'LineWidth', 0.5);
                        text(radius * 1.1, 0, elevationLabels{i}, ...
                            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontWeight', 'bold');
                    end
                    
                    % Redraw azimuth labels
                    for i = 1:length(azimuthAngles)
                        angleRad = deg2rad(azimuthAngles(i));
                        text(1.1 * maxRadius * sin(angleRad), 1.1 * maxRadius * cos(angleRad), azimuthLabels{i}, ...
                            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontWeight', 'bold');
                    end
                    
                    % Convert azimuth and elevation to Cartesian coordinates
                    for i = 1:length(azimuths)
                        azRad = deg2rad(azimuths(i)); % Convert azimuth to radians
                        elevRad = deg2rad(elevationsData(i)); % Convert elevation to radians
                        % Calculate the position on the plot
                        radius = maxRadius * (1 - (elevationsData(i) / 90)); % Adjusted for 90 degrees at center
                        x = radius * sin(azRad);
                        y = radius * cos(azRad);
                        scatter(x, y, 36, 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'b');
                    end
                    
                    % Title or labels can be updated if needed
                    title('Real-Time Satellite Tracking Sky Plot');
                    
                    drawnow; % Update the plot
                end
            catch ME
                disp("Error parsing data: " + dataLine);
                disp(ME.message);
            end
        end
    end
end

% Clear the serial object on completion
clear s;
