% Load satellite TLE data (replace with actual data)
tle = {
    'ONEWEB-0208',...
  '1 48243U 21031AK  24298.74286871  .00000032  00000+0  51320-4 0  9995',...
  '2 48243  87.9019  49.0995 0001636  94.3263 265.8054 13.14505755169424'
};

% Set up the figure with a specific map projection
figure;
axesm('MapProjection', 'mercator', 'Frame', 'on', 'Grid', 'on');

% Initialize satellite object (Note: Use Satellite Communication Toolbox)
sat = satellite(tle{1}, tle{2}, tle{3});

% Time interval for the simulation
dt = 60; % seconds
numPositions = 10; % Number of positions to calculate

% Loop to calculate position
for i = 1:numPositions
    % Update satellite position based on time
    % Get current position
    [lat, lon] = sat.position(); % Get current latitude and longitude
    plotm(lat, lon, 'ro'); % Plot satellite position
    pause(1); % Pause to visualize movement
    % Update satellite time (optional, if you want to show movement over time)
    sat.Time = sat.Time + seconds(dt); % Increment time
end

title('Satellite Path');
