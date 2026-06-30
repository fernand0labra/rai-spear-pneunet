clear; clc; close all;

% ------------------ User Parameters ---------------------------
experiment = 'real';  % 'real' or 'sim'
throttles = {'00', '10', '20', '30'};  % '00', '10', '20', '30', '40'
axesList = {'x', 'y', 'z'};

dataDir = 'data/';  % base folder

% Specify roll, pitch, yaw in degrees
roll_deg  = 0;   % rotation around X-axis
pitch_deg = 0;   % rotation around Y-axis
yaw_deg   = 35;   % rotation around Z-axis (35)

% Convert to radians
r = deg2rad(roll_deg);
p = deg2rad(pitch_deg);
y = deg2rad(yaw_deg);

% Rotation matrices for each axis
Rx = [1,      0,       0;
      0, cos(r), -sin(r);
      0, sin(r),  cos(r)];

Ry = [cos(p),  0, sin(p);
      0,       1,      0;
     -sin(p),  0, cos(p)];

Rz = [cos(y), -sin(y), 0;
      sin(y),  cos(y), 0;
      0,       0,      1];

% Combine into a full rotation matrix (XYZ convention: roll -> pitch -> yaw)
R = Rz * Ry * Rx;

% ------------------ Loop over axes -----------------------------
for a = 1:numel(axesList)
    axisName = axesList{a};
    figure('Color', 'w', 'Position', [100 100 900 600]); hold on;

    offset = 0;
    for t = 1:numel(throttles) 
        throttle = throttles{t};

        % --- Load X, Y, Z for rotation ---
        data = struct();
        for axIdx = 1:3
            axName = axesList{axIdx};
            filePath = fullfile(dataDir, throttle, experiment, [axName '.txt']);
            fid = fopen(filePath, 'r');

            floatValues = [];
            ldx = 0;
            while ~feof(fid)
                ldx = ldx + 1;
                line = fgetl(fid);
                if ~ischar(line), break; end
                if ldx < 100, continue; end

                % Skip and stop conditions
                if strcmp(experiment, 'real')
                    if t == 1
                        if ldx < 1700, continue
                        elseif ldx == 9700, break; end
                    elseif t == 2
                        if ldx < 1600, continue
                        elseif ldx == 9600, break; end
                    elseif t == 3
                        if ldx < 2200, continue
                        elseif ldx == 10200, break; end
                    elseif t == 4
                        if ldx < 2000, continue
                        elseif ldx == 10000, break; end
                    end
                end

                val = str2double(strtrim(line));
                if isnan(val)
                    continue;
                end
                floatValues(end+1,1) = val; %#ok<SAGROW>
            end
            fclose(fid);

            % Unit conversion
            if strcmp(experiment, 'sim'),       floatValues = floatValues / 10;
            elseif strcmp(experiment, 'real'),  floatValues = floatValues * 100; end

            data.(axName) = floatValues;
        end

        % --- Ensure same length across axes for rotation ---
        minLen = min([numel(data.x), numel(data.y), numel(data.z)]);
        if minLen == 0
            continue;
        end
        X = data.x(1:minLen);
        Y = data.y(1:minLen);
        Z = data.z(1:minLen);

        % --- Apply rotation about Z-axis ---
        rotated = R * [X'; Y'; Z'];
        Xr = rotated(1,:)';
        Yr = rotated(2,:)';
        Zr = rotated(3,:)';

        % --- Use the rotated axis data for plotting ---
        switch axisName
            case 'x', floatValues = Xr; 
            case 'y', floatValues = Y;
            case 'z', floatValues = Zr;
        end

        % if strcmp(throttle, '00'),  offset = floatValues(1); end
        floatValues = floatValues - floatValues(1);

        % --- Create x-values (time vector, unchanged) ---
        if strcmp(experiment, 'real'),  xValues = (1:1:numel(floatValues))' * 0.005;   % 200 Hz
        else,                           xValues = (1:1:numel(floatValues))' * 0.01; end% 100 Hz

        % --- Plot data (titles unchanged) ---
        plot(xValues, floatValues, 'DisplayName', sprintf('%s%% Throttle', throttle), 'LineWidth', 0.8);
    end

    % ------------------ Plot Formatting (unchanged) -------------------------
    title(['\textbf{Simulated Position Values on ' upper(axisName) ' Axis}'], 'Interpreter','latex','FontSize',30);
    xlabel('Time [s]','Interpreter','latex','FontSize',20);
    ylabel('Displacement [cm]','Interpreter','latex','FontSize',20);

    % ylim([-2.0, 2.0])
    xlim([0 40])

    ax = gca;
    set(ax, 'FontSize',14, 'XGrid','on', 'YGrid','on', 'Box','on', 'YColor',[0 0 0], 'Color','w');
    legend('show', 'Location', 'best');
    grid on;
    hold off;
end