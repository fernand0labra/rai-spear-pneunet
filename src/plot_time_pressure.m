clear; clc; close all;

% ------------------ User Parameters ---------------------------
experiment = {'Q1', 'Q2', 'Q3', 'Q4'};
cavity = {'p1', 'p2', 'p3', 'p4'};

dataDir = 'data/';  % base folder

% ------------------ Loop over experiments -----------------------------
for t = 1:numel(experiment)
    edx = experiment{t};

    figure('Color', 'w', 'Position', [100 100 900 600]); hold on;

    % -------- Read all cavity files once for this experiment --------
    data = struct();
    for axIdx = 1:4
        cdx = cavity{axIdx};
        filePath = fullfile(dataDir, edx, [cdx '.txt']);
        fid = fopen(filePath, 'r');

        floatValues = [];
        while ~feof(fid)
            line = fgetl(fid);
            if ~ischar(line), break; end

            val = str2double(strtrim(line));
            if isnan(val), continue; end
            floatValues(end+1,1) = val; %#ok<SAGROW>
        end
        fclose(fid);

        % Unit conversion
        floatValues = floatValues * 1000;
        data.(cdx) = floatValues;
    end

    % --- Ensure same length across axes for rotation ---
    minLen = min([numel(data.p3), numel(data.p4), numel(data.p1), numel(data.p2)]);
    if minLen == 0
        hold off;
        continue;
    end

    p3 = data.p3(1:minLen);
    p4 = data.p4(1:minLen);
    p1 = data.p1(1:minLen);
    p2 = data.p2(1:minLen);

    % -------- Plot each cavity (axis) as a line in this experiment figure --------
    for a = 1:numel(cavity)
        axisName = cavity{a};

        switch axisName
            case 'p1', floatValues = p1; displayname = '$P_1$';
            case 'p2', floatValues = p2; displayname = '$P_2$';
            case 'p3', floatValues = p3; displayname = '$P_3$';
            case 'p4', floatValues = p4; displayname = '$P_4$';
        end

        floatValues = floatValues - floatValues(1);
        xValues = (1:numel(floatValues))' * 0.01;  % 100 Hz

        plot(xValues, floatValues, ...
            'DisplayName', displayname, ...
            'LineWidth', 0.8);
    end

    switch edx
        case 'Q1',  displaytitle = '$1^{st}$ Quadrant Experiment';
        case 'Q2', displaytitle = '$2^{nd}$ Quadrant Experiment';
        case 'Q3',  displaytitle = '$3^{rd}$ Quadrant Experiment';
        case 'Q4', displaytitle = '$4^{th}$ Quadrant Experiment';
    end

    % ------------------ Plot Formatting (adapted) -------------------------
    %title(sprintf('%s', displaytitle), 'Interpreter','latex','FontSize',30);  % experiment title
    xlabel('Time [s]','Interpreter','latex','FontSize',20);
    ylabel('Pressure [Pa]','Interpreter','latex','FontSize',20);

    ylim([0.0, 1.05])
    xlim([0 1.5])

    ax = gca;
    set(ax, 'FontSize',14, 'XGrid','on', 'YGrid','on', 'Box','on', ...
        'YColor',[0 0 0], 'Color','w');

    legend('show', 'Location', 'northwest', 'Interpreter','latex');
    grid on;
    hold off;
end