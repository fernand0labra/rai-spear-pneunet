clear; clc; close all;

% ------------------ User Parameters ---------------------------
experiments = {'Q1', 'Q2', 'Q3', 'Q4'};
sigName = 'e_mag';

dataDir = 'data/';     % base folder
dt = 0.01;             % 100 Hz sampling

% ------------------ Single figure for all experiments -------------------
figure('Color', 'w', 'Position', [100 100 900 600]);
hold on;

h = gobjects(numel(experiments), 1);
labels = cell(numel(experiments), 1);

minLenAll = inf;
allVals = cell(numel(experiments), 1);

% -------- Read all experiments first --------
for t = 1:numel(experiments)
    edx = experiments{t};
    filePath = fullfile(dataDir, edx, [sigName '.txt']);

    fid = fopen(filePath, 'r');
    if fid == -1
        warning('Could not open file: %s. Skipping experiment %s.', filePath, edx);
        allVals{t} = [];
        continue;
    end

    vals = [];
    while ~feof(fid)
        line = fgetl(fid);
        if ~ischar(line), break; end

        v = str2double(strtrim(line));
        if isnan(v), continue; end

        vals(end+1,1) = v; %#ok<SAGROW>
    end
    fclose(fid);

    allVals{t} = vals;
    if ~isempty(vals)
        minLenAll = min(minLenAll, numel(vals));
    end
end

% If nothing loaded, stop
if isinf(minLenAll) || minLenAll == 0
    error('No valid data loaded from any experiment.');
end

xValues = (1:minLenAll)' * dt;

% -------- Plot each experiment on the same axes --------
for t = 1:numel(experiments)
    vals = allVals{t};
    vals = vals/10;  % mm -> cm

    if isempty(vals)
        continue;
    end

    vals = vals(1:minLenAll);

    % Optional baseline removal (uncomment if desired)
    % vals = vals - vals(1);

    switch experiments{t}
        case 'Q1', displaytitle = '$1^{st}$ Quadrant';
        case 'Q2', displaytitle = '$2^{nd}$ Quadrant';
        case 'Q3', displaytitle = '$3^{rd}$ Quadrant';
        case 'Q4', displaytitle = '$4^{th}$ Quadrant';
    end

    labels{t} = strrep(displaytitle, '.', '\.'); % latex-friendly for dots
    h(t) = plot(xValues, vals, 'LineWidth', 0.8, 'DisplayName', labels{t});
end

% Remove empty handles (if some experiments were skipped)
keep = isgraphics(h);
h = h(keep);
labels = labels(keep);

% ------------------ Plot Formatting -------------------------
%title(sprintf('%s (all experiments)', sigName), 'Interpreter', 'latex', 'FontSize', 30);
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 20);
ylabel('Position Error (cm)', 'Interpreter', 'latex', 'FontSize', 20);

xlim([0 1.5])

ax = gca;
set(ax, 'FontSize', 14, 'XGrid', 'on', 'YGrid', 'on', 'Box', 'on', ...
    'YColor', [0 0 0], 'Color', 'w');

legend(h, labels, 'Interpreter', 'latex', 'Location', 'northeast');
grid on;
hold off;