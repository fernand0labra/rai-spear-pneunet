% Example images (replace with your own)
img1 = imread('imgs/softdrone.side.sim.actuated.jpg'); % top image
img2 = imread('imgs/softdrone.top.sim.stable.jpg');    % bottom-left
img3 = imread('imgs/softdrone.top.sim.actuated.png');  % bottom-right

% Convert grayscale images to RGB
if size(img2,3)==1, img2 = repmat(img2,[1 1 3]); end
if size(img3,3)==1, img3 = repmat(img3,[1 1 3]); end

% Resize images to align properly
topWidth = 600;
bottomWidth = 300;
topHeight = 300;
bottomHeight = 300;

img1 = imresize(img1, [topHeight topWidth]);
img2 = imresize(img2, [bottomHeight bottomWidth]);
img3 = imresize(img3, [bottomHeight bottomWidth]);

% Combine bottom two images horizontally
bottomRow = [img2 img3];

% Make white background in case sizes don't match perfectly
background = uint8(255 * ones(topHeight + bottomHeight, topWidth, 3));

% Place top image
background(1:topHeight, :, :) = img1;

% Place bottom row (centered horizontally)
background(topHeight+1:end, :, :) = bottomRow;

% Display the final combined image
figure('Color', 'w');
imshow(background);
axis off;

% === Add (a), (b), (c) labels ===
% Coordinates for labels
labelA_pos = [20, 40];                          % top-left of first image
labelB_pos = [20, topHeight + 40];              % bottom-left image
labelC_pos = [bottomWidth + 20, topHeight + 40]; % bottom-right image

positions = [labelA_pos; labelB_pos; labelC_pos];

for i = 1:3
    label = sprintf('(%c)', 'a' + i - 1);
    text(positions(i,1), positions(i,2), label, ...
        'FontSize', 14, ...
        'FontWeight', 'bold', ...
        'Color', 'k', ...              % black text
        'BackgroundColor', 'w', ...    % white box
        'EdgeColor', 'k', ...          % black border
        'Margin', 4, ...
        'Interpreter', 'none');
end