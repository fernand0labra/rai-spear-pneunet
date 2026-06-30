% Load example images (make sure they are the same height)
img1 = imread('spa0.jpg');
img2 = imread('spa1.jpg');
img3 = imread('spa2.jpg');

% Resize to same height if needed
targetHeight = min([size(img1,1), size(img2,1), size(img3,1)]);
img1 = imresize(img1, [targetHeight NaN]);
img2 = imresize(img2, [targetHeight NaN]);
img3 = imresize(img3, [targetHeight NaN]);

% Concatenate horizontally
combined = [img1 img2 img3];

% Create figure with white background
figure('Color','w');
imshow(combined);
axis off;

% Compute x positions for each image
widths = [size(img1,2), size(img2,2), size(img3,2)];
xpos = [10, widths(1)+10, widths(1)+widths(2)+10];  % label offsets
ypos = 30;  % label height from top

% Add labels (a), (b), (c) with white box and black text
for i = 1:3
label = sprintf('(%c)', 'a' + i - 1);
text(xpos(i), ypos, label, ...
'FontSize', 14, ...
'FontWeight', 'bold', ...
'Color', 'k', ...              % black text
'BackgroundColor', 'w', ...    % white box
'EdgeColor', 'k', ...          % black border
'Margin', 4, ...
'Interpreter', 'none');
end