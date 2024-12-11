clear all;
close all;
%%%%%%%%%%%
% (Part 1)
%%%%%%%%%%%
filename = input('Name of file to scan including extension (in quotes): ');
im = imread(filename);
height = size(im, 1);
width = size(im, 2);
aspectRatio = height/width;
widthResize = 512;
heightResize = round(widthResize * aspectRatio);
imResize = imresize(im, [heightResize, widthResize]);
figure(1);
imshow(imResize);
axis on;
title('Part 1: resized input');
imwrite(imResize, "output/imResize.png");
imResizeGray = rgb2gray(imResize);
function getAllCorners = getAllCorners(edges, widthResize, heightResize)
    figure(2);
    imshow(edges);
    axis on;
    title('Part 1: edges');
    imwrite(edges, "output/edges.png");
    
    
    
    %%%%%%%%%%%
    % (Part 2)
    %%%%%%%%%%%
    
    d = round(sqrt(widthResize^2 + heightResize^2));
    lineVM = zeros(360, d, 1);
    [imEdgeY, imEdgeX] = find(edges == 1);
    numIterations = length(imEdgeX);
    h = waitbar(0, 'Processing...');
    c = 0;
    for i = 1:length(imEdgeX)
        x = imEdgeX(i);
        y = imEdgeY(i);
        for thetaDeg = 1:360
            thetaRad = deg2rad(thetaDeg);
	        r = round(cos(thetaRad)*x + sin(thetaRad)*y);
            if r > 0 && r <= d
		        lineVM(thetaDeg, r, 1) = lineVM(thetaDeg, r, 1) + 1;
            end
        end
        waitbar(i / numIterations, h, sprintf('Line Hough Progress: %d%%', round((i / numIterations) * 100)));
    end
    close(h);
    
    figure(3);
    imshow(lineVM/255);
    axis on;
    title('Part 2: 2D hough transform histogram');
    imwrite(lineVM/255, "output/lineVM.png");
    
    
    
    %%%%%%%%%%%
    % (Part 3)
    %%%%%%%%%%%
        
    numTopLines = 10; 
    topLines = zeros(numTopLines, 3); 
    
    lineVMTemp = lineVM;
    
    for k = 1:numTopLines
        [maxLineVM, i] = max(lineVMTemp(:)); 
        [thetaMaxLine, rMaxLine] = ind2sub(size(lineVMTemp), i); 
        topLines(k, :) = [thetaMaxLine, rMaxLine, maxLineVM];
    
        rRange = max(1, rMaxLine-1):min(d, rMaxLine+1);
        thetaRange = max(1, thetaMaxLine-1):min(360, thetaMaxLine+1);
        lineVMTemp(thetaRange, rRange) = 0;
    end
    
    imLines = cat(3, edges, edges, edges); 
    imLines = im2uint8(imLines);
    
    for k = 1:numTopLines
        thetaRad = deg2rad(topLines(k, 1));
        r = topLines(k, 2);
    
        for x = 1:widthResize
            y = round((-cos(thetaRad) / sin(thetaRad)) * x + (r / sin(thetaRad)));
            if y >= 1 && y <= heightResize
                imLines(y, x, 1) = 255;
                imLines(y, x, 2) = 0;   
                imLines(y, x, 3) = 0;   
            end
        end
    end
    
    figure(4);
    imshow(imLines);
    axis on;
    titleStr = sprintf("Part 3: %d selected lines superimposed onto edges image", numTopLines);
    title(titleStr);
    imwrite(imLines, "output/imLines.png")
    
    
    
    %%%%%%%%%%%
    % (Part 4)
    %%%%%%%%%%%
    
    % remove duplicate lines
    toleranceTheta = 80; 
    toleranceR = 80;
    consolidatedLines = [];
    
    for i = 1:numTopLines
        currentLine = topLines(i, :);
        if isempty(consolidatedLines)
            consolidatedLines = currentLine; 
        else
            isUnique = true;
            for j = 1:size(consolidatedLines, 1)
                if abs(currentLine(1) - consolidatedLines(j, 1)) <= toleranceTheta && ...
                   abs(currentLine(2) - consolidatedLines(j, 2)) <= toleranceR
                    if currentLine(3) > consolidatedLines(j, 3)
                        consolidatedLines(j, :) = currentLine;
                    end
                    isUnique = false;
                    break;
                end
            end
            if isUnique
                consolidatedLines = [consolidatedLines; currentLine];
            end
        end
    end
    
    
    
    
    
    % get all corners within bounds
    syms x y;
    validCorners = []; 
    
    for i = 1:size(consolidatedLines, 1)
        for j = i+1:size(consolidatedLines, 1)
    % for i = 1:4
    %     for j = i+1:4
            line1 = consolidatedLines(i, :);
            thetaRad1 = deg2rad(line1(1));
            r1 = line1(2);
            m1 = -cos(thetaRad1) / sin(thetaRad1);
            b1 = r1 / sin(thetaRad1);
            line1Eq = y == m1*x+b1;
    
            line2 = consolidatedLines(j, :);
            thetaRad2 = deg2rad(line2(1));
            r2 = line2(2);
            m2 = -cos(thetaRad2) / sin(thetaRad2);
            b2 = r2 / sin(thetaRad2);
            line2Eq = y == m2*x+b2;
    
            if m1 == m2
                continue;
            end
    
            corner = solve([line1Eq, line2Eq], [x, y]);
    
            cornerX = round(double(corner.x));
            cornerY = round(double(corner.y));
    
            if cornerX >= 1 && cornerX <= widthResize && ...
               cornerY >= 1 && cornerY <= heightResize
               validCorners = [validCorners; cornerX, cornerY];
            end
        end
    end
    getAllCorners = validCorners;
end

% Initialize threshold for edge detection
threshold = 0.2;

% Perform edge detection with initial threshold
edges = edge(imResizeGray, 'canny', threshold);
[imEdgeY, imEdgeX] = find(edges == 1);

% Get initial set of corners
allCorners = getAllCorners(edges, widthResize, heightResize);
numCorners = size(allCorners, 1);
fprintf("Initial threshold = %.2f, numCorners = %d\n", threshold, numCorners);

% Adjust threshold to reduce number of corners
while numCorners > 4 && threshold < 0.95
    threshold = threshold + 0.05;
    edges = edge(imResizeGray, 'canny', threshold);
    allCorners = getAllCorners(edges, widthResize, heightResize);
    numCorners = size(allCorners, 1);
    fprintf("Updated threshold = %.2f, numCorners = %d\n", threshold, numCorners);
end

% Use the filtered corners as valid corners for K-means
validCorners = allCorners;

% Initialize four centroids manually (one near each corner of the image)
centroids = [
    1, 1;                     % Top-left
    widthResize, 1;           % Top-right
    widthResize, heightResize; % Bottom-right
    1, heightResize           % Bottom-left
];

% Tolerance to stop updating centroids
tolerance = 1e-3;
maxIterations = 100; % To prevent infinite loops

for iter = 1:maxIterations
    % Assign points to the nearest centroid
    clusterAssignments = zeros(size(validCorners, 1), 1);
    for i = 1:size(validCorners, 1)
        distances = vecnorm(validCorners(i, :) - centroids, 2, 2); % Euclidean distance
        [~, clusterAssignments(i)] = min(distances);
    end

    % Recompute centroids
    newCentroids = zeros(size(centroids));
    for i = 1:size(centroids, 1)
        clusterPoints = validCorners(clusterAssignments == i, :);
        if ~isempty(clusterPoints)
            newCentroids(i, :) = mean(clusterPoints, 1);
        else
            % If a cluster is empty, retain the old centroid
            newCentroids(i, :) = centroids(i, :);
        end
    end

    % Check for convergence
    if max(vecnorm(newCentroids - centroids, 2, 2)) < tolerance
        break;
    end

    centroids = newCentroids;
end

% Assign final clusters
topLeft = validCorners(clusterAssignments == 1, :);
topRight = validCorners(clusterAssignments == 2, :);
bottomRight = validCorners(clusterAssignments == 3, :);
bottomLeft = validCorners(clusterAssignments == 4, :);

% Calculate the final coordinates (averages) for each cluster
topLeftCenter = round(mean(topLeft, 1));
topRightCenter = round(mean(topRight, 1));
bottomLeftCenter = round(mean(bottomLeft, 1));
bottomRightCenter = round(mean(bottomRight, 1));

paperCorners = [topLeftCenter; topRightCenter; bottomRightCenter; bottomLeftCenter];

% Superimpose 4 corners
imCorners = imResize; 
imCorners = im2uint8(imCorners); 
highlightRadius = 2; 

for i = 1:length(paperCorners)
    y = paperCorners(i, 2);
    x = paperCorners(i, 1);

    xMin = max(1, x - highlightRadius);
    xMax = min(size(imCorners, 2), x + highlightRadius);
    yMin = max(1, y - highlightRadius);
    yMax = min(size(imCorners, 1), y + highlightRadius);

    imCorners(yMin:yMax, xMin:xMax, 1) = 255; 
    imCorners(yMin:yMax, xMin:xMax, 2) = 0;   
    imCorners(yMin:yMax, xMin:xMax, 3) = 0;  
end

figure();
imshow(imCorners);
axis on;
title('Corners Detected using Edge Detection and K-means Clustering');
imwrite(imCorners, "output/imCorners.png");
%%%%%%%%%%%
% (Part 5)
%%%%%%%%%%%
letterWidth = 850;  
letterHeight = 1100;  
destPoints = [
    1, 1;                   
    letterWidth, 1;          
    letterWidth, letterHeight;  
    1, letterHeight         
];
function sortedCorners = sortCorners(corners)
    sorted = sortrows(corners, 2);
    top = sorted(1:2, :);
    bottom = sorted(3:4, :);
    if top(1,1) < top(2,1)
        topLeft = top(1,:);
        topRight = top(2,:);
    else
        topRight = top(1,:);
        topLeft = top(2,:); 
    end
    if bottom(1,1) < bottom(2,1)
        bottomLeft = bottom(1,:);
        bottomRight = bottom(2,:);
    else
        bottomRight = bottom(1,:);
        bottomLeft = bottom(2,:); 
    end
    sortedCorners = [topLeft; topRight; bottomRight; bottomLeft];
end
sortedCorners = sortCorners(allCorners);
A = [];
for i = 1:4
    x = sortedCorners(i,1);
    y = sortedCorners(i,2);
    u = destPoints(i,1);
    v = destPoints(i,2);
    
    A = [A; 
        x, y, 1, 0, 0, 0, -u*x, -u*y, -u;
        0, 0, 0, x, y, 1, -v*x, -v*y, -v];
end
[~,~,V] = svd(A);
H = reshape(V(:,end), 3, 3)';
H = H ./ H(3,3);
imRectified = zeros(letterHeight, letterWidth, 3, 'uint8');
numIterations = letterHeight;
h = waitbar(0, 'Processing...');
for v = 1:letterHeight
    for u = 1:letterWidth
        temp = H \ [u; v; 1];
        x = round(temp(1) / temp(3));
        y = round(temp(2) / temp(3));
        
        if x > 0 && x <= widthResize && y > 0 && y <= heightResize
            imRectified(v, u, :) = imResize(y, x, :);
        end
    end
    waitbar(v / numIterations, h, sprintf('Rectification Progress: %d%%', round((v / numIterations) * 100)));
end
close(h);
figure(6);
imshow(imRectified);
axis on;
title('Part 5: rectified');
imwrite(imRectified, 'output/imRectified.png');
