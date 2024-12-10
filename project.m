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

newWidth = 512;
newHeight = round(newWidth * aspectRatio);
newIm = imresize(im, [newHeight, newWidth]);

figure();
imshow(im);
axis on;
title('im');

newImGray = rgb2gray(newIm);
edges = edge(newImGray, 'canny', 0.9);

figure();
imshow(edges);
axis on;
title('edges');
imwrite(edges, "edges.png");

%%%%%%%%%%%
% (Part 2)
%%%%%%%%%%%
%  Line Hough
d = round(sqrt(newWidth^2 + newHeight^2));
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

figure();
imshow(lineVM/255);
axis on;
title('hough transform');
imwrite(lineVM, "lineVM.png");

%%%%%%%%%%%
% (Part 3)
%%%%%%%%%%%

[maxLineVM, i] = max(lineVM(:));
normLineVM = lineVM / maxLineVM;
[thetaMaxLine, rMaxLine] = ind2sub(size(normLineVM), i);

binaryNormLineVM = normLineVM;
binaryNormLineVM(binaryNormLineVM < 1) = 0;


fprintf('LineVM: The maximum value is %d, located at theta = %d, r = %d.\n', maxLineVM, thetaMaxLine, rMaxLine);
m = round(cos(thetaMaxLine) / sin(thetaMaxLine));
b = round(-rMaxLine / sin(thetaMaxLine));
fprintf('Line: m = %d, b = %d.\n', m, b);
fprintf('%d %d\n', maxLineVM, length(imEdgeX));

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

toleranceTheta = 5; 
toleranceR = 10;
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

consolidatedLines = sortrows(consolidatedLines, -3); 
consolidatedLines = consolidatedLines(1:4, :);

imOverlay = cat(3, edges, edges, edges); 
imOverlay = im2uint8(imOverlay);


for k = 1:numTopLines
    thetaRad = deg2rad(topLines(k, 1));
    r = topLines(k, 2);

    for x = 1:newWidth
        y = round((-cos(thetaRad) / sin(thetaRad)) * x + (r / sin(thetaRad)));
        if y >= 1 && y <= newHeight
            imOverlay(y, x, 1) = 255;
            imOverlay(y, x, 2) = 0;   
            imOverlay(y, x, 3) = 0;   
        end
    end
end

figure();
imshow(imOverlay);
title('Edges with Superimposed Lines');
imwrite(imOverlay, "imOverlay.png")

%%%%%%%%%%%
% (Part 4)
%%%%%%%%%%%
syms x y;
validCorners = []; 

for i = 1:4
    for j = i+1:4
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
        

        if cornerX >= 1 && cornerX <= newWidth && ...
           cornerY >= 1 && cornerY <= newHeight
            validCorners = [validCorners; cornerX, cornerY];
        end
    end
end

disp('Valid corners:');
disp(validCorners);

imOverlayCorners = newIm; 
imOverlayCorners = im2uint8(imOverlayCorners); 
highlightRadius = 2; 

for i = 1:length(validCorners)
    y = validCorners(i, 2);
    x = validCorners(i, 1);

    xMin = max(1, x - highlightRadius);
    xMax = min(size(imOverlayCorners, 2), x + highlightRadius);
    yMin = max(1, y - highlightRadius);
    yMax = min(size(imOverlayCorners, 1), y + highlightRadius);

    imOverlayCorners(yMin:yMax, xMin:xMax, 1) = 255; 
    imOverlayCorners(yMin:yMax, xMin:xMax, 2) = 0;   
    imOverlayCorners(yMin:yMax, xMin:xMax, 3) = 0;  
end

figure();
imshow(imOverlayCorners);
axis on;
title('Edges with Superimposed Corners and Highlight Region');
imwrite(imOverlayCorners, "imOverlayCorners.png");


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

sortedCorners = sortCorners(validCorners);

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

rectifiedIm = zeros(letterHeight, letterWidth, 3, 'uint8');

for v = 1:letterHeight
    for u = 1:letterWidth
        temp = H \ [u; v; 1];
        x = round(temp(1) / temp(3));
        y = round(temp(2) / temp(3));
        
        if x > 0 && x <= newWidth && y > 0 && y <= newHeight
            rectifiedIm(v, u, :) = newIm(y, x, :);
        end
    end
end

figure;
imshow(rectifiedIm);
title('Rectified Image');
imwrite(rectifiedIm, 'rectified_paper.png');

function sortedCorners = sortCorners(corners)

    sorted = sortrows(corners, 1);

    left = sorted(1:2, :);
    right = sorted(3:4, :);

    if left(1,2) < left(2,2)
        topLeft = left(1,:);
        bottomLeft = left(2,:);
    else
        bottomLeft = left(1,:);
        topLeft = left(2,:); 
    end

    if right(1,2) < right(2,2)
        topRight = right(1,:);
        bottomRight = right(2,:);
    else
        bottomRight = right(1,:);
        topRight = right(2,:); 
    end

    sortedCorners = [topLeft; topRight; bottomRight; bottomLeft];
end
