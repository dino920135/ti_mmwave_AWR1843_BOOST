 clearvars; close all
clear;
delete(instrfind);

%% Settings

N = 1000;

% Setup radar with the parameters from the configuration file
configFile = "../data/20210923/2/xwr18xx_profile_2021_09_23T13_38_57_906.cfg";
ConfigParameters = radarSetup18XX_dat(configFile);

fileName = '../data/20210923/2/xwr18xx_processed_stream_2021_09_23T13_36_44_332.dat';

fid = fopen(fileName);

%% initialize Radar parameters
NUM_ANGLE_BINS = 64;
ANGLE_BINS = asind((-NUM_ANGLE_BINS/2+0.5 : NUM_ANGLE_BINS/2-0.5)'*(2/NUM_ANGLE_BINS));
RANGE_BINS = (0:ConfigParameters.numRangeBins-1) * ConfigParameters.rangeIdxToMeters;

%% Initialize the figure
% radar point scatter plot
f1.fig = figure('Name','Scatter plot','NumberTitle','off');
set(gcf, 'Position', get(0, 'Screensize')*0.6);
f1.H = uicontrol('Style', 'PushButton', ...
                    'String', 'Stop', ...
                    'Callback', 'stopVal = 1','Position',[100 600 100 30]);
f1.h = scatter([],[],'filled');
axis([-4,4,0,5]);
% axis([-0.5,0.5,0,0.9]);
xlabel('X (m)'); ylabel('Y (m)');
grid minor

% radar Heatmap meshplot
f2.fig = figure('Name','Heat map','NumberTitle','off');
set(gcf, 'Position', get(0, 'Screensize')*0.6);
% transform data in polar coordinates to Cartesian coordinates.
theta=repmat(ANGLE_BINS, [1 ConfigParameters.numRangeBins]);
range=repmat(RANGE_BINS, [NUM_ANGLE_BINS 1]);
YY = range.*cosd(theta);
XX = range.*sind(theta);
u = reshape(XX, [], 1);
v = reshape(YY, [], 1);
tri=delaunay(u, v);
rangeY = floor(min(v)):ConfigParameters.rangeResolutionMeters:...
        ceil(max(v));
rangeX = floor(min(u)):deg2rad(ANGLE_BINS(2)-ANGLE_BINS(1)):...
        ceil(max(u));
[X, Y] = meshgrid(rangeX, rangeY);
f2.meshgrid = surf(X, Y, zeros(length(rangeY), length(rangeX)));     %// create an empty "surface" object
f2.meshgrid.CData = zeros(length(rangeY), length(rangeX));
xlabel('Cross-Range [m]');
ylabel('Range [m]');
axis([-4,4,0,5]);

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%&&&&%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%                   MAIN   LOOP              %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%&&&&%%%%%%%%%%%%%%%%%%%%%%%%%

myInd = 0;
lastreadTime = 0;
dataOk = 0;
stopVal = 0;
prevSeparation = -0.3;
previousTime = 0;
prevCentroidPos = nan(1,6);
shoeSize = 26.5/100; % 26.5 cm

tic
while (myInd <= N)
    dataOk = 0;
    timeNow = toc;
    elapsedTime = timeNow - lastreadTime;
    
    % Read the data from the radar:
    [dataOk, frameNumber, detObj] = readAndParseData18XX_dat(fid, ConfigParameters);
    lastreadTime = timeNow;
    
    if dataOk == 1  
        
        myInd = myInd + 1;
        frame{myInd} = detObj; % Store all the data from the radar
        
        % Plot the radar points
%         figure(f1);
        f1.h.XData = detObj.x;
        f1.h.YData = detObj.y;
        if ~isfield(detObj, 'QQ'), myInd = myInd + 1; continue, end
        if myInd < 50, continue, end
        
        % Plot heatmap
%         figure(f2);
        im = detObj.QQ;
        im = sqrt(im);
                
        % mask by range
        im(1:ConfigParameters.numRangeBins .* ConfigParameters.rangeResolutionMeters < 1 , :) = 0;
        im(end-20:end,:)=0;
        
        im_norm = im./max(im(:));
        
        % plot deluney
%         trisurf(tri, reshape(XX, [], 1), reshape(YY, [], 1), sqrt(reshape(im_norm', [], 1)));

        Z = griddata(u, v, reshape(im_norm', [], 1), X, Y);
%         surf(X,Y,Z);
        f2.meshgrid.CData = Z;
        f2.meshgrid.ZData = Z;
        
        drawnow limitrate;
        
    end
    pause(0.001);   
    if stopVal == 1
        stopVal = 0;
        break;
    end   
end
disp('sensorStop');
delete(instrfind);
close all
