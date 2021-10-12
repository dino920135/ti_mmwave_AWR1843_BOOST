clearvars; close all
clear;
delete(instrfind);

N = 1000;

% Setup radar with the parameters from the configuration file
configFile = "../data/20210923/2/xwr18xx_profile_2021_09_23T13_38_57_906.cfg";
ConfigParameters = radarSetup18XX_dat(configFile);

fileName = '../data/20210923/2/xwr18xx_processed_stream_2021_09_23T13_36_44_332.dat';

fid = fopen(fileName);

%% Initialize the figure
% f1=figure('Name','Scatter plot','NumberTitle','off');
% set(gcf, 'Position', get(0, 'Screensize'));
% H = uicontrol('Style', 'PushButton', ...
%                     'String', 'Stop', ...
%                     'Callback', 'stopVal = 1','Position',[100 600 100 30]);
% h = scatter([],[],'filled');
% axis([-4,4,0,5]);
% % axis([-0.5,0.5,0,0.9]);
% xlabel('X (m)'); ylabel('Y (m)');
% grid minor
% f2=figure('Name','Heat map','NumberTitle','off');
f3=figure('Name','Projection Profile','NumberTitle','off');
% f4=figure('Name','Transformation','NumberTitle','off');
% movegui(f4);
% hViewPanel = uipanel(f4,'Position',[0 0 1 1],'Title','Plot of Optical Flow Vectors');
% hPlot = axes(hViewPanel);

f5=figure('Name', 'Points', 'NumberTitle', 'off');
f6=figure('Name', 'Range-Azimuth heatmap', 'NumberTitle', 'off');
im_pre = [];
position = [0, 0, 1];
detect_points = [,];
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
%         h.XData = detObj.x;
%         h.YData = detObj.y;
        if ~isfield(detObj, 'QQ'), myInd = myInd + 1; continue, end
        if myInd < 50, continue, end
        % Plot heatmap
%         figure(f2);
        im = detObj.QQ;
%         im = sqrt(im);
        % transform data in polar coordinates to Cartesian coordinates.
        NUM_ANGLE_BINS = 64;
        theta=repmat(detObj.theta, [1 ConfigParameters.numRangeBins]);
        range=repmat(detObj.range, [NUM_ANGLE_BINS 1]);
        YY = range.*cosd(theta);
        XX = range.*sind(theta);
        % mask by range
        im(1:ConfigParameters.numRangeBins * ConfigParameters.rangeIdxToMeters < 1 , :) = 0;
        im(end-20:end,:)=0;
        
        im_norm = im./max(im(:));
%         a = pcolor(XX', YY', im_norm);
%         a.LineStyle = 'none';
%         xlabel('Cross-Range [m]');
%         ylabel('Range [m]');
%         colormap('jet');
%         xlim([-10 10]);
%         ylim([0 20]);  
%         drawnow limitrate;
%         title('Range-Azimuth Heatmap')
        
        figure(f6)
        subplot(4, 1, 1:3)
        imagesc(im_norm)
        subplot(4, 1, 4)
        plot( sum(im));
        
        
        figure(f3)
        subplot(4, 1, 1:3)
        % plot deluney
        u = reshape(XX, [], 1);
        v = reshape(YY, [], 1);
        tri=delaunay(u, v);
%         trisurf(tri, reshape(XX, [], 1), reshape(YY, [], 1), sqrt(reshape(im', [], 1)));
        rangeY=floor(min(v)):ConfigParameters.rangeIdxToMeters:...
                ceil(max(v));
        rangeX=floor(min(u)):0.3:...
                ceil(max(u));
        [X, Y]=meshgrid(rangeX, rangeY);
        Z = griddata(u, v, reshape(im', [], 1), X, Y);
%         surf(X,Y,Z);
        imagesc(Z');
        title('Radar Heatmap')
        ylabel('Cross Range [bin]')
        
        subplot(4, 1, 4)
        Z(isnan(Z)) = 0;
        Z(1:10, :) = 0;
        [pks, locs] = findpeaks(sum(Z', 1), 'MinPeakProminence',std(sum(Z', 1)));
        hold off
        plot(sum(Z', 1));
        hold on 
        plot(locs, pks, 'o');
        axis tight;
        xlabel('Range [bin]')
        ylabel('Intensity')
        
        figure(f5)
        hold on;
        plot(ones( length(locs), 1)*myInd, locs, 'b.');
        xlabel('Time[frame]')
        ylabel('Range[bin]')
        title('Range-Time')
        detect_points = [detect_points; ones(length(locs), 1)*myInd, locs'];
        %{
        figure(f4)
        Z = Z./max(max(Z))*255;
        if myInd > 50 %&& mod(myInd, 2) == 0
            % ORB matching
            ptsOriginal  = detectHarrisFeatures(im_pre);
            ptsDistorted = detectHarrisFeatures(Z);
            if length(ptsOriginal)<3 || length(ptsOriginal)<3
                continue;
            end

            [featuresOriginal,validPtsOriginal] = extractFeatures(im_pre,ptsOriginal);
            [featuresDistorted,validPtsDistorted] = extractFeatures(Z,ptsDistorted);
            
            index_pairs = matchFeatures(featuresOriginal,featuresDistorted);
            matchedPtsOriginal  = validPtsOriginal(index_pairs(:,1));
            matchedPtsDistorted = validPtsDistorted(index_pairs(:,2));
            
            if length(matchedPtsOriginal)<3 || length(matchedPtsDistorted)<3
                continue;
            end
            
            [tform,inlierIdx] = estimateGeometricTransform2D(matchedPtsDistorted, matchedPtsOriginal, 'rigid');
            inlierPtsDistorted = matchedPtsDistorted(inlierIdx,:);
            inlierPtsOriginal  = matchedPtsOriginal(inlierIdx,:);

            showMatchedFeatures(im_pre, Z, inlierPtsOriginal, inlierPtsDistorted,'montage');
            title('Matched Inlier Points')

            % Optical Flow
%             opticFlow = opticalFlowLK('NoiseThreshold',0.01);
%             flow = estimateFlow(opticFlow, Z-im_pre);
%             image(Z-im_pre)
%             hold on
%             plot(flow,'DecimationFactor',[10 10],'ScaleFactor',100,'Parent',hPlot);
%             hold off
            
            
%             figure(f5)
%             subplot(2, 1, 1)
            hold on
%             plot(myInd, matchedPtsDistorted.Location(:, 1), '.');            
%             subplot(2, 1, 2)
%             hold on
%             plot(myInd, matchedPtsDistorted.Location(:, 2), '.');

            position = [position; [tform.T' * position(end, :)']'];
            plot(position(end, 1), position(end, 2), 'bo');
        end
        %}
        im_pre = Z;
    end
    pause(0.001);   
    if stopVal == 1
        stopVal = 0;
        break;
    end   
end
fprintf(UART_sphandle, 'sensorStop');
delete(instrfind);
close all

function displayPolarHeatmap(rangeAzimuth_2plot, theta, range)
    %figure(1)
    cLim = [0, Inf];
    imagesc_polar2(theta, range, rangeAzimuth_2plot, cLim); hold on

    xlabel('Cross-Range [m]');
    ylabel('Range [m]');
    yLim = [0, range(end)];
    xLim = yLim(2)*sin(max(abs(theta))) * [-1,1];
    ylim(yLim);
    xlim(xLim);
    delta = 0.2;
    set(gca, 'Xtick', [-50:delta:50]);
    set(gca, 'Ytick', [0:delta:100]);
    set(gca,'Color', [0.5 0.5 0.5])
    grid on;
end



function displayRectangleHeatmap(rangeAzimuth, theta_degree, range, heattype)

    figure(1)

    cLim = [0, Inf];
    imagesc(theta_degree, range, rangeAzimuth, cLim);

    set(gca,'YDir','normal')
    if (heattype == 1)
      xlabel('Azimuth Angle [degree]');
    else
      xlabel('Elevation [m]');
    end
    ylabel('Range [m]');
end


function imagesc_polar2(theta, rr, im, cLim) %==>>
% Plot imagesc-like plot in polar coordinates using pcolor()

if nargin<4, cLim = []; end
theta=repmat(detObj.theta,[1 512]);
range=repmat(detObj.range,[63 1]);
% transform data in polar coordinates to Cartesian coordinates.
YY = range.*cos(theta);
XX = range.*sin(theta);
% plot data on top of grid
h = pcolor(XX, YY, im);
shading flat
grid on;
axis equal;

%
if ~isempty(cLim)
    caxis(cLim);
end

end










