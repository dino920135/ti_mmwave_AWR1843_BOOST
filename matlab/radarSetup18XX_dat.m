function [ConfigParameters] = radarSetup18XX(configfile)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%         CONFIGURE SERIAL PORT          %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %%%% UART COM PORT:
% comPortString = 'COM4';
% UART_sphandle = serial(comPortString,'BaudRate',115200);
% set(UART_sphandle,'Parity','none')
% set(UART_sphandle,'Terminator','LF')
% fopen(UART_sphandle);
% 
% %%%% DATA COM PORT:
% comPortString = 'COM3';
% DATA_sphandle = serial(comPortString,'BaudRate',921600);
% set(DATA_sphandle,'Terminator', '');
% set(DATA_sphandle,'InputBufferSize', 6586);% 65536);
% set(DATA_sphandle,'Timeout',10);
% set(DATA_sphandle,'ErrorFcn',@dispError);
% set(DATA_sphandle,'BytesAvailableFcnMode','byte');
% set(DATA_sphandle,'BytesAvailableFcnCount', 2^16+1);%BYTES_AVAILABLE_FCN_CNT);
% set(DATA_sphandle,'BytesAvailableFcn',@readUartCallbackFcn);
% fopen(DATA_sphandle);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%        READ CONFIGURATION FILE         %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%configfile = "xwr18xx_profile_2021_04_13T11_35_14_057.cfg";
config = cell(1,100);
fid = fopen(configfile, 'r');
if fid == -1
    fprintf('File %s not found!\n', configfile);
    return;
else
    fprintf('Opening configuration file %s ...\n', configfile);
end
tline = fgetl(fid);
k=1;
while ischar(tline)
    config{k} = tline;
    tline = fgetl(fid);
    k = k + 1;
end
config = config(1:k-1);
fclose(fid);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%&&&&%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%       PARSE THE CONFIGURATION FILE         %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%&&&&%%%%%%%%%%%%%%%%%%%%%%%%%

for i = 1:length(config)
    configLine = strsplit(config{i});
    
    % We are only interested in the channelCfg, profileCfg and frameCfg parameters:
    if strcmp(configLine{1},'channelCfg')
        ConfigParameters.txChannelEn = str2double(configLine{3});
        
        ConfigParameters.numTxAzimAnt = bitand(bitshift(ConfigParameters.txChannelEn,0),1) +...
                bitand(bitshift(ConfigParameters.txChannelEn,-2),1);
        ConfigParameters.numTxElevAnt = bitand(bitshift(ConfigParameters.txChannelEn,-1),1);
                
        ConfigParameters.rxChannelEn = str2double(configLine{2});
        ConfigParameters.numRxAnt = bitand(bitshift(ConfigParameters.rxChannelEn,0),1) +...
            bitand(bitshift(ConfigParameters.rxChannelEn,-1),1) +...
            bitand(bitshift(ConfigParameters.rxChannelEn,-2),1) +...
            bitand(bitshift(ConfigParameters.rxChannelEn,-3),1);
        ConfigParameters.numTxAnt = ConfigParameters.numTxElevAnt + ConfigParameters.numTxAzimAnt;
        
    elseif  strcmp(configLine{1},'profileCfg')
        ConfigParameters.startFreq = str2double(configLine{3});
        ConfigParameters.idleTime =  str2double(configLine{4});
        ConfigParameters.rampEndTime = str2double(configLine{6});
        ConfigParameters.freqSlopeConst = str2double(configLine{9});
        ConfigParameters.numAdcSamples = str2double(configLine{11});
        ConfigParameters.numAdcSamplesRoundTo2 = 1;
        while ConfigParameters.numAdcSamples > ConfigParameters.numAdcSamplesRoundTo2
            ConfigParameters.numAdcSamplesRoundTo2 = ConfigParameters.numAdcSamplesRoundTo2 * 2;
        end 
        ConfigParameters.digOutSampleRate = str2double(configLine{12}); %uints: ksps
        
    elseif strcmp(configLine{1},'frameCfg')
        ConfigParameters.chirpStartIdx = str2double(configLine{2});
        ConfigParameters.chirpEndIdx = str2double(configLine{3});
        ConfigParameters.numLoops = str2double(configLine{4});
        ConfigParameters.numFrames = str2double(configLine{5});
        ConfigParameters.framePeriodicity = str2double(configLine{6});
    end
end  


ConfigParameters.numChirpsPerFrame = (ConfigParameters.chirpEndIdx -...
    ConfigParameters.chirpStartIdx + 1) *...
   ConfigParameters.numLoops;
ConfigParameters.numDopplerBins = ConfigParameters.numChirpsPerFrame / ConfigParameters.numTxAnt;
ConfigParameters.numRangeBins = ConfigParameters.numAdcSamplesRoundTo2;
ConfigParameters.rangeResolutionMeters = 3e8 * ConfigParameters.digOutSampleRate * 1e3 /...
    (2 * ConfigParameters.freqSlopeConst * 1e12 * ConfigParameters.numAdcSamples);
ConfigParameters.rangeIdxToMeters = 3e8 * ConfigParameters.digOutSampleRate * 1e3 /...
    (2 * ConfigParameters.freqSlopeConst * 1e12 * ConfigParameters.numRangeBins);
ConfigParameters.dopplerResolutionMps = 3e8 / (2*ConfigParameters.startFreq*1e9 *...
    (ConfigParameters.idleTime + ConfigParameters.rampEndTime) *...
    1e-6 * ConfigParameters.numDopplerBins * ConfigParameters.numTxAnt);
ConfigParameters.maxRange = 300 * 0.9 * ConfigParameters.digOutSampleRate /(2 * ConfigParameters.freqSlopeConst * 1e3);
ConfigParameters.maxVelocity = 3e8 / (4*ConfigParameters.startFreq*1e9 *(ConfigParameters.idleTime + ConfigParameters.rampEndTime) * 1e-6 * ConfigParameters.numTxAnt);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%        SEND CONFIGURATION TO SENSOR         %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% mmwDemoCliPrompt = char('mmwDemo:/>');
% 
% %Send CLI configuration to IWR14xx
% fprintf('Sending configuration from %s file to Radar ...\n', configfile);
% for k=1:length(config)
%     command = config{k};
%     fprintf(UART_sphandle, command);
%     fprintf('%s\n', command);
%     echo = fgetl(UART_sphandle); % Get an echo of a command
%     done = fgetl(UART_sphandle); % Get "Done"
%     prompt = fread(UART_sphandle, size(mmwDemoCliPrompt,2)); % Get the prompt back
% end


end