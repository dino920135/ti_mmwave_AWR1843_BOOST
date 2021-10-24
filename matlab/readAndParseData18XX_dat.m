function [dataOk, frameNumber, detObj] = readAndParseData18XX(DATA_sphandle, ConfigParameters)
    OBJ_STRUCT_SIZE_BYTES = 16;
    BYTE_VEC_ACC_MAX_SIZE = 2^16;
    MMWDEMO_UART_MSG_DETECTED_POINTS = 1;
    MMWDEMO_UART_MSG_RANGE_PROFILE   = 2;
    MMWDEMO_UART_MSG_NOISE_PROFILE   = 3;
    MMWDEMO_UART_MSG_RANGE_AZIMUTH_HEATMAP = 4;
    MMWDEMO_UART_MSG_RANGE_DOPPLER_HEATMAP = 5;
    MMWDEMO_UART_MSG_DETECTED_POINTS_SIDE_INFO  = 7;
    maxBufferSize = BYTE_VEC_ACC_MAX_SIZE;
    NUM_ANGLE_BINS=64;
    detObj = [];
    frameNumber = 0;
      
    persistent byteBuffer
    if isempty(byteBuffer)
        byteBuffer = zeros(maxBufferSize,1);
    end
    
    persistent byteBufferLength
    if isempty(byteBufferLength)
        byteBufferLength = 0;
    end
    
    persistent magiNotOkCounter
    if isempty(magiNotOkCounter)
        magiNotOkCounter = 0;
    end
    
    magicOk = 0;
    dataOk = 0;
    
%     bytesToRead = fread(DATA_sphandle,'BytesAvailable');
    if (DATA_sphandle ~= -1)
        % Read the Data Serial Port
        [bytevec, byteCount] = fread(DATA_sphandle, 16544, 'uint8');
        
         % Check if the buffer is not full, and then add the data to the buffer:
        if(byteBufferLength + byteCount < maxBufferSize)
            byteBuffer(byteBufferLength+1:byteBufferLength + byteCount) = bytevec(1:byteCount);
            byteBufferLength = byteBufferLength + byteCount;
        end
        disp([num2str(byteBufferLength) ' ' num2str(byteCount)])
        
    end
%     if byteBufferLength == 0
%         disp(byteBufferLength)
%     end
    
    % Check that the buffer is not empty:
    if byteBufferLength > 16
        byteBufferStr = char(byteBuffer);
        
        % Search for the magic number inside the buffer and check that at least one magic number has been found:
        startIdx = strfind(byteBufferStr', char([2 1 4 3 6 5 8 7]));
        if ~isempty(startIdx)
            
            % Check the position of the first magic number and put it at
            % the beginning of the buffer
            if length(startIdx) >= 2
                if startIdx(end-1) > 1
                    byteBuffer(1:byteBufferLength-(startIdx(1)-1)) = byteBuffer(startIdx(1):byteBufferLength);
                    byteBufferLength = byteBufferLength - (startIdx(1)-1);
                end
            else
                if startIdx(1) > 1
                    byteBuffer(1:byteBufferLength-(startIdx(1)-1)) = byteBuffer(startIdx(1):byteBufferLength);
                    byteBufferLength = byteBufferLength - (startIdx(1)-1);
                end
            end
            if byteBufferLength < 0
                byteBufferLength = 0;
            end
            
            totalPacketLen = sum(byteBuffer(8+4+[1:4]) .* [1 256 65536 16777216]');
            if ((byteBufferLength >= totalPacketLen) && (byteBufferLength ~= 0)) 
                magicOk = 1;
            else
                magicOk = 0;
            end
        end
    end
%     disp(magicOk)
    if (magicOk == 1)
        %%%%% HEADER
        word = [1 256 65536 16777216]';
        idx = 0;
        magicNumber = byteBuffer(idx + 1:8);
        idx = idx + 8;
        Header.version = dec2hex(sum(byteBuffer(idx+[1:4]) .* word));
        idx = idx + 4;
        Header.totalPacketLen = sum(byteBuffer(idx+[1:4]) .* word);
        idx = idx + 4;
        Header.platform = dec2hex(sum(byteBuffer(idx+[1:4]) .* word));
        idx = idx + 4;
        Header.frameNumber = sum(byteBuffer(idx+[1:4]) .* word);
        frameNumber = Header.frameNumber;
        idx = idx + 4;
        Header.timeCpuCycles = sum(byteBuffer(idx+[1:4]) .* word);
        idx = idx + 4;
        Header.numDetectedObj = sum(byteBuffer(idx+[1:4]) .* word);
        idx = idx + 4;
        Header.numTLVs = sum(byteBuffer(idx+[1:4]) .* word);
        idx = idx + 4;
        Header.subFrameNumber = sum(byteBuffer(idx+[1:4]) .* word);
        idx = idx + 4;
        
        
        %%%%% TLV
        
        % Analyze each of TLV messages:
        for tlvIdx = 1:Header.numTLVs
            word = [1 256 65536 16777216]';
            % First, analyze the TLV header (TLV type and length):
            tlv.type = sum(byteBuffer(idx+(1:4)) .* word);
            disp(['tlv.type: ' num2str(tlv.type)])
            idx = idx + 4;
            tlv.length = sum(byteBuffer(idx+(1:4)) .* word);
            idx = idx + 4;
            % Check that the TLV message is of the right type (Detected objects):
            switch tlv.type
                case MMWDEMO_UART_MSG_DETECTED_POINTS
                    detObj =[];
                    if tlv.length > 0   
                        % Extract the raw data for all the detected points
                        bytes = byteBuffer(idx+(1:Header.numDetectedObj*OBJ_STRUCT_SIZE_BYTES));
                        idx = idx + Header.numDetectedObj*OBJ_STRUCT_SIZE_BYTES;
                        
                        % Reshape the array to have the data for each point
                        % (X,Y,Z,doppler) in each column
                        bytes = reshape(bytes, OBJ_STRUCT_SIZE_BYTES, Header.numDetectedObj);
                        
                        % Convert the byte matrix to float data
                        floatData = reshape(typecast(reshape(uint8(bytes), 1, []), 'single'),4,Header.numDetectedObj);
                      
                        detObj.numObj = Header.numDetectedObj;
                        detObj.x = floatData(1,:);
                        detObj.y = floatData(2,:);
                        detObj.z = floatData(3,:);
                        detObj.doppler = floatData(4,:); 
                    end
                case MMWDEMO_UART_MSG_DETECTED_POINTS_SIDE_INFO
                    
                    if tlv.length > 0   
                        bytes = byteBuffer(idx+(1:Header.numDetectedObj*4));
                        idx = idx + Header.numDetectedObj*4;
                        
                        % Reshape the array to have the data for each point
                        % (snr,noise) in each column
                        bytes = reshape(bytes, 4, Header.numDetectedObj);
                        
                        % Convert the byte matrix to float data
                        floatData = reshape(typecast(reshape(uint8(bytes), 1, []), 'int16'),2,Header.numDetectedObj);
                        detObj.snr = floatData(1,:);
                        detObj.noise = floatData(2,:);
                        dataOk = 1;
                    end
                case MMWDEMO_UART_MSG_RANGE_PROFILE
                    rp = byteBuffer(idx+(1:tlv.length));
                    idx = idx + tlv.length;
                    rp=rp(1:2:end)+rp(2:2:end)*256;
                    dataOk = 1;

                    
                case MMWDEMO_UART_MSG_RANGE_AZIMUTH_HEATMAP
                    numBytes = 2 * 4 * ConfigParameters.numRangeBins * 4;               
                    q = byteBuffer(idx+(1:numBytes));
                    idx = idx + numBytes;
                    q = q(1:2:end)+q(2:2:end)*2^8;
                    q(q>32767) = q(q>32767) - 65536;
                    q = q(1:2:end)+ 1j*q(2:2:end);
                    q = reshape(q, 2*4, ConfigParameters.numRangeBins);
                    Q = fft(q, NUM_ANGLE_BINS);  % column based NUM_ANGLE_BINS-point fft, padded with zeros
                    QQ=fftshift(abs(Q),1);
                    QQ=QQ.';
                    QQ=QQ(:,1:end);
                    detObj.QQ=fliplr(QQ);
%                     detObj.ele = 
%                     detObj.theta = asind((-NUM_ANGLE_BINS/2+0.5 : NUM_ANGLE_BINS/2-0.5)'*(2/NUM_ANGLE_BINS));
%                     detObj.range = (0:ConfigParameters.numRangeBins-1) * ConfigParameters.rangeIdxToMeters;
                    dataOk = 1;
                    
                case MMWDEMO_UART_MSG_RANGE_DOPPLER_HEATMAP
                    
            end
            
        end
        %Remove processed data
        if idx > 0
            shiftSize = Header.totalPacketLen;
            byteBuffer(1: byteBufferLength-shiftSize) = byteBuffer(shiftSize+1:byteBufferLength);
            byteBufferLength = byteBufferLength - shiftSize;
            if byteBufferLength < 0
                %             fprintf('Error: bytevec_cp_len < bytevecAccLen, %d %d \n', bytevec_cp_len, bytevecAccLen)
                byteBufferLength = 0;
            end
        end
%         if byteBufferLength > (byteBufferLength * 7/8)
%             byteBufferLength = 0;
%         end
        
    else
        magiNotOkCounter = magiNotOkCounter + 1;
    end
    
                        
                    
