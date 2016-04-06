function uploadBinary(BinaryFileName,SerialPortName)
%UPLOADBINARY  Upload binary file to OpenCM9.04 microcontroller over USB
%
%   UPLOADBINARY(BINARYFILENAME) uploads the .bin binary file, BINARYFILENAME,
%   to OpenCM9.04 microcontroller using the default USB serial port ('COM1' on
%   Windows, '/dev/tty.usbmodem1411' on OS X).
%   
%   UPLOADBINARY(BINARYFILENAME, SERIALPORTNAME) optionally specifies USB the
%   serial port name, SERIALPORTNAME, as a string.

%   Andrew D. Horchler, adh9 @ case . edu
%   Created: 7-10-15, Revision: 1.1, 2-25-16
    

if nargin < 1
    error('uploadBinary:TooFewInputs','Too few inputs.');
end
if ~ischar(BinaryFileName)
    error('uploadBinary:InvalidBinaryFileName','Invalid Binary file name.');
end
if nargin > 1
    if ~ischar(SerialPortName)
        error('uploadBinary:InvalidSerialPortName','Invalid Serial port name.');
    end
else
    SerialPortName = '';
end

BaudRate = struct('BAUD_9600',    9600,...
                  'BAUD_19200',  19200,...
                  'BAUD_57600',  57600,...
                  'BAUD_115200',115200,...
                  'BAUD_230400',230400,...
                  'BAUD_460800',460800,...
                  'BAUD_921600',921600);

SerialBaudRate = BaudRate.BAUD_115200;
        
SerialOutputBufferSize = 2048;
SerialTimeout = 3;

SerialObject = [];
SerialOpened = false;

BinaryData = [];
BinaryLength = NaN;
BinaryChecksum = NaN;

Verbosity = 2;

% Open port, reset board, upload new binary file, and close port
open();
verbosePrint({'', 'Uploading new binary file to board.\n'});
uploadBinary();
removePort();


function open()
%OPEN  Create and open serial connection to OpenCM9.04.

    if ~SerialOpened
        if isempty(SerialPortName)
            if ismac
                SerialPortName = '/dev/tty.usbmodem1411';
            else
                SerialPortName = 'COM1';
            end
        end
        
        % Check if port is used, safely close and delete any connections
        s = instrfind('Type', 'serial');
        if ~isempty(s)
            for p = s(strcmp(s.Port, SerialPortName))
                if strcmp(p.Status, 'open')
                    fclose(p);
                end
                delete(p);
            end
        end
        
        if ~isValidBaudRate(SerialBaudRate)
            error('uploadBin:InvalidBaudRate','Invalid baud rate.');
        end
        
        % Create serial port object
        SerialObject = serial(SerialPortName, 'BaudRate', SerialBaudRate, ...
                                              'OutputBufferSize', SerialOutputBufferSize, ...
                                              'Timeout', SerialTimeout);
        SerialOpened = true;
    end
    
    % Open serial port
    openPort();
end

function tf = isValidBaudRate(baud)
%ISVALIDBAUDRATE  

    if ischar(baud)
        tf = isfield(BaudRate, baud);
    elseif isnumeric(baud) && isscalar(baud)
        tf = any(structfun(@(s)s==baud, BaudRate));
    end
end

function openPort()
%OPENPORT  

    if ~isOpen()
        verbosePrint({'', 'Connecting... '});
        try
            fopen(SerialObject);
            verbosePrint({'', 'Connected.\n'});
        catch ME
            verbosePrint('Connection failed.\n');
            removePort();
            rethrow(ME);
        end
    end
end

function tf = isOpen()
%ISOPEN  

    tf = ~isempty(SerialObject) && ...
          isvalid(SerialObject) && ...
          strcmp(SerialObject.Status, 'open');
end

function verbosePrint(str)
%VERBOSEPRINT  

    if Verbosity > 0
        if iscell(str)
            str = str{min(numel(str), Verbosity)};
        end
        if ~isempty(str)
            s = strsplit(str, '\\n');
            s = sprintf('%s\n', s{:});
            fprintf(1, s(1:end-1));
        end
    end
end

function removePort()
%REMOVEPORT  

    if ~isempty(SerialObject) && isvalid(SerialObject)
        verbosePrint({'', 'Closing serial connection... '});
        if isOpen()
            fclose(SerialObject);
        end
        delete(SerialObject);
        SerialObject = [];
        SerialOpened = false;
        verbosePrint({'', 'Closed. Serial port removed.\n'});
    end
end

function uploadBinary()
%UPLOADBINARY  

    readBinary(BinaryFileName);
    try
        wasOpen = isOpen();
        if wasOpen
            fclose(SerialObject);
        end
        
        fopen(SerialObject);
        fwrite(SerialObject, 'CM9X');
        fclose(SerialObject);
        pause(0.85);
        
        fopen(SerialObject);
        fwrite(SerialObject, 'AT&LD');
        
        while true
            tline = strtrim(fgetl(SerialObject));
            if strcmp(tline, 'Ready..')
                verbosePrint('Uploading binary file..');
                bData = BinaryData;
                while numel(bData) >= SerialOutputBufferSize
                    verbosePrint('.');
                    fwrite(SerialObject, bData(1:SerialOutputBufferSize));
                    bData = bData(SerialOutputBufferSize+1:end);
                end
                verbosePrint('.');
                fwrite(SerialObject, bData);
                fwrite(SerialObject, BinaryChecksum);
                verbosePrint([' ' int2str(BinaryLength) ' bytes uploaded.\n']);
            else
                if strcmp(tline, 'Success..')
                    verbosePrint({'', 'Upload succeeded. Resetting board... '});
                else
                    verbosePrint({'', 'Upload failed. Resetting board... '});
                end
                pause(0.1);
                fwrite(SerialObject, 'AT&RST');
                fclose(SerialObject);
                pause(0.85);
                verbosePrint({'', 'Reset.\n'});
                break;
            end
        end
        
        if wasOpen
            fopen(SerialObject);
        end
    catch ME
        verbosePrint('Uploading failed.\n');
        removePort();
        rethrow(ME);
    end
end

function readBinary(filename)
%READBINARY  
    
    fid = fopen(filename, 'r');
    if fid > 0
        try
            [BinaryData, BinaryLength] = fread(fid, 'uint8=>uint8');
        catch ME
            verbosePrint('Could not read binary file.\n');
            fclose(fid);
            rethrow(ME);
        end
        BinaryChecksum = mod(sum(BinaryData), 256);
    else
        verbosePrint('Could not open binary file.\n');
    end
end

end