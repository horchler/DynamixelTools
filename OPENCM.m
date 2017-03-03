classdef OPENCM < handle
    %OPENCM  Create OPENCM object to communicate with OpenCM9.04.
    %   
    %   OBJ = OPENCM() constructs an OPENCM class object with default
    %   parameters. The default USB port name is 'COM1' on Windows and
    %   '/dev/tty.usbmodem1411' on OS X. The default USB baud rate is 9600 Mbps.
    %   
    %   OBJ = OPENCM(OPTION1,...) specifies optional serial connection
    %   parameters, constructs an OPENCM class object, opens a serial connection
    %   to a OpenCM9.04 microcontroller over USB, and begins communication with
    %   the microcontroller. The name of the USB serial port can be specifified
    %   as a string and the USB baud rate can be specified as a scalar value
    %   (see BaudRate property for list of valid rates).
    %
    %   Methods:
    %       open           - Create and open serial connection to OpenCM9.04.
    %       isOpen         - True if serial connection to OpenCm9.04 is open. 
    %       begin          - Begin communication to and from Dynamixels.
    %       pause          - Pause communication to and from Dynamixels.
    %       micros         - Get time in microseconds from OpenCM9.04.
    %       millis         - Get time in milliseconds from OpenCM9.04.
    %       status         - Get state of tosser code running on OpenCM9.04.
    %       baud           - Get baud rate between OpenCM9.04 and Dynamixels.
    %       version        - Get version of tosser code running on OpenCM9.04.
    %       getLED         - Get state of board LED on OpenCM9.04.
    %       setLED         - Set state of board LED on OpenCM9.04.
    %       blink          - Enable blinking of board LED on OpenCM9.04.
    %       reset          - Reset state of tosser code running on OpenCM9.04.
    %       exit           - Exit tosser code running on OpenCM9.04.
    %       close          - Safely close serial connection to OpenCM9.04.
    %       delete         - Close serial connection and delete OPENCM object.
    %       
    %   Properties (Read-only):
    %       INVALID_BYTE   - Value returned indicating invalid byte data.
    %       INVALID_WORD   - Value returned indicating invalid word data.
    %       BaudRate       - Structure of USB serial baud rates.
    %       BaudRateValue  - Structure of Dynamixel baud rate values.
    %       SerialPortName - Name of USB serial port, specified in constructor. 
    %       SerialBaudRate - USB serial baud rate, specified in constructor.
    %       
    %   Properties (Read-Write):
    %       Verbosity      - Message and warning verbosity level.
    %       
    %   See also:
    %       OPENCM/open, OPENCM/begin, OPENCM/close, OPENCM/delete
    
    
    %   Andrew D. Horchler, horchler @ gmail . com
    %   Created: 7-10-15, Revision: 1.0, 2-25-16
    %   
    %   Copyright (c) 2015-2017, Andrew D. Horchler
    %   All rights reserved.
    %   
    %   Redistribution and use in source and binary forms, with or without
    %   modification, are permitted provided that the following conditions are
    %   met:
    %     * Redistributions of source code must retain the above copyright
    %       notice, this list of conditions and the following disclaimer.
    %     * Redistributions in binary form must reproduce the above copyright
    %       notice, this list of conditions and the following disclaimer in the
    %       documentation and/or other materials provided with the distribution.
    %     * Neither the name of Case Western Reserve University nor the names of
    %       its contributors may be used to endorse or promote products derived
    %       from this software without specific prior written permission.
    %       
    %       THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    %       "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    %       LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    %       FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL ANDREW D.
    %       HORCHLER BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
    %       EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
    %       PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
    %       PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
    %       OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    %       (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
    %       USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
    %       DAMAGE.
    
    
    properties (Constant, Hidden)
        % Version number of tosser code that must be running on OpenCM9.04
        Version = 2;
    end
    
    properties (Constant)
        % Baud rates between computer and OpenCM9.04
        BaudRate = struct('BAUD_9600',    9600,...
                          'BAUD_19200',  19200,...
	                      'BAUD_57600',  57600,...
	                      'BAUD_115200',115200,...
	                      'BAUD_230400',230400,...
                          'BAUD_460800',460800,...
	                      'BAUD_921600',921600);
    end
    
    properties (Access=private, Constant, Hidden)
        PACKET_HEADER = uint8('QQ').';
        Command = struct('BEGIN',  uint8('A'),...
                         'PAUSE',  uint8('B'),...
                         'MICROS', uint8('C'),...
                         'MILLIS', uint8('D'),...
                         'STATUS', uint8('E'),...
                         'BAUD',   uint8('F'),...
                         'VERSION',uint8('G'),...
                         'GETLED', uint8('H'),...
                         'SETLED', uint8('I'),...
                         'BLINK',  uint8('J'),...
                         'RESET',  uint8('K'),...
                         'EXIT',   uint8('L'));
    end
    
    properties (Constant)
        % Baud rates values between OpenCM9.04 and Dynamixels
        BaudRateValue = struct('BAUD_9600',   uint8(207),...
                               'BAUD_19200',  uint8(103),...
	                           'BAUD_57600',   uint8(34),...
	                           'BAUD_115200',  uint8(16),...
	                           'BAUD_200000',   uint8(9),...
                               'BAUD_250000',   uint8(7),...
	                           'BAUD_400000',   uint8(4),...
	                           'BAUD_500000',   uint8(3),...
	                           'BAUD_1000000',  uint8(1),...
	                           'BAUD_2000000',  uint8(0),...
	                           'BAUD_2250000',uint8(250),...
	                           'BAUD_2500000',uint8(251),...
	                           'BAUD_3000000',uint8(252));
    end
    
    properties (Access=protected, Constant, Hidden)
        DEFAULT_BAUD_RATE_VALUE = OPENCM.BaudRateValue.BAUD_3000000;
        
        % Macros
        WORD2BYTES = @(a)typecast(uint16(a), 'uint8');
        LONG2BYTES = @(a)typecast(uint32(a), 'uint8');
        MAKEWORD = @(a,b)typecast(uint8(reshape([a(:) b(:)].',1,[])), 'uint16');
        MAKELONG = @(a,b,c,d)typecast(uint8(reshape([a(:) b(:) c(:) d(:)].',1,[])), 'uint32');
        CHECKSUM = @(b)bitcmp(mod(sum(b), 256), 'uint8');
        CHECKSUM_MASK = uint8(255);
        
        BYTE_LENGTH = uint8(1);
        WORD_LENGTH = uint8(2);
        LONG_LENGTH = uint8(4);
    end
    
    properties (Constant)
        LED_OFF = false;
        LED_ON = true;
        
        FAILURE = false;
        SUCCESS = true;
        
        % Packet properties
        INVALID_BYTE = intmax('uint8');
        INVALID_WORD = intmax('uint16');
        INVALID_LONG = intmax('uint32');
    end
    
    properties (SetAccess=protected)
        SerialPortName
        SerialBaudRate = OPENCM.BaudRate.BAUD_9600;
    end
    
    properties (Access=protected, Hidden)
        SerialOutputBufferSize = 2048;
        SerialTimeout = 1;
        
        TxBuffer
        RxBuffer
        RxLength
    end
    
    properties (Access=private, Hidden)
        SerialObject
        SerialOpened = false;
        
        BinaryData
        BinaryFileName = 'tosser.cpp.bin';
        BinaryLength
        BinaryChecksum
    end
    
    properties
        Verbosity = 2;
    end
    
    methods
        function OpenCM = OPENCM(varargin)
        %OPENCM  Create OPENCM class object and start serial connection.
        
            idx = 0;
            if nargin > 0 && ~isempty(varargin{1}) && isa(varargin{1}, 'OPENCM')
                % Copy object properties
                Obj = varargin{1};
                OpenCM.SerialPortName = Obj.SerialPortName;
                OpenCM.SerialBaudRate = Obj.SerialBaudRate;
                OpenCM.SerialOutputBufferSize = Obj.SerialOutputBufferSize;
                OpenCM.SerialTimeout = Obj.SerialTimeout;
                OpenCM.SerialObject = Obj.SerialObject;
                OpenCM.SerialOpened = Obj.SerialOpened;
                OpenCM.BinaryData = Obj.BinaryData;
                OpenCM.BinaryFileName = Obj.BinaryFileName;
                OpenCM.BinaryLength = Obj.BinaryLength;
                OpenCM.BinaryChecksum = Obj.BinaryChecksum;
                OpenCM.Verbosity = Obj.Verbosity;
                idx = 1;
            end
            for i = idx+1:nargin
                if ischar(varargin{i})
                    OpenCM.SerialPortName = varargin{i};
                elseif isnumeric(varargin{i}) && isscalar(varargin{i})
                    OpenCM.SerialBaudRate = varargin{i};
                end
            end
            if ~isempty(i)
                begin(OpenCM);
            end
        end
        
        function delete(OpenCM)
        %DELETE  Safely close serial connection and delete OPENCM object.
        %
        %   See also: OPENCM/close, OPENCM/open, OPENCM/isOpen, serial
        
            close(OpenCM);
        end
        
        function open(OpenCM,varargin)
        %OPEN  Create and open serial connection to OpenCM9.04.
        %
        %   See also: OPENCM/close, OPENCM/delete, OPENCM/isOpen, serial
        
            if ~OpenCM.SerialOpened
                for i = 1:nargin-1
                    if ischar(varargin{i})
                        OpenCM.SerialPortName = varargin{i};
                    elseif isValidBaudRate(OpenCM, varargin{i})
                        OpenCM.SerialBaudRate = varargin{i};
                    end
                end
                
                if isempty(OpenCM.SerialPortName)
                    if ismac
                        OpenCM.SerialPortName = '/dev/tty.usbmodem1411';
                    else
                        OpenCM.SerialPortName = 'COM1';
                    end
                end
                
                % Check if port is used, safely close and delete any connections
                s = instrfind('Type', 'serial');
                if ~isempty(s)
                    for p = s(strcmp(s.Port, OpenCM.SerialPortName))
                        if strcmp(p.Status, 'open')
                            fclose(p);
                        end
                        delete(p);
                    end
                end

                % Create serial port object
                OpenCM.SerialObject = serial(OpenCM.SerialPortName, 'BaudRate', OpenCM.SerialBaudRate, ...
                                                                    'OutputBufferSize', OpenCM.SerialOutputBufferSize, ...
                                                                    'Timeout', OpenCM.SerialTimeout);
                OpenCM.SerialOpened = true;
            end
            
            % Open serial port
        	openPort(OpenCM);
            
            OpenCM.TxBuffer = OpenCM.PACKET_HEADER;
            writeRaw(OpenCM);
        end
        
        function tf = isOpen(OpenCM)
        %ISOPEN  True if serial connection to OpenCm9.04 is valid and open.
        %
        %   See also: OPENCM/open, OPENCM/close, OPENCM/delete, serial
        
            tf = ~isempty(OpenCM.SerialObject) && ...
                  isvalid(OpenCM.SerialObject) && ...
                  strcmp(OpenCM.SerialObject.Status, 'open');
        end
        
        function close(OpenCM)
        %CLOSE  Safely close and remove serial connection to OpenCM9.04.
        %
        %   See also: OPENCM/delete, OPENCM/open, OPENCM/isOpen, serial
        
            if isOpen(OpenCM)
                OpenCM.pause();
            end
            removePort(OpenCM);
        end
        
        function connect(OpenCM)
        %CONNECT  Connect to tosser code runing on OpenCM9.04.
        %
        %   See also: OPENCM/open, OPENCM/begin, OPENCM/version,
        %             OPENCM/resetBoard, OPENCM/uploadBinary
        
            if ~isOpen(OpenCM)
                open(OpenCM);
            end
            VersionNumber = version(OpenCM);
            if VersionNumber ~= OpenCM.Version
                verbosePrint(OpenCM, {'', 'Board not responding. '});
                if VersionNumber == OpenCM.INVALID_BYTE;
                    resetBoard(OpenCM);
                    
                    if version(OpenCM) ~= OpenCM.Version
                        verbosePrint(OpenCM, {'', 'Uploading new binary file to board.\n'});
                        uploadBinary(OpenCM);
                    end
                else
                    verbosePrint(OpenCM, {'', 'Uploading new binary file to board.\n'});
                    uploadBinary(OpenCM);
                end
                if version(OpenCM) ~= OpenCM.Version
                    error('OPENCM:connect:UploadFailed', 'Upload failed.');
                end
            end
        end
        
        function begin(OpenCM,baud)
        %BEGIN  Begin communication to and from Dynamixels.
        %
        %   See also: OPENCM/open, OPENCM/connect, OPENCM/pause, OPENCM/status,
        %             OPENCM/version, OPENCM/reset, OPENCM/exit
        
            connect(OpenCM);
            if nargin < 2
                baud = OpenCM.DEFAULT_BAUD_RATE_VALUE;
            elseif isValidBaudRateValue(OpenCM, baud)
                if ischar(baud)
                    baud = OpenCM.BaudRateValue.(baud);
                elseif baud >= 9600
                    baud = OpenCM.BaudRateValue.(['BAUD_' int2str(baud)]);
                else
                    baud = uint8(baud);
                end
            else
                baud = OpenCM.INVALID_BYTE;
            end

            if baud ~= OpenCM.INVALID_BYTE
                b = [OpenCM.Command.BEGIN baud];
                OpenCM.TxBuffer = [OpenCM.PACKET_HEADER; b(:); OpenCM.CHECKSUM(b)];
                writeRaw(OpenCM);
                verbosePrint(OpenCM, {'', 'Started.\n'});
            else
                verbosePrint(OpenCM, 'Invalid baud rate value. Begin command failed.\n');
            end
        end
        
      	function pause(OpenCM)
        %PAUSE  Pause communication to and from Dynamixels.
        %
        %   See also: OPENCM/begin, OPENCM/status, OPENCM/version, OPENCM/reset,
        %             OPENCM/exit
        
            if isOpen(OpenCM)
                b = [OpenCM.Command.PAUSE OpenCM.Command.PAUSE];
                OpenCM.TxBuffer = [OpenCM.PACKET_HEADER; b(:); OpenCM.CHECKSUM(b)];
                writeRaw(OpenCM);
                verbosePrint(OpenCM, {'', 'Paused.\n'});
            else
                verbosePrint(OpenCM, 'Serial connection not open. Pause command failed.\n');
            end
        end
        
        function t = micros(OpenCM)
        %MICROS  Get time in microseconds from OpenCM9.04.
        %
        %   See also: OPENCM/begin, OPENCM/pause, OPENCM/millis, OPENCM/status,
        %             OPENCM/baud, OPENCM/version, OPENCM/reset, OPENCM/exit
        
            t = OpenCM.INVALID_LONG;
            if isOpen(OpenCM)
                b = [OpenCM.Command.MICROS OpenCM.Command.MICROS];
                OpenCM.TxBuffer = [OpenCM.PACKET_HEADER; b(:); OpenCM.CHECKSUM(b)];
                writeRaw(OpenCM);
                
                OpenCM.RxLength = 8;
                Count = readRaw(OpenCM);
                
                if isValidPacket(OpenCM, OpenCM.Command.MICROS, Count)
                    t = OpenCM.MAKELONG(OpenCM.RxBuffer(4), OpenCM.RxBuffer(5), OpenCM.RxBuffer(6), OpenCM.RxBuffer(7));
                else
                    verbosePrint(OpenCM, 'Invalid packet returned from Micros command.\n');
                end
            else
                verbosePrint(OpenCM, 'Serial connection not open. Micros command failed.\n');
            end
        end
        
        function t = millis(OpenCM)
        %MILLIS  Get time in milliseconds from OpenCM9.04.
        %
        %   See also: OPENCM/begin, OPENCM/pause, OPENCM/micros, OPENCM/status,
        %             OPENCM/baud, OPENCM/version, OPENCM/reset, OPENCM/exit
        
            t = OpenCM.INVALID_LONG;
            if isOpen(OpenCM)
                b = [OpenCM.Command.MILLIS OpenCM.Command.MILLIS];
                OpenCM.TxBuffer = [OpenCM.PACKET_HEADER; b(:); OpenCM.CHECKSUM(b)];
                writeRaw(OpenCM);
                
                OpenCM.RxLength = 8;
                Count = readRaw(OpenCM);
                
                if isValidPacket(OpenCM, OpenCM.Command.MILLIS, Count)
                    t = OpenCM.MAKELONG(OpenCM.RxBuffer(4), OpenCM.RxBuffer(5), OpenCM.RxBuffer(6), OpenCM.RxBuffer(7));
                else
                    verbosePrint(OpenCM, 'Invalid packet returned from Millis command.\n');
                end
            else
                verbosePrint(OpenCM, 'Serial connection not open. Millis command failed.\n');
            end
        end
        
        function st = status(OpenCM)
        %STATUS  Get state of tosser code running on OpenCM9.04.
        %
        %   See also: OPENCM/begin, OPENCM/pause, OPENCM/micros, OPENCM/millis,
        %             OPENCM/baud, OPENCM/version, OPENCM/reset, OPENCM/exit
        
            st = OpenCM.INVALID_BYTE;
            if isOpen(OpenCM)
                b = [OpenCM.Command.STATUS OpenCM.Command.STATUS];
                OpenCM.TxBuffer = [OpenCM.PACKET_HEADER; b(:); OpenCM.CHECKSUM(b)];
                writeRaw(OpenCM);
                
                OpenCM.RxLength = 5;
                Count = readRaw(OpenCM);
                
                if isValidPacket(OpenCM, OpenCM.Command.STATUS, Count)
                    st = OpenCM.RxBuffer(4);
                else
                    verbosePrint(OpenCM, 'Invalid packet returned from Status command.\n');
                end
            else
                verbosePrint(OpenCM, 'Serial connection not open. Status command failed.\n');
            end
        end
        
        function baudRateValue = baud(OpenCM)
        %BAUD  Get baud rate between OpenCM9.04 and Dynamixels.
        %
        %   See also: OPENCM/begin, OPENCM/pause, OPENCM/micros, OPENCM/millis,
        %             OPENCM/status, OPENCM/version, OPENCM/reset, OPENCM/exit
        
            baudRateValue = OpenCM.INVALID_BYTE;
            if isOpen(OpenCM)
                b = [OpenCM.Command.BAUD OpenCM.Command.BAUD];
                OpenCM.TxBuffer = [OpenCM.PACKET_HEADER; b(:); OpenCM.CHECKSUM(b)];
                writeRaw(OpenCM);
                
                OpenCM.RxLength = 5;
                Count = readRaw(OpenCM);
                
                if isValidPacket(OpenCM, OpenCM.Command.BAUD, Count)
                    baudRateValue = OpenCM.RxBuffer(4);
                else
                    verbosePrint(OpenCM, 'Invalid packet returned from Baud command.\n');
                end
            else
                verbosePrint(OpenCM, 'Serial connection not open. Baud command failed.\n');
            end
        end
        
        function ver = version(OpenCM)
        %VERSION  Get version of tosser code running on OpenCM9.04.
        %
        %   See also: OPENCM/begin, OPENCM/pause, OPENCM/micros, OPENCM/millis,
        %             OPENCM/status, OPENCM/baud, OPENCM/reset, OPENCM/exit
        
            ver = OpenCM.INVALID_BYTE;
            if isOpen(OpenCM)
                b = [OpenCM.Command.VERSION OpenCM.Command.VERSION];
                OpenCM.TxBuffer = [OpenCM.PACKET_HEADER; b(:); OpenCM.CHECKSUM(b)];
                writeRaw(OpenCM);
                
                OpenCM.RxLength = 5;
                Count = readRaw(OpenCM);
                
                if isValidPacket(OpenCM, OpenCM.Command.VERSION, Count)
                    ver = OpenCM.RxBuffer(4);
                elseif Count == 0
                    verbosePrint(OpenCM, {'', 'No packet returned from Version command.\n'});
                else
                    verbosePrint(OpenCM, 'Invalid packet returned from Version command.\n');
                end
            else
                verbosePrint(OpenCM, 'Serial connection not open. Version command failed.\n');
            end
        end
        
        function LEDState = getLED(OpenCM)
        %GETLED  Get state of board LED on OpenCM9.04.
        %
        %   See also: OPENCM/begin, OPENCM/pause, OPENCM/status, OPENCM/baud,
        %             OPENCM/version, OPENCM/setLED, OPENCM/blink, OPENCM/reset,
        %             OPENCM/exit
        
            LEDState = OpenCM.INVALID_BYTE;
            if isOpen(OpenCM)
                b = [OpenCM.Command.GETLED OpenCM.Command.GETLED];
                OpenCM.TxBuffer = [OpenCM.PACKET_HEADER; b(:); OpenCM.CHECKSUM(b)];
                writeRaw(OpenCM);
                
                OpenCM.RxLength = 5;
                Count = readRaw(OpenCM);
                
                if isValidPacket(OpenCM, OpenCM.Command.GETLED, Count)
                    LEDState = OpenCM.RxBuffer(4);
                else
                    verbosePrint(OpenCM, 'Invalid packet returned from GetLED command.\n');
                end
            else
                verbosePrint(OpenCM, 'Serial connection not open. GetLED command failed.\n');
            end
        end
        
        function setLED(OpenCM,LEDState)
        %SETLED  Set state of board LED on OpenCM9.04.
        %
        %   See also: OPENCM/begin, OPENCM/pause, OPENCM/status, OPENCM/baud,
        %             OPENCM/version, OPENCM/getLED, OPENCM/blink, OPENCM/reset,
        %             OPENCM/exit
        
            if isOpen(OpenCM)
                LEDState = ~~uint8(LEDState);
                b = [OpenCM.Command.SETLED LEDState];
                OpenCM.TxBuffer = [OpenCM.PACKET_HEADER; b(:); OpenCM.CHECKSUM(b)];
                writeRaw(OpenCM);
                if LEDState
                    verbosePrint(OpenCM, {'', 'LED state on.\n'});
                else
                    verbosePrint(OpenCM, {'', 'LED state off.\n'});
                end
            else
                verbosePrint(OpenCM, 'Serial connection not open. SetLED command failed.\n');
            end
        end
        
        function blink(OpenCM,MicroSecPeriod)
        %BLINK  Enable blinking of board LED on OpenCM9.04.
        %
        %   See also: OPENCM/begin, OPENCM/pause, OPENCM/status, OPENCM/baud,
        %             OPENCM/version, OPENCM/getLED, OPENCM/setLED,
        %             OPENCM/reset, OPENCM/exit
        
            if isOpen(OpenCM)
                if nargin > 1
                    if round(MicroSecPeriod) <= 0 || round(MicroSecPeriod) > flintmax || ~isfinite(MicroSecPeriod)
                        b = [OpenCM.Command.BLINK OpenCM.INVALID_BYTE];
                    else
                        b = [OpenCM.Command.BLINK OpenCM.LONG_LENGTH];
                    end
                else
                    b = [OpenCM.Command.BLINK OpenCM.Command.BLINK];
                end
                OpenCM.TxBuffer = [OpenCM.PACKET_HEADER; b(:); OpenCM.CHECKSUM(b)];
                writeRaw(OpenCM);
                if b(2) == OpenCM.LONG_LENGTH
                    b = [OpenCM.LONG_LENGTH OpenCM.LONG2BYTES(MicroSecPeriod)];
                    OpenCM.TxBuffer = [OpenCM.PACKET_HEADER; b(:); OpenCM.CHECKSUM(b)];
                    writeRaw(OpenCM);
                end
                verbosePrint(OpenCM, {'', 'Blink enabled.\n'});
            else
                verbosePrint(OpenCM, 'Serial connection not open. Blink command failed.\n');
            end
        end
        
        function reset(OpenCM)
        %RESET  Reset state of tosser code running on OpenCM9.04.
        %
        %   See also: OPENCM/begin, OPENCM/pause, OPENCM/status, OPENCM/baud,
        %             OPENCM/version, OPENCM/exit
        
            if isOpen(OpenCM)
                b = [OpenCM.Command.RESET OpenCM.Command.RESET];
                OpenCM.TxBuffer = [OpenCM.PACKET_HEADER; b(:); OpenCM.CHECKSUM(b)];
                writeRaw(OpenCM);
                verbosePrint(OpenCM, {'', 'Reset.\n'});
            else
                verbosePrint(OpenCM, 'Serial connection not open. Reset command failed.\n');
            end
        end
        
        function exit(OpenCM)
        %EXIT  Exit tosser code running on OpenCM9.04.
        %
        %   See also: OPENCM/begin, OPENCM/pause, OPENCM/status, OPENCM/baud,
        %             OPENCM/version, OPENCM/reset
        
            if isOpen(OpenCM)
                b = [OpenCM.Command.EXIT OpenCM.Command.EXIT];
                OpenCM.TxBuffer = [OpenCM.PACKET_HEADER; b(:); OpenCM.CHECKSUM(b)];
                writeRaw(OpenCM);
                verbosePrint(OpenCM, {'', 'Exited.\n'});
            else
                verbosePrint(OpenCM, 'Serial connection not open. Exit command failed.\n');
            end
        end
    end
    
    methods (Access=protected)
        function Count = readRaw(OpenCM)
        %READRAW  
        %
        %   See also: OPENCM/writeRaw, serial/fread
        
            if ~isempty(OpenCM.RxLength)
                [OpenCM.RxBuffer, Count, msg] = fread(OpenCM.SerialObject, OpenCM.RxLength, 'uint8');
            else
                [OpenCM.RxBuffer, Count, msg] = fread(OpenCM.SerialObject);
            end
            if ~isempty(msg)
                if strcmp(msg, 'The specified amount of data was not returned within the Timeout period.')
                    verbosePrint(OpenCM, {'', 'Serial connection timeout.\n'});
                else
                    warning('DXL:readRaw:Timeout', msg);
                end
            end
        end
        
        function writeRaw(OpenCM)
        %WRITERAW  
        %
        %   See also: OPENCM/readRaw, serial/fwrite
        
            fwrite(OpenCM.SerialObject, OpenCM.TxBuffer, 'uint8');
        end
    end
    
    methods (Access=private)
        function openPort(OpenCM)
        %OPENPORT  
        %
        %   See also: OPENCM/open, OPENCM/closePort, OPENCM/removePort,
        %             OPENCM/isOpen
        
            if ~isOpen(OpenCM)
                verbosePrint(OpenCM, {'', 'Connecting... '});
                try
                    fopen(OpenCM.SerialObject);
                    verbosePrint(OpenCM, {'', 'Connected.\n'});
                catch ME
                    verbosePrint(OpenCM, 'Connection failed.\n');
                    removePort(OpenCM);
                    rethrow(ME);
                end
            end
        end
        
        function closePort(OpenCM)
        %CLOSEPORT  
        %
        %   See also: OPENCM/open, OPENCM/openPort, OPENCM/removePort,
        %             OPENCM/isOpen
        
            if isOpen(OpenCM)
                verbosePrint(OpenCM, {'', 'Closing serial connection... '});
                fclose(OpenCM.SerialObject);
                verbosePrint(OpenCM, {'', 'Closed.\n'});
            end
        end
        
        function removePort(OpenCM)
        %REMOVEPORT  
        %
        %   See also: OPENCM/open, OPENCM/openPort, OPENCM/closePort,
        %             OPENCM/isOpen
        
            if ~isempty(OpenCM.SerialObject) && isvalid(OpenCM.SerialObject)
                verbosePrint(OpenCM, {'', 'Closing serial connection... '});
                if isOpen(OpenCM)
                    fclose(OpenCM.SerialObject);
                end
                delete(OpenCM.SerialObject);
                OpenCM.SerialObject = [];
                OpenCM.SerialOpened = false;
                verbosePrint(OpenCM, {'', 'Closed. Serial port removed.\n'});
            end
        end
        
        function resetBoard(OpenCM)
        %RESETBOARD  
        %
        %   See also: OPENCM/uploadBinary, OPENCM/open, OPENCM/begin
        
            verbosePrint(OpenCM, {'', 'Resetting board... '});
            try
                wasOpen = isOpen(OpenCM);
                if wasOpen
                    fclose(OpenCM.SerialObject);
                end

                fopen(OpenCM.SerialObject);
                fwrite(OpenCM.SerialObject, 'CM9X');
                fclose(OpenCM.SerialObject);
                pause(0.9);

                fopen(OpenCM.SerialObject);
                fwrite(OpenCM.SerialObject, 'AT&RST');
                fclose(OpenCM.SerialObject);
                pause(0.85);
                
                if wasOpen
                    fopen(OpenCM.SerialObject);
                end
            catch ME
                verbosePrint(OpenCM, 'Reset failed.\n');
                removePort(OpenCM);
                rethrow(ME);
            end
            verbosePrint(OpenCM, {'', 'Board reset.\n'});
        end
        
        function readBinary(OpenCM,filename)
        %READBINARY  
        %
        %   See also: OPENCM/uploadBinary, OPENCM/open, OPENCM/begin
        
            if nargin < 2
                filename = OpenCM.BinaryFileName;
            end
            fid = fopen(filename, 'r');
            if fid > 0
                try
                    [OpenCM.BinaryData, OpenCM.BinaryLength] = fread(fid, 'uint8=>uint8');
                catch ME
                    verbosePrint(OpenCM, 'Could not read binary file.\n');
                    fclose(fid);
                    rethrow(ME);
                end
                OpenCM.BinaryChecksum = mod(sum(OpenCM.BinaryData), 256);
            else
                verbosePrint(OpenCM, 'Could not open binary file.\n');
            end
        end
        
        function uploadBinary(OpenCM)
        %UPLOADBINARY  
        %
        %   See also: OPENCM/readBinary, OPENCM/resetBoard, OPENCM/open
        %             OPENCM/begin
        
            readBinary(OpenCM);
            try
                wasOpen = isOpen(OpenCM);
                if wasOpen
                    fclose(OpenCM.SerialObject);
                end
                OpenCM.SerialObject.Timeout = 3;
                
              	fopen(OpenCM.SerialObject);
                fwrite(OpenCM.SerialObject, 'CM9X');
                fclose(OpenCM.SerialObject);
                pause(0.85);
                
                fopen(OpenCM.SerialObject);
                fwrite(OpenCM.SerialObject, 'AT&LD');
                
                while true
                    tline = strtrim(fgetl(OpenCM.SerialObject));
                    if strcmp(tline, 'Ready..')
                        verbosePrint(OpenCM, 'Uploading binary file..');
                        bData = OpenCM.BinaryData;
                        while numel(bData) >= OpenCM.SerialOutputBufferSize
                            verbosePrint(OpenCM, '.');
                            fwrite(OpenCM.SerialObject, bData(1:OpenCM.SerialOutputBufferSize));
                            bData = bData(OpenCM.SerialOutputBufferSize+1:end);
                        end
                        verbosePrint(OpenCM, '.');
                        fwrite(OpenCM.SerialObject, bData);
                        fwrite(OpenCM.SerialObject, OpenCM.BinaryChecksum);
                        verbosePrint(OpenCM, [' ' int2str(OpenCM.BinaryLength) ' bytes uploaded.\n']);
                    else
                        if strcmp(tline, 'Success..')
                            verbosePrint(OpenCM, {'', 'Upload succeeded. Resetting board... '});
                        else
                            verbosePrint(OpenCM, {'', 'Upload failed. Resetting board... '});
                        end
                        pause(0.1);
                        fwrite(OpenCM.SerialObject, 'AT&RST');
                        fclose(OpenCM.SerialObject);
                        pause(0.85);
                        verbosePrint(OpenCM, {'', 'Reset.\n'});
                        break;
                    end
                end
                OpenCM.SerialObject.Timeout = OpenCM.SerialTimeout;
                
                if wasOpen
                    fopen(OpenCM.SerialObject);
                end
            catch ME
                verbosePrint(OpenCM, 'Uploading failed.\n');
                removePort(OpenCM);
                rethrow(ME);
            end
        end
        
        function out = verbosePrint(OpenCM, str)
        %VERBOSEPRINT  
        
            if OpenCM.Verbosity > 0
                if iscell(str)
                    str = str{min(numel(str), OpenCM.Verbosity)};
                end
                if ~isempty(str)
                    s = strsplit(str, '\\n');
                    s = sprintf('%s\n', s{:});
                    if nargout > 0
                        out = sprintf(s(1:end-1));
                    else
                        fprintf(1, s(1:end-1));
                    end
                end
            end
        end
        
        function tf = isValidPacket(OpenCM,Command,Count)
        %ISVALIDPACKET  
        
            tf = Count == OpenCM.RxLength && ...
                 all(OpenCM.RxBuffer(1:3) == [OpenCM.PACKET_HEADER; Command]) && ...
                 bitand(sum(OpenCM.RxBuffer(3:end)), double(OpenCM.CHECKSUM_MASK)) == OpenCM.CHECKSUM_MASK;
        end
        
        function tf = isValidBaudRate(OpenCM,baud)
        %ISVALIDBAUDRATE  
        %
        %   See also OPENCM/isValidBaudRateValue
        
            if ischar(baud)
                tf = isfield(OpenCM.BaudRate, baud);
            elseif isnumeric(baud) && isscalar(baud)
                tf = any(structfun(@(s)s==baud, OpenCM.BaudRate));
            end
        end
        
        function tf = isValidBaudRateValue(OpenCM,baud)
        %ISVALIDBAUDRATEVALUE  
        %
        %   See also OPENCM/isValidBaudRate
        
            if ischar(baud)
                tf = isfield(OpenCM.BaudRateValue, baud);
            elseif isnumeric(baud) && isscalar(baud)
                if baud >= 9600
                    tf = isfield(OpenCM.BaudRateValue, ['BAUD_' int2str(baud)]);
                else
                    tf = any(structfun(@(s)s==baud, OpenCM.BaudRateValue));
                end
            else
                tf = false;
            end
        end
    end
end