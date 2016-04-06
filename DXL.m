classdef DXL < OPENCM
    %DXL  Create DXL object to communicate with OpenCM9.04 and Dynamixels
    %   
    %   OBJ = DXL() constructs a DXL class object with default parameters. The
    %   default USB port name is 'COM1' on Windows and '/dev/tty.usbmodem1411'
    %   on OS X. The default USB baud rate is 9600 Mbps.
    %   
    %   OBJ = DXL(OPTION1,...) specifies optional serial connection parameters,
    %   constructs a DXL class object, opens a serial connection to an
    %   OpenCM9.04 microcontroller over USB, and begins communication with the
    %   microcontroller and any connected Dynamixel actuators. The name of the
    %   USB serial port can be specifified as a string and the USB baud rate can
    %   be specified as a scalar value (see BaudRate property for list of valid
    %   rates).
    %   
    %   Methods:
    %       open           - Create and open serial connection to OpenCM9.04.
    %       begin          - Begin communication to OpenCM9.04 and Dynamixels.
    %       ping           - Check for existence of Dynamixel actuators.
    %       readByte       - Read byte values from Dynamixel addresses.
    %       writeByte      - Write byte values to Dynamixel addresses.
    %       readWord       - Read word values from Dynamixel addresses.
    %       writeWord      - Write word values to Dynamixel addresses.
    %       close          - Safely close serial connection to OpenCM9.04.
    %       delete         - Close serial connection and delete DXL object.
    %       
    %   Properties (Read-only):
    %       AXModels       - Structure of AX Series Dynamixel model numbers.
    %       AXProperties   - Structure of AX Series-specific properties.
    %       MXModels       - Structure of MX Series Dynamixel model numbers.
    %       MXProperties   - Structure of MX Series-specific properties.
    %       Address        - Structure of Dynamixel addresses.
    %       NUM_ADDRESS    - Number of Dynamixel addresses.
    %       BROADCAST_ID   - Broacast ID value.
    %       MAX_ID         - Largest valid Actuator ID.
    %       Properties     - Structure of general Dynamixel properties.
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
    %       DXL/open, DXL/writeByte, DXL/readByte, DXL/writeWord, DXL/readWord,
    %       DXL/begin, DXL/ping, DXL/close, DXL/delete
    
    %   Andrew D. Horchler, adh9 @ case . edu
    %   Created: 6-28-15, Revision: 1.0, 8-4-15
    %   
    %   Copyright (c) 2015, Andrew D. Horchler
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
    

    properties (Constant)
        % AX Series model numbers
        AXModels = struct('AX12',  uint16(12),...
                          'AX18',  uint16(18),...
                          'AX12W',uint16(300));
        
        % AX Series limiting values and conversions
        AXProperties = struct('NUM_ADDRESS',uint8(49),...
                              ...
                              'DEFAULT_BAUD_RATE_VALUE',DXL.BaudRateValue.BAUD_1000000,...
                              'MAX_BAUD_RATE_VALUE',    DXL.BaudRateValue.BAUD_1000000,...
                              ...
                              'MIN_COMPLIANCE_SLOPE',  uint8(2),...
                              'MAX_COMPLIANCE_SLOPE',uint8(128),...
                              ...
                              'POSITION_RESOLUTION',       uint16(1023),...
                              'MAX_MOVING_SPEED_MAGNITUDE',uint16(1023),...
                              ...
                              'COMPLIANCE_SLOPE',@(x)bitshift(1,uint8(log2(max(x, uint8(2))))),...
                              ...
                              'DEGREES_PER_TICK', 360/1023,...
                              'RADIANS_PER_TICK',pi*2/1023,...
                              ...
                              'DEGREES_PER_SECOND_PER_TICK',    2/3,...
                              'RADIANS_PER_SECOND_PER_TICK',pi/270);
        
        % MX Series model numbers
        MXModels = struct('MX12W',uint16(360),...
                          'MX28',  uint16(29),...
                          'MX64', uint16(310),...
                          'MX106',uint16(320));
        
        % MX Series limiting values and conversions
        MXProperties = struct('NUM_ADDRESS',DXL.NUM_ADDRESS,...
                              ...
                              'DEFAULT_BAUD_RATE_VALUE',DXL.BaudRateValue.BAUD_57600,...
                              'MAX_BAUD_RATE_VALUE',  DXL.BaudRateValue.BAUD_3000000,...
                              ...
                              'MAX_MULTI_TURN_OFFSET_MAGNITUDE',uint16(24576),...
                              'MIN_RESOLUTION_DIVIDER',              uint8(1),...
                              'MAX_RESOLUTION_DIVIDER',              uint8(4),...
                              ...
                              'MAX_GAIN',uint8(254),...
                              ...
                              'POSITION_RESOLUTION',       uint16(4095),...
                              'MAX_MOVING_SPEED_MAGNITUDE',uint16(1023),...
                              ...
                              'MAX_CURRENT',    uint16(65535),...
                              'MAX_GOAL_TORQUE', uint16(2047),...
                              'MAX_ACCELERATION',  uint8(254),...
                              ...
                              'DEGREES_PER_TICK', 360/4095,...
                              'RADIANS_PER_TICK',pi*2/4095,...
                              ...
                              'DEGREES_PER_SECOND_PER_TICK',     103/150,...
                              'RADIANS_PER_SECOND_PER_TICK',pi*103/27000,...
                              ...
                              'DEGREES_PER_SECOND_SQUARED_PER_TICK',   2180/254,...
                              'RADIANS_PER_SECOND_SQUARED_PER_TICK',pi*109/2286,...
                              ...
                              'MICROAMPS_PER_TORQUE',4500);
        
        % Dynamixel register addresses common to all AX and MX series actuators
        Address = struct('MODEL_NUMBER',                  uint8(0),...
                         'VERSION_OF_FIRMWARE',           uint8(2),...
                         'ID',                            uint8(3),...
                         'BAUD_RATE',                     uint8(4),...
                         'RETURN_DELAY_TIME',             uint8(5),...
                         'CW_ANGLE_LIMIT',                uint8(6),...
                         'CCW_ANGLE_LIMIT',               uint8(8),...
                         'THE_HIGHEST_LIMIT_TEMPERATURE',uint8(11),...
                         'THE_LOWEST_LIMIT_VOLTAGE',     uint8(12),...
                         'THE_HIGHEST_LIMIT_VOLTAGE',    uint8(13),...
                         'MAX_TORQUE',                   uint8(14),...
                         'STATUS_RETURN_LEVEL',          uint8(16),...
                         'ALARM_LED',                    uint8(17),...
                         'ALARM_SHUTDOWN',               uint8(18),...
                         ...
                         'TORQUE_ENABLE',uint8(24),...
                         'LED',          uint8(25),...
                         ...
                         'GOAL_POSITION',      uint8(30),...
                         'MOVING_SPEED',       uint8(32),...
                         'TORQUE_LIMIT',       uint8(34),...
                         'PRESENT_POSITION',   uint8(36),...
                         'PRESENT_SPEED',      uint8(38),...
                         'PRESENT_LOAD',       uint8(40),...
                         'PRESENT_VOLTAGE',    uint8(42),...
                         'PRESENT_TEMPERATURE',uint8(43),...
                         'REGISTERED',         uint8(44),...
                         'MOVING',             uint8(46),...
                         'LOCK',               uint8(47),...
                         'PUNCH',              uint8(48),...
                         ...
                         ... % Dynamixel register addresses for AX series
                         'CW_COMPLIANCE_MARGIN', uint8(26),...
                         'CCW_COMPLIANCE_MARGIN',uint8(27),...
                         'CW_COMPLIANCE_SLOPE',  uint8(28),...
                         'CCW_COMPLIANCE_SLOPE', uint8(29),...
                         ...
                         ... % Dynamixel register addresses for MX series
                         'MULTI_TURN_OFFSET', uint8(20),...
                         'RESOLUTION_DIVIDER',uint8(22),...
                         ...
                         'D_GAIN',uint8(26),...
                         'I_GAIN',uint8(27),...
                         'P_GAIN',uint8(28),...
                         ...
                         ... % DXL_MX64 and DXL_MX106 only
                         'CURRENT',                   uint8(68),...
                         'TORQUE_CONTROL_MODE_ENABLE',uint8(70),...
                         'GOAL_TORQUE',               uint8(71),...
                         ...
                         'GOAL_ACCELERATION',uint8(73));
        NUM_ADDRESS = uint8(74);
        
        Series = struct('AX_SERIES',     uint8(0),...
                        'MX_SERIES',     uint8(1),...
                        'UNKNOWN_SERIES',uint8(2));
    end
    
    properties (Constant, Hidden)
        SeriesType = struct('AX_SERIES',         uint8(0),...
                            'MX_SERIES_BASIC',   uint8(1),...
                            'MX_SERIES_ADVANCED',uint8(2),...
                            'UNKNOWN_SERIES',    uint8(3));
        
        AddressType = struct('BYTE_ADDRESS',     uint8(0),...
                             'WORD_L_ADDRESS',   uint8(1),...
                             'WORD_H_ADDRESS',   uint8(2),...
                             'UNDEFINED_ADDRESS',uint8(3));
    end
    
    properties (Constant)
        BROADCAST_ID = uint8(254);
        MAX_ID = 252;
        MAX_ACTUATORS = DXL.MAX_ID+1;
    end
    
    properties (Constant, Hidden)
        % Instruction commands
        Instruction = struct('PING',         uint8(1),...
                             'READ',         uint8(2),...
                             'WRITE',        uint8(3),...
                             'REG_WRITE',    uint8(4),...
                             'ACTION',       uint8(5),...
                             'FACTORY_RESET',uint8(6),...
                             'REBOOT',       uint8(8),...
                             'SYSTEM_WRITE',uint8(13),...
                             'STATUS',      uint8(85),...
                             'SYNC_READ',  uint8(130),...
                             'SYNC_WRITE', uint8(131),...
                             'BULK_READ',  uint8(146),...
                             'BULK_WRITE', uint8(147));
        
        % DXL_ALARM_LED and DXL_ALARM_SHUTDOWN error messages for 1.0 protocol
        Alarm = struct('ERROR_BIT_VOLTAGE',     uint8(1),...
                       'ERROR_BIT_ANGLE',       uint8(2),...
                       'ERROR_BIT_OVERHEAT',    uint8(4),...
                       'ERROR_BIT_RANGE',       uint8(8),...
                       'ERROR_BIT_CHECKSUM',   uint8(16),...
                       'ERROR_BIT_OVERLOAD',   uint8(32),...
                       'ERROR_BIT_INSTRUCTION',uint8(64));
        MAX_ALARM_ERROR = uint8(127);
        
        % Communication messages
        CommStatus = struct('TXSUCCESS',uint8(0),...
                            'RXSUCCESS',uint8(1),...
                            'TXFAIL',   uint8(2),...
                            'RXFAIL',   uint8(3),...
                            'TXERROR',  uint8(4),...
                            'RXWAITING',uint8(5),...
                            'RXTIMEOUT',uint8(6),...
                            'RXCORRUPT',uint8(7));
        
        % DXL_STATUS_RETURN_LEVEL options
        StatusReturn = struct('NONE',     uint8(0),...
                              'READ_ONLY',uint8(1),...
                              'ALL',      uint8(2),...
                              'UNKNOWN',uint8(255));
        
        ControlMode = struct('JOINT_MODE',         uint8(0),...
                             'WHEEL_MODE',         uint8(1),...
                             'MULTI_TURN_MODE',    uint8(2),...
                             'TORQUE_CONTROL_MODE',uint8(3),...
                             'UNKNOWN_MODE',       uint8(4));
    end
	
    properties (Access=protected, Constant, Hidden)
        MAX_PACKET_LENGTH = 143;
        PACKET_HEADER_LENGTH = 6;
        READ_WRITE_MIN_PARMETER_LENGTH = 2;
        SYNC_WRITE_MIN_PARMETER_LENGTH = 4;
        MAX_PARAM_BUFFER_LENGTH = DXL.MAX_PACKET_LENGTH-DXL.PACKET_HEADER_LENGTH-DXL.SYNC_WRITE_MIN_PARMETER_LENGTH;
        MAX_IDS_PER_BYTE = uint8(DXL.MAX_PARAM_BUFFER_LENGTH/2);
        MAX_IDS_PER_WORD = uint8(DXL.MAX_PARAM_BUFFER_LENGTH/3);
        DXL_PACKET_HEADER = uint8([255; 255]);
        
        UNINTIALIZED_BYTE = DXL.INVALID_BYTE;
        UNINTIALIZED_WORD = DXL.INVALID_WORD;
        UNINTIALIZED_LONG = DXL.INVALID_LONG;
        
        % Macros
        MAKEBYTES = @(w)typecast(uint16(w(:).'), 'uint8');
        LOBYTE = @(w)bitand(w, 255);
        HIBYTE = @(w)bitshift(w, -8);
    end
    
    properties (Constant)
        % Limiting values and conversions
        Properties = struct('MAX_RETURN_DELAY_TIME',uint8(254),...
                            ...
                            'MIN_LIMIT_VOLTAGE',uint8(50),...
                            'MAX_LIMIT_VOLTAGE',uint8(150),...
                            ...
                            'MAX_TORQUE_LIMIT',uint16(1023),...
                            ...
                            'MAX_PUNCH',uint16(1023),...
                            ...
                            'MICROSECOND_PER_RETURN_DELAY_TIME',2,...
                            'MILLIVOLT_PER_LIMIT_VOLTAGE',100);
    end
    
    properties (Access=private, Constant, Hidden)
        PktIdIdx = 3;
        PktLengthIdx = 4;
        PktInstIdx = 5;
    end
    
    methods
        function Dxl = DXL(varargin)
        %DXL  Create DXL class object.
        
            % Call superclass constructor
          	Dxl = Dxl@OPENCM(varargin{:});
        end
        
        function bPingID = ping(Dxl,bID)
        %PING  Check for existence of Dynamixel actuators.
        %
        %   See also DXL/readByte, DXL/readWord
        
            if nargin == 1
                bID = Dxl.BROADCAST_ID;
            end
            if isscalar(bID) && bID == Dxl.BROADCAST_ID
                b = [bID 2 Dxl.Instruction.PING];
                Dxl.TxBuffer = [Dxl.DXL_PACKET_HEADER; b(:); Dxl.CHECKSUM(b)];
              	writeRaw(Dxl);
                
                Dxl.RxLength = [];
                Count = readRaw(Dxl);
                Dxl.RxLength = Dxl.PACKET_HEADER_LENGTH;
                
                if mod(Count, Dxl.RxLength) == 0
                    bData = reshape(Dxl.RxBuffer, Dxl.RxLength, []);
                    sz = size(bData);
                    bPingID = zeros(sz(2), 1)+double(Dxl.INVALID_BYTE);
                    
                    for i = 1:sz(2)
                        Dxl.RxBuffer = bData(:, i);
                        if isValidPacket(Dxl, Dxl.RxBuffer(3), sz(1))
                            bPingID(i) = Dxl.RxBuffer(Dxl.PktIdIdx);
                        end
                    end
                else
                    bPingID = zeros(ceil(Count/Dxl.RxLength), 1)+double(Dxl.INVALID_BYTE);
                end
            else
                bPingID = zeros(numel(bID), 1)+double(Dxl.INVALID_BYTE);
                Dxl.RxLength = Dxl.PACKET_HEADER_LENGTH;
                for i = 1:numel(bID)
                    if bID(i) ~= Dxl.BROADCAST_ID
                        b = [bID(i) 2 Dxl.Instruction.PING];
                        Dxl.TxBuffer = [Dxl.DXL_PACKET_HEADER; b(:); Dxl.CHECKSUM(b)];
                        writeRaw(Dxl);
                        
                        Count = readRaw(Dxl);
                        if isValidPacket(Dxl, bID(i), Count)
                            bPingID(i) = Dxl.RxBuffer(Dxl.PktIdIdx);
                        end
                    end
                end
            end
        end
        
        function bData = readByte(Dxl,bID,bAddress,bNumAddress)
        %READBYTE  Read byte values from Dynamixel addresses.
        %
        %   See also: DXL/writeByte, DXL/readWord, DXL/writeWord, DXL/ping
        
            if isscalar(bID) && isscalar(bAddress) && nargin > 3
                bData = zeros(numel(bNumAddress), 1)+double(Dxl.INVALID_BYTE);
                if bID ~= Dxl.BROADCAST_ID && bNumAddress > 0
                    b = [bID 4 Dxl.Instruction.READ bAddress Dxl.BYTE_LENGTH*bNumAddress];
                    Dxl.TxBuffer = [Dxl.DXL_PACKET_HEADER; b(:); Dxl.CHECKSUM(b)];
                    writeRaw(Dxl);
                    
                    Dxl.RxLength = Dxl.PACKET_HEADER_LENGTH+double(Dxl.BYTE_LENGTH)*bNumAddress;
                    Count = readRaw(Dxl);
                    
                    if isValidPacket(Dxl, bID, Count)
                        for i = 1:bNumAddress
                            bData(i) = Dxl.RxBuffer(5+i);
                        end
                    end
                end
            else
                bData = zeros(numel(bID), 1)+double(Dxl.INVALID_BYTE);
                Dxl.RxLength = Dxl.PACKET_HEADER_LENGTH+double(Dxl.BYTE_LENGTH);
                for i = 1:numel(bID)
                    if bID(i) ~= Dxl.BROADCAST_ID
                        if isscalar(bAddress)
                            b = [bID(i) 4 Dxl.Instruction.READ bAddress Dxl.BYTE_LENGTH];
                        else
                            b = [bID(i) 4 Dxl.Instruction.READ bAddress(i) Dxl.BYTE_LENGTH];
                        end
                        Dxl.TxBuffer = [Dxl.DXL_PACKET_HEADER; b(:); Dxl.CHECKSUM(b)];
                        writeRaw(Dxl);
                        
                        Count = readRaw(Dxl);
                        if isValidPacket(Dxl, bID(i), Count)
                            bData(i) = Dxl.RxBuffer(6);
                        end
                    end
                end
            end
        end
        
        function status = writeByte(Dxl,varargin)
        %WRITEBYTE  Write byte values to Dynamixel addresses.
        %
        %   STATUS = WRITEBYTE(DXL, ADDRESS, DATA)
        %   
        %   STATUS = WRITEBYTE(DXL, ID, ADDRESS, DATA)
        %
        %   See also: DXL/readByte, DXL/writeWord, DXL/readWord
        
            if nargin == 3
                bID = Dxl.BROADCAST_ID;
                bAddress = varargin{1}(:).';
                bData = varargin{2};
            elseif nargin == 4
                bID = varargin{1}(:).';
                bAddress = varargin{2}(:).';
                bData = varargin{3};
            else
                if nargin < 3
                    error('DXL:writeByte:TooFewInputs',...
                          'Not enough input arguments.');
                else
                    error('DXL:writeByte:TooManyInputs',...
                          'Too many input arguments.');
                end
            end
            
            if isscalar(bID)
                DataLength = max(numel(bAddress), numel(bData));
                bData = bData(:).'+zeros(1, DataLength);
                b = [bID Dxl.BYTE_LENGTH*DataLength+3 Dxl.Instruction.WRITE bAddress(1) bData];
            else
                IDlength = numel(bID);
                AddressLength = numel(bAddress);
                if isvector(bData)
                    bData = bsxfun(@plus, bData(:).', zeros(AddressLength, IDlength));
                else
                    bData = bData.';
                end
                b = [bID; bData];
                b = [Dxl.BROADCAST_ID (Dxl.BYTE_LENGTH+1)*IDlength+4 Dxl.Instruction.SYNC_WRITE bAddress(1) Dxl.BYTE_LENGTH*AddressLength b(:).'];
            end
            
            Dxl.TxBuffer = [Dxl.DXL_PACKET_HEADER; b(:); Dxl.CHECKSUM(b)];
         	writeRaw(Dxl);
            
            status = Dxl.SUCCESS;
            if isscalar(bID) && bID ~= Dxl.BROADCAST_ID
                Dxl.RxLength = Dxl.PACKET_HEADER_LENGTH;
                Count = readRaw(Dxl);
                if ~isValidPacket(Dxl, bID, Count)
                    status = Dxl.FAILURE;
                end
            end
        end
        
        function wData = readWord(Dxl,bID,bAddress,bNumAddress)
        %READWORD  Read word values from Dynamixel addresses.
        %
        %   See also: DXL/writeWord, DXL/readByte, DXL/writeByte, DXL/ping
        
            if isscalar(bID) && isscalar(bAddress) && nargin > 3
                wData = zeros(numel(bNumAddress), 1)+double(Dxl.INVALID_WORD);
                if bID ~= Dxl.BROADCAST_ID && bNumAddress > 0
                    b = [bID 4 Dxl.Instruction.READ bAddress Dxl.WORD_LENGTH*bNumAddress];
                    Dxl.TxBuffer = [Dxl.DXL_PACKET_HEADER; b(:); Dxl.CHECKSUM(b)];
                    writeRaw(Dxl);
                    
                    Dxl.RxLength = Dxl.PACKET_HEADER_LENGTH+double(Dxl.WORD_LENGTH)*bNumAddress;
                    Count = readRaw(Dxl);
                    
                    if isValidPacket(Dxl, bID, Count)
                        for i = 1:bNumAddress
                            wData(i) = Dxl.MAKEWORD(Dxl.RxBuffer(4+2*i), Dxl.RxBuffer(5+2*i));
                        end
                    end
                end
            else
                wData = zeros(numel(bID), 1)+double(Dxl.INVALID_WORD);
                Dxl.RxLength = Dxl.PACKET_HEADER_LENGTH+double(Dxl.WORD_LENGTH);
                for i = 1:numel(bID)
                    if bID(i) ~= Dxl.BROADCAST_ID
                        if isscalar(bAddress)
                            b = [bID(i) 4 Dxl.Instruction.READ bAddress Dxl.WORD_LENGTH];
                        else
                            b = [bID(i) 4 Dxl.Instruction.READ bAddress(i) Dxl.WORD_LENGTH];
                        end
                        Dxl.TxBuffer = [Dxl.DXL_PACKET_HEADER; b(:); Dxl.CHECKSUM(b)];
                        writeRaw(Dxl);
                        
                        Count = readRaw(Dxl);
                        if isValidPacket(Dxl, bID(i), Count)
                            wData(i) = Dxl.MAKEWORD(Dxl.RxBuffer(6), Dxl.RxBuffer(7));
                        end
                    end
                end
            end
        end
        
        function status = writeWord(Dxl,varargin)
        %WRITEWORD  Write word values to Dynamixel addresses.
        %
        %   STATUS = WRITEWORD(DXL, ADDRESS, DATA) 
        %   
        %   STATUS = WRITEWORD(DXL, ID, ADDRESS, DATA) 
        %   
        %   See also: DXL/readWord, DXL/writeByte, DXL/readByte
        
            if nargin == 3
                bID = Dxl.BROADCAST_ID;
                bAddress = varargin{1}(:).';
                wData = varargin{2};
            elseif nargin == 4
                bID = varargin{1}(:).';
                bAddress = varargin{2}(:).';
                wData = varargin{3};
            else
                if nargin < 3
                    error('DXL:writeWord:TooFewInputs',...
                          'Not enough input arguments.');
                else
                    error('DXL:writeWord:TooManyInputs',...
                          'Too many input arguments.');
                end
            end
            
            if isscalar(bID)
                DataLength = max(numel(bAddress), numel(wData));
                wData = wData(:).'+zeros(1, DataLength);
                b = [bID Dxl.WORD_LENGTH*DataLength+3 Dxl.Instruction.WRITE bAddress(1) Dxl.MAKEBYTES(wData)];
            else
                IDlength = numel(bID);
                AddressLength = numel(bAddress);
                if isvector(wData)
                    wData = bsxfun(@plus, wData(:), zeros(IDlength, AddressLength));
                    DataLength = AddressLength;
                else
                    DataLength = max(AddressLength, size(wData, 2));
                end
                b = [bID; reshape(Dxl.MAKEBYTES(wData.'), Dxl.WORD_LENGTH*DataLength, IDlength)];
                b = [Dxl.BROADCAST_ID (Dxl.WORD_LENGTH+1)*IDlength+4 Dxl.Instruction.SYNC_WRITE bAddress(1) Dxl.WORD_LENGTH*AddressLength b(:).'];
            end
            
            Dxl.TxBuffer = [Dxl.DXL_PACKET_HEADER; b(:); Dxl.CHECKSUM(b)];
            writeRaw(Dxl);
            
            status = Dxl.SUCCESS;
            if isscalar(bID) && bID ~= Dxl.BROADCAST_ID
                Dxl.RxLength = Dxl.PACKET_HEADER_LENGTH;
                Count = readRaw(Dxl);
                if ~isValidPacket(Dxl, bID, Count)
                    status = Dxl.FAILURE;
                end
            end
        end
    end
    
    methods (Access=private, Hidden)
        function tf = isValidPacket(Dxl,bID,Count)
        %ISVALIDPACKET  
        
            tf = Count == Dxl.RxLength && ...
                 all(Dxl.RxBuffer(1:4) == [Dxl.DXL_PACKET_HEADER; bID; Count-Dxl.PktInstIdx+1]) && ...
                 bitand(sum(Dxl.RxBuffer(3:end)), double(Dxl.CHECKSUM_MASK)) == Dxl.CHECKSUM_MASK;
        end
    end
end