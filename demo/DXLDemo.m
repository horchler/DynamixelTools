function DXLDemo
%DXLDemo  Simple script to demonstrate DXL class
%
%   Ensure that OpenCM9.04 microcotroller is connected via USB. Actuators are
%   assumed to be MX Series actuators by default. To handle AX Series actuators
%   (or mixed types), change the DEFAULT_BAUD_RATE_VALUE property to
%   BaudRateValue.BAUD_1000000 (or less) or call the begin method and specify
%   the desired baud rate value.

%   Andrew D. Horchler, adh9 @ case . edu
%   Created: 7-6-15, Revision: 1.0, 7-13-15


USBBaudRate = 115200;
Id = 1;

% Instantiate DXL class, and begin connection
Dxl = DXL(USBBaudRate);

% Stop all actuators
Dxl.writeWord(Dxl.BROADCAST_ID,Dxl.Address.MOVING_SPEED,0);

% Confirm actuator ID is present before proceeding
if Dxl.ping(Id) == Id
    % Set return delay to minimum
    Dxl.writeByte(Id,Dxl.Address.RETURN_DELAY_TIME,0);
    
    % Set wheel mode by setting both CW_ANGLE_LIMIT and CCW_ANGLE_LIMIT to 0
    Dxl.writeWord(Id,Dxl.Address.CW_ANGLE_LIMIT,[0 0]);
    Dxl.writeWord(Id,Dxl.Address.TORQUE_LIMIT,Dxl.Properties.MAX_TORQUE_LIMIT);
    
    % Get and print present position
    fprintf(1, 'Time: %u, Position: %u.\n',Dxl.micros(),Dxl.readWord(Id,Dxl.Address.PRESENT_POSITION));
    
    % Set moving speed
    Dxl.writeWord(Id,Dxl.Address.MOVING_SPEED,50);
    
    % Time 1,000 read operations
    n = 1e3;
    times = zeros(n,1);
    position = zeros(n,1);
    tic;
    for i = 1:n
        %Dxl.writeWord(Id,Dxl.Address.MOVING_SPEED,round(50+50*sin(i/10)));
        times(i) = double(Dxl.micros());
        position(i) = Dxl.readWord(Id,Dxl.Address.PRESENT_POSITION);
    end
    t = toc;
    fprintf(1, 'Elapsed time: %.4f seconds (%.4f reads/second).\n',t,n/t);
    
    % Stop actuator
    Dxl.writeWord(Id,Dxl.Address.MOVING_SPEED,0);
    
    figure;
    plot(times,position);
end

% Pause tosser state
Dxl.pause();

% Cleanup
delete(Dxl);