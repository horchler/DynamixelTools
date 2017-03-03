function OPENCMDemo
%OPENCMDEMO  Simple script to demonstrate OPENCM class
%
%   Ensure that OpenCM9.04 microcontroller is connected via USB. Actuators are
%   assumed to be MX Series actuators by default.

%   Andrew D. Horchler, horchler @ gmail . com
%   Created: 7-6-15, Revision: 1.0, 2-25-16


USBBaudRate = 115200;

% Instantiate OPENCM class
OpenCM = OPENCM();

% Set verbosity level to maximum for demo
OpenCM.Verbosity = 2;

% Open serial connection to OpenCM9.04
OpenCM.open(USBBaudRate);

% Get version number of Tosser running on OpenCM9.04, if present
VersionNumber = version(OpenCM);
fprintf(1, 'Version: %u.\n', VersionNumber);

VersionNumber = version(OpenCM);
fprintf(1, 'Version: %u.\n', VersionNumber);

VersionNumber = version(OpenCM);
fprintf(1, 'Version: %u.\n', VersionNumber);
return

% Reset board and upload new binary file if needed
if VersionNumber ~= OpenCM.Version
    OpenCM.connect();
end

OpenCM.begin();
t0 = OpenCM.micros();
pause(2);
fprintf(1, 'Elapsed time: %u microseconds.\n', OpenCM.micros()-t0);
OpenCM.pause();
OpenCM.setLED(OpenCM.LED_OFF);
pause(2);
OpenCM.setLED(OpenCM.LED_ON);
pause(2);
OpenCM.blink();
pause(2);
%OpenCM.exit();

% Not really necessary, object auto-deleted when OpenCM goes out of scope
delete(OpenCM);