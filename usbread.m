handle = serial('/dev/tty.usbmodem411','Baudrate', 9600);
fopen(handle);
fprintf(handle, message); 
fwrite(handle, variable); 
fclose(handle);

% X1:   560   Y1:   660   X2:   487   Y2:   625   X3:   574   Y3:   624   X4:   536   Y4:   592

% 4-points detectors results
% xloc: 989   yloc:353   angle:-309   current angle:472

% X1:   560   Y1:   660   X2:   1024   Y2:   1024   X3:   574   Y3:   624   X4:   536   Y4:   592

% -158





