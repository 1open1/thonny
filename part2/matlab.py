% Channel details
channelID = 2934028;
readAPIKey = '57HB9YD28FI7C4VF';
fieldNumber = 1; % Use 2 for Data2 if needed

% Read the last 100 points from the specified field
data = thingSpeakRead(channelID, ...
    "Fields", fieldNumber, ...
    "NumPoints", 100, ...
    "ReadKey", readAPIKey);

% Calculate statistics
maxval = max(data);
minval = min(data);
meanval = mean(data);
stdval = std(data);

% Display the results
disp("Maximum Value:");
disp(maxval);

disp("Minimum Value:");
disp(minval);

disp("Mean Value:");
disp(meanval);

disp("Standard Deviation:");
disp(stdval);
