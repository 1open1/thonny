channelID = 2934028; 
readAPIKey = '57HB9YD28FI7C4VF';
writeAPIKey = 'YOUR_WRITE_API_KEY'; data = thingSpeakRead(channelID, "NumPoints", 100, "ReadKey", readAPIKey); 
thingSpeakWrite(channelID, "Fields", [1, 2, 3, 4], "Values", [max(data), min(data), mean(data), std(data)], "WriteKey", writeAPIKey);

maxval = max(data);
minval = min(data);
meanval = mean(data);
stdval = std(data);

disp("Maximum Value:");
disp(maxval);
disp("Minimum Value:");
disp(minval);
disp("Mean Value:");
disp(meanval);
disp("Standard Deviation:");
disp(stdval);
