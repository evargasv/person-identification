function [Y_Test, Y_Label] = Test_Temp(testdir, n, code)

Y_Test = [];
Y_Label = [];

for k = 1:n                     % read test sound file of each speaker
    file = sprintf('%s%d.wav', testdir, k);
    [Audio_Data, fs] = audioread(file);
    Audio_Data = mean(Audio_Data, 2);
    s = Voice_Activity_Detector(Audio_Data, fs);
    Person_Test = Test_Person_ours(k);    % Our dataset
    Y_Test = [Y_Test; Person_Test];
    
    v_temp = mfcc(s, fs);            % Compute MFCC's
    v = v_temp(2:end, :);            % Exclude the first MFCC Coefficient (Very high compared to the others)
    Temp = mean(v, 2);
    
    Y_Label_Temp = predict(code,Temp');
    Y_Label = [Y_Label; Y_Label_Temp];

end

end