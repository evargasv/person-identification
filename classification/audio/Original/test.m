function [Y_Test, Y_Label] = test(testdir, n, code)
% Speaker Recognition: Testing Stage
%
% Input:
%       testdir : string name of directory contains all test sound files
%       n       : number of test files in testdir
%       code    : codebooks of all trained speakers
%
% Note:
%       Sound files in testdir is supposed to be:
%               s1.wav, s2.wav, ..., sn.wav
%
% Example:
%       >> test('C:\data\test\', 8, code);

Y_Test = [];
Y_Label = [];

for k = 1:n                     % read test sound file of each speaker
    file = sprintf('%s%d.wav', testdir, k);
    [Audio_Data, fs] = audioread(file);
    Audio_Data = mean(Audio_Data, 2);
    s = Voice_Activity_Detector(Audio_Data, fs);
       
    %     Person_Test = Test_Person(k);       % Another data set
    Person_Test = Test_Person_ours(k);    % Our dataset
    
    Y_Test = [Y_Test; Person_Test];
    
    v_temp = mfcc(s, fs);            % Compute MFCC's
    v = v_temp(2:end, :);            % Exclude the first MFCC Coefficient (Very high compared to the others)
    
    distmin = inf;
    k1 = 0;
    
    for l = 1:length(code)      % each trained codebook, compute distortion
        d = disteu(v, code{l});
        dist = sum(min(d,[],2)) / size(d,1);
        
        if dist < distmin
            distmin = dist;
            k1 = l;
            %             Person_Label = Test_Person(k1);      % Another data set
            Person_Label = Train_Person_ours(k1);  % Our dataset
        end
    end
    
    Y_Label = [Y_Label; Person_Label];
    
    %     msg = sprintf('Speaker %d matches with speaker %d', k, k1);
    msg = sprintf('Speaker %d matches with speaker %d', Person_Test, Person_Label);
    disp(msg);
end