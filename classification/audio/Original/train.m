function code = train(traindir, n)
% Speaker Recognition: Training Stage
%
% Input:
%       traindir : string name of directory contains all train sound files
%       n        : number of train files in traindir
%
% Output:
%       code     : trained VQ codebooks, code{i} for i-th speaker
%
% Note:
%       Sound files in traindir is supposed to be:
%                       s1.wav, s2.wav, ..., sn.wav
% Example:
%       >> code = train('C:\data\train\', 8);

%Y_Train = [];
%X_Train = [];
k = 16;                         % number of centroids required

for i = 1:n                     % train a VQ codebook for each speaker
    file = sprintf('%s%d.wav', traindir, i);
    disp(file);
    
    [Audio_Data, fs] = audioread(file);
    Audio_Data = mean(Audio_Data, 2);
    s = Voice_Activity_Detector(Audio_Data, fs);
    
    v_temp = mfcc(s, fs);            % Compute MFCC's
    v = v_temp(2:end, :);            % Exclude the first MFCC Coefficient (Very high compared to the others)
    
    code{i} = vqlbg(v, k);      % Train VQ codebook
    
    %%%%%%%%%%%%%%%%%%%%%%% KNN %%%%%%%%%%%%%%%%%%%%%%%%%%
   % Person_Label = Train_Person_ours(i);  % Our dataset
   % Y_Train = [Y_Train; Person_Label];
   % Temp = mean(v, 2);
   % X_Train = [X_Train; Temp'];
   % code = fitcknn(X_Train, Y_Train, 'NumNeighbors', k);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
end