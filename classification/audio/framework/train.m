function code = train(train_X,train_Y,N)
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

k = 16;                         % number of centroids required

for i = 1:N                     % train a VQ codebook for each speaker

    idx_col = (train_Y == i);       
    f = train_X(idx_col,:);     % Compute MFCC's
    code{i} = vqlbg(f', k);      % Train VQ codebook
    
end