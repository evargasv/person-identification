function [code, X_Train, Y_Train] = train(traindir, n)
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

X_Train = [];
Y_Train = [];

for i = 1 : n                     % train a VQ codebook for each speaker
    
    % Read the audio file
    file = sprintf('%s%d.wav', traindir, i);
    disp(file);
    [s_1, fs] = audioread(file);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Remove the noise:-
    [vs,zo]=vadsohn(s_1, fs, 'a');
    [r, c] = find(vs(:, 1)==1);
    
    s = zeros(size(r));
    for n = 1:size(r, 1)
        s(n) = s_1(r(n));
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Generate the MFCC coefficients
    v = mfcc(s, fs);            % Compute MFCC's
    X_Train = [X_Train; v'];
    
    Person = Test_Person(i);
    k_Temp = Person; %* ones(size(v, 1), 1);
    Y_Train = [Y_Train; k_Temp];
    
%     code{i} = vqlbg(v, k);      % Train VQ codebook
    
end

end