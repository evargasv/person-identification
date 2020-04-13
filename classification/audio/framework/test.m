function [ C, a ] = test(model, test_X, test_Y, N)
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


    % number of test entries
    N_TEST = size(test_X,1);

    % classification
    for i=1:N_TEST
        t = test_X(i,:);
        label(i,:) = predict(model,t);
    end

    % confusion matrix
    C=confusionmat(test_Y,label);
    C=C';
    
    % normalisation
    sum_col = sum(C);
    norm = repmat(sum_col,N,1);
    C = C ./ norm;
    C(isnan(C(:))) = 0;
    
    % accuracy
    a = trace(C)/N;
    
    % plot the confusion matrix
    imagesc(C)
