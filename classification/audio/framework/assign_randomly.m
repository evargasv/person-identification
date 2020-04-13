function [ train_X,train_Y,test_X,test_Y ] = assign_randomly( N,F )
%ASSIGN_RANDOMLY Summary of this function goes here
%   Detailed explanation goes here

    % variable initialisation
    train_X = [];
    train_Y = [];
    test_X = [];
    test_Y = [];

    for i = 1:N                     

        % features vector
        feat_vec = F{i,1};
        % number of samples
        n = size(feat_vec,1);

        % permute the index
        rand_indx = randperm(n);

        % train
        ntrain = round(n * 0.6);
        x = feat_vec(rand_indx(1:ntrain),:);
        y = ones( size(x,1),1 ) * i; 
        train_X = [train_X; x];
        train_Y = [train_Y; y];

        % test
        test_x = feat_vec(rand_indx(ntrain+1:end),:);
        test_X = [test_X; test_x];
        test_y = ones( size(test_x,1),1 ) * i; 
        test_Y = [test_Y; test_y];
    end
    
end

