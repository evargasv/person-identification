function [train_X,train_Y,test_X,test_Y]=assign_train_test(N,F,trn)
%ASSIGN_TRAIN_TEST Summary of this function goes here
%   Detailed explanation goes here

    % variable initialisation
    train_X = [];
    train_Y = [];
    test_X = [];
    test_Y = [];

    for i = 1:N                     

        % type of file
        T = F{i,3};

        % features vector
        feat_vec = F{i,1};

        % train
        idx_train = ( T(:) == trn );
        x = feat_vec(idx_train,:);
        y = ones( size(x,1),1 ) * i; 
        train_X = [train_X; x];
        train_Y = [train_Y; y];

        % test
        idx_test = ( T(:) ~= trn );
        test_x = feat_vec(idx_test,:);
        test_X = [test_X; test_x];
        test_y = ones( size(test_x,1),1 ) * i; 
        test_Y = [test_Y; test_y];
   
    end

end

