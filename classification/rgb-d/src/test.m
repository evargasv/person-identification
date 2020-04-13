function [ C, a ] = test( model, test_X, test_Y, N )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

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

end

