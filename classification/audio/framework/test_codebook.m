function [ Y_test, Y_label, C, a ] = test_codebook( F, N, code_book )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

    Y_test = zeros(N,1);
    Y_label = zeros(N,1);
    
    for i=1:N
        
        mfcc_vec = F{i,1};
        lang_vec = F{i,3};
        v = mfcc_vec( lang_vec == 'E',: );
        
        distmin = inf;
        k1 = 1;

        for l = 1:length(code_book)
            
            d = disteu(v', code_book{l});
            dist = sum(min(d,[],2)) / size(d,1);

            if dist < distmin
                distmin = dist;
                k1 = l;
            end
        end
        
        Y_test(i) = i; 
        Y_label(i) = k1; 
    end

    % confusion matrix
    C=confusionmat(Y_test,Y_label);
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

