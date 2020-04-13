function [ AF ] = avg_files( N,F )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    for i = 1:N   
        
        % variable initialisation
        X = [];
        Y = [];
        T = [];
        
        % number of files
        num = F{i,2};
        uni_num = unique(num);
        nf = size(uni_num,1);

        % features vector
        feat_vec = F{i,1};
        % clothes type
        cloth = F{i,3};

        for j=1:nf
                
            % train
            idx_train = ( num(:) == j );
            x = feat_vec(idx_train,:);
            t = cloth(idx_train,:);
            % mean of features
            x = mean(x,1);
            X = [X; x];
            
            Y = [Y; j];
            T = [T; t(1)];
            
        end
        
        AF(i,1) = {X};
        AF(i,2) = {Y};
        AF(i,3) = {T};
        
    end

end

