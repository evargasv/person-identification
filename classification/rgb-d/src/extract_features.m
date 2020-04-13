function [ F ] = extract_features( N, path_feat )
%EXTRACT_FEATURES Summary of this function goes here
%   Detailed explanation goes here

    % cell array of features
    F = cell(N,3);
    
    for i = 1:N                     

        % list of csv files
        folder_feat = strcat( path_feat, num2str(i) );
        csv_list = dir(folder_feat);

        % vector of features per person
        f_person = [];
        n = [];
        c = [];

        for j = 3:size(csv_list,1)

            % features from csv files
            fname = csv_list(j).name;
            f = read_csv( fname );
            f_person = [f_person; f];
            nf = size(f,1);
            n = [n; repmat(j-2,nf,1)];
            
            % assign the type of clothes
            if( strfind(fname,'w') >= 1 )% winter clothes
                c = [c; repmat('W',nf,1)];
            else % summer clothes
                c = [c; repmat('S',nf,1)];
            end
            
        end

        F(i,1) = {f_person};
        F(i,2) = {n};
        F(i,3) = {c};
    end

end

