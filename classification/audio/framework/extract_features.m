function [ F ] = extract_features( N, path_feat )
%EXTRACT_FEATURES Summary of this function goes here
%   Detailed explanation goes here

    % cell array of features
    F = cell(N,3);
    
    for i = 1:N                     

        % list of csv files
        folder_feat = strcat( path_feat, num2str(i) );
        wav_list = dir(folder_feat);

        % vector of features per person
        f_person = [];
        n = [];
        c = [];
        video_nr = 1;
        
        for j = 3:5:size(wav_list,1)
            
            % 5 files per audio
            for k = j:j+4
                
                % features from audio files
                fname = strcat( strcat(folder_feat,'/'), wav_list(k).name);
                [Audio_Data, fs] = audioread(fname);
                Audio_Data = mean(Audio_Data, 2);
                New_Audio = Voice_Activity_Detector (Audio_Data, fs);
                f = mfcc(New_Audio, fs);  
                f = f(2:end, :);

                f_person = [f_person; f'];
                nf = size(f,2);
                n = [n; repmat(video_nr,nf,1)];
                
                if ( ( video_nr >= 1 ) && ( video_nr <= 2 ) )
                    c = [c; repmat('E',nf,1)]; % english
                elseif ( video_nr == 3 )
                    c = [c; repmat('N',nf,1)]; % native
                else
                    c = [c; repmat('O',nf,1)]; % other
                end
            end
            
            video_nr = video_nr + 1;
        end

        F(i,1) = {f_person};
        F(i,2) = {n};
        F(i,3) = {c};
    end

end

