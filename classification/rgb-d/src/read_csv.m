function [ f ] = read_csv( fname )
%READ_CSV Read an csv file and eliminates the rows containing not
%consistent data

    % read CSV file
    M = csvread(fname);
    % eliminate the zero of the last column
    f = M(:,1:end-1);
    
    % delete NaN entries
    f(any(isnan(f),2),:)=[];
    % delete < 0 entries
    f(any(f<0,2),:)=[];

end

