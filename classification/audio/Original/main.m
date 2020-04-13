clc
close all;
warning off;

addpath(genpath(':'))

%% Our dataset
N = 15;     % Number of persons in the dataset
traindir = '.\New_Data\train_ours\';
testdir = '.\New_Data\test_ours\';
code = train(traindir, 144);
[Y_Test, Y_Label] = test(testdir, 96, code);
% [Y_Test, Y_Label] = Test_Temp(testdir, 96, code);

% %% Small dataset (just us with English files only)
% N = 4;   % Number of persons in the dataset
% traindir = '.\New_Data\Train_small\';
% testdir = '.\New_Data\Test_small\';
% code = train(traindir, 24);
% [Y_Test, Y_Label] = test(testdir, 16, code);

%% Another dataset
% N = 9;   % Number of persons in the dataset
% traindir = '.\New_Data\train\';
% testdir = '.\New_Data\test\';
% code = train(traindir, 27);
% [Y_Test, Y_Label] = test(testdir, 27, code);

% %% Very new dataset (just us with English files only)
% N = 4;   % Number of persons in the dataset
% traindir = '.\New_Data\Train_Small_New\';
% testdir = '.\New_Data\Test_Small_New\';
% code = train(traindir, 12);
% [Y_Test, Y_Label] = test(testdir, 8, code);


% confusion matrix
C = confusionmat(Y_Test,Y_Label);

% normalisation
sum_col = sum(C);
norm = repmat(sum_col, N, 1);
C = C ./ norm;
C(isnan(C(:))) = 0;

% accuracy
a = trace(C)*100/N

% plot the confusion matrix
imagesc(C)
title('K = 10')