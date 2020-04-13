clear;
clc;
close all;
addpath(genpath('.'));
% -------------------------------------------------------------------------
% -- Extract feature vector
% -------------------------------------------------------------------------

% number of people
N = 15;
% folder of features
path_feat = './audio1/';
% extract the features from csv files
F = extract_features( N, path_feat );

% -------------------------------------------------------------------------
% -- Select train/test samples
% -------------------------------------------------------------------------

% assign according to the language
[ train_X, train_Y, test_X, test_Y ] = assign_train_test(N,F,'N');

% assign the data for training (60%) and testing (40%)
%[ train_X, train_Y, test_X, test_Y ] = assign_randomly(N,F);

%code_book = train_codebook(F,N);

% -------------------------------------------------------------------------
% -- Train
% -------------------------------------------------------------------------

% create model using train data
%model = fitcknn(train_X,train_Y,'NumNeighbors',1);
code_book = train(train_X,train_Y,N);

[ Y_test, Y_label, C, a ] = test_codebook( F, N, code_book );

% -------------------------------------------------------------------------
% -- Test
% -------------------------------------------------------------------------

%[ C, a ] = test( model, test_X, test_Y, N );
%label = test(test_X, test_Y, N, code_book);

% -------------------------------------------------------------------------
% -- Export to latex
% -------------------------------------------------------------------------

%export_features( F, N );
%export_confusion_matrix( C, N );
%a


