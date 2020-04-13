% clc
% clear all
% close all;

traindir = '.\Test\';
testdir = '.\Train\';

[code, X_Train, Y_Train] = train(traindir, 144);
[X_Test, Y_Test, Y_Final] = test(testdir, 96, code);


% normalisation
C=confusionmat(Y_Test,Y_Final);
sum_col = sum(C);
norm = repmat(sum_col,11,1);
C = C ./ norm;
C(isnan(C(:))) = 0;

% accuracy
a = trace(C)*100/11

% plot the confusion matrix
imagesc(C)


% c=confusionmat(Y_Test,Y_Final);
% for i=1:size(c,2)
%     norm=sum(c(:,i));
%     c1(:,i)=c(:,i)./norm;
% end
% imagesc(c1)
% Accuracy = trace(c1)*100/15

% % normalisation
% sum_col = sum(c);
% norm = repmat(sum_col,15,1);
% c = c ./ norm;
% c(isnan(c(:))) = 0;
%
% % accuracy
% a = trace(c)/15

% plot the confusion matrix
imagesc(c)

% file = 'Train/4.wav'
% [s, fs] = audioread(file);
% v = mfcc(s, fs);            % Compute MFCC's
% imagesc(v(2:end,:))
%
% file = 'Train/5.wav';
% [s, fs] = audioread(file);
% v1 = mfcc(s, fs);            % Compute MFCC's
% figure
% imagesc(v1(2:end,:))


