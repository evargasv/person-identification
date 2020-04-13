clear;
clc;
close all;

% -------------------------------------------------------------------------
% -- Extract feature vector
% -------------------------------------------------------------------------

% number of people
N = 14;
% folder of features
path_feat = '../features1/';
% extract the features from csv files
F = extract_features( N, path_feat );

% -------------------------------------------------------------------------
% -- Avg features per file
% -------------------------------------------------------------------------

[ AF ] = avg_files( N,F );

% -------------------------------------------------------------------------
% -- Select train/test samples
% -------------------------------------------------------------------------

% assign the data for training (60%) and testing (40%)
%[ train_X, train_Y, test_X, test_Y ] = select_standard( N, F );

% train using summer clothes and test with winter clothes
%[ train_X, train_Y, test_X, test_Y ] = select_season(N,F,'S','W');

% train using winter clothes and test with summer clothes
[ train_X, train_Y, test_X, test_Y ] = select_season(N,F,'W','S');

% -------------------------------------------------------------------------
% -- Train
% -------------------------------------------------------------------------

% create model using train data
%model = fitcknn(train_X,train_Y);
%model   = fitcsvm(train_X, train_Y);

% -------------------------------------------------------------------------
% -- Test
% -------------------------------------------------------------------------

%[ C, a ] = test( model, test_X, test_Y, N );

% -------------------------------------------------------------------------
% -- Export to latex
% -------------------------------------------------------------------------

%export_features( F, N );

%export_confusion_matrix( C, N );
%a

% -------------------------------------------------------------------------
% -- Weka
% -------------------------------------------------------------------------

vecTrain = train_X;
labTrain = train_Y';

vecTest = test_X;
labTest = test_Y';

javaaddpath('C:/Program Files (x86)/Weka-3-6/weka.jar');
import weka.*;

%each feature needs a name
for i=1:size(vecTrain,2)+1,
	f{i}=num2str(i);
end

%each classname saved as a string 
for i=1:size(vecTrain,1),
	for j=1:size(vecTrain,2),
		ctrain{i,j} = vecTrain(i,j);
	end
	ctrain{i,j+1} = [char(labTrain(i)+64) char(labTrain(i)+64)];
end
for i=1:size(vecTest,1),
	for j=1:size(vecTest,2),
		ctest{i,j} = vecTest(i,j);
	end
	ctest{i,j+1} = [char(labTest(i)+64) char(labTest(i)+64)];
end

%conversion functions
wekaTrain = convertWekaDataset('training',f,ctrain);
wekaTest = convertWekaDataset('testing',f,ctest);

%% FOR Random Forest

% %Settings for the classifier
% v(1) = java.lang.String('-I');
% v(2) = java.lang.String('10');
% v(3) = java.lang.String('-K');
% v(4) = java.lang.String('0');
% v(5) = java.lang.String('-S');
% v(6) = java.lang.String('1');
% v(7) = java.lang.String('-depth');
% v(8) = java.lang.String('0');
% prm = cat(1,v(1:end));
% 
% %create classifier instance, and perform the evaluation
%  classifier = javaObject('weka.classifiers.trees.RandomForest');
%  
%  classifier.setOptions(prm)
%  
%  %build classifier model
%  classifier.buildClassifier(wekaTrain);

%% FOR SVM:

classifier = weka.classifiers.functions.SMO();     
  classifier.setC(100);
  k = weka.classifiers.functions.supportVector.Puk();
  k.setOmega(1.0);
  k.setSigma(1.0);
  classifier.setKernel(k);  
classifier.buildClassifier(wekaTrain); %% "data" here is the training data
% 
%% testing with final confusion matrix
cm = zeros(wekaTest.numClasses,wekaTest.numClasses);
mx = zeros(1,wekaTest.numInstances);
pmx = zeros(1,wekaTest.numInstances);
predicted = zeros(wekaTest.numInstances,wekaTest.numClasses);
for i=1:wekaTest.numInstances
	instance = wekaTest.instance(i-1);
	predicted = classifier.distributionForInstance(instance)';
    % maximum probability and maximum class(pmx)
	[mx(i),pmx(i)] = max(predicted);
	cm(instance.classValue+1,pmx(i)) = cm(instance.classValue+1,pmx(i))+1;
end
cm
correct = sum(diag(cm))/sum(cm(:))

% %evaluate classifier
% ev = javaObject('weka.classifiers.Evaluation',wekaTrain);
% cloutputbuf = java.lang.StringBuffer();
% pt = javaObject('weka.classifiers.evaluation.output.prediction.PlainText');
% pt.setBuffer(cloutputbuf);
% pt.setHeader(wekaTrain);
% ev.evaluateModel(classifier,wekaTest,pt);
% cm = ev.confusionMatrix
% correct = sum(diag(cm))/sum(cm(:))


