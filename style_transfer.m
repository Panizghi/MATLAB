styleImage = im2double(imread("starryNight.jpg"));
contentImage = imread("lighthouse.png");

% Display the style image and content image as a montage.
imshow(imtile({styleImage,contentImage},BackgroundColor="w"));


% pretrained VGG-19 network
net = vgg19;

lastFeatureLayerIdx = 38;
layers = net.Layers;
layers = layers(1:lastFeatureLayerIdx);

for l = 1:lastFeatureLayerIdx
    layer = layers(l);
    if isa(layer,"nnet.cnn.layer.MaxPooling2DLayer")
        layers(l) = averagePooling2dLayer( ...
            layer.PoolSize,Stride=layer.Stride,Name=layer.Name);
    end
end

lgraph = layerGraph(layers);
% extrcation plot
plot(lgraph)
  
% convert the layer graph to a dlnetwork object 
dlnet = dlnetwork(lgraph);


% pre process data

imageSize = [384,512];
styleImg = imresize(styleImage,imageSize);
contentImg = imresize(contentImage,imageSize);

imgInputLayer = lgraph.Layers(1);
meanVggNet = imgInputLayer.Mean(1,1,:);

styleImg = rescale(single(styleImg),0,255) - meanVggNet;
contentImg = rescale(single(contentImg),0,255) - meanVggNet;

% transfer image 

noiseRatio = 0.7;
randImage = randi([-20,20],[imageSize 3]);
transferImage = noiseRatio.*randImage + (1-noiseRatio).*contentImg;

% loss 

styleTransferOptions.contentFeatureLayerNames = "conv4_2";
styleTransferOptions.contentFeatureLayerWeights = 1;

styleTransferOptions.styleFeatureLayerNames = [ ...
    "conv1_1","conv2_1","conv3_1","conv4_1","conv5_1"];

styleTransferOptions.styleFeatureLayerWeights = [0.5,1.0,1.5,3.0,4.0];

% total loss

styleTransferOptions.alpha = 1; 
styleTransferOptions.beta = 1e3;

% training option 

numIterations = 2500;
learningRate = 2;
trailingAvg = [];
trailingAvgSq = [];


% train 

dlStyle = dlarray(styleImg,"SSC");
dlContent = dlarray(contentImg,"SSC");
dlTransfer = dlarray(transferImage,"SSC");
if canUseGPU
    dlContent = gpuArray(dlContent);
    dlStyle = gpuArray(dlStyle);
    dlTransfer = gpuArray(dlTransfer);
end
numContentFeatureLayers = numel(styleTransferOptions.contentFeatureLayerNames);
contentFeatures = cell(1,numContentFeatureLayers);
[contentFeatures{:}] = forward(dlnet,dlContent,Outputs=styleTransferOptions.contentFeatureLayerNames);

numStyleFeatureLayers = numel(styleTransferOptions.styleFeatureLayerNames);
styleFeatures = cell(1,numStyleFeatureLayers);
[styleFeatures{:}] = forward(dlnet,dlStyle,Outputs=styleTransferOptions.styleFeatureLayerNames);


figure

minimumLoss = inf;

for iteration = 1:numIterations
    % Evaluate the transfer image gradients and state using dlfeval and the
    % imageGradients function listed at the end of the example
    [grad,losses] = dlfeval(@imageGradients,dlnet,dlTransfer, ...
        contentFeatures,styleFeatures,styleTransferOptions);
    [dlTransfer,trailingAvg,trailingAvgSq] = adamupdate( ...
        dlTransfer,grad,trailingAvg,trailingAvgSq,iteration,learningRate);
  
    if losses.totalLoss < minimumLoss
        minimumLoss = losses.totalLoss;
        dlOutput = dlTransfer;        
    end   
    
    % Display the transfer image on the first iteration and after every 50
    % iterations. The postprocessing steps are described in the "Postprocess
    % Transfer Image for Display" section of this example
    if mod(iteration,50) == 0 || (iteration == 1)
        
        transferImage = gather(extractdata(dlTransfer));
        transferImage = transferImage + meanVggNet;
        transferImage = uint8(transferImage);
        transferImage = imresize(transferImage,size(contentImage,[1 2]));
        
        image(transferImage)
        title(["Transfer Image After Iteration ",num2str(iteration)])
        axis off image
        drawnow
    end   
    
end
