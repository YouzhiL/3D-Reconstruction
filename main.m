clear;
clc;
close all

%%% read the 3 images and resize them. The 3 images will be unified to the
%%% size[512,512]
IMG1 = imread('1.jpeg');
IMG2 = imread('2.jpeg');
IMG3 = imread('3.jpeg');
IMG1 = imresize(IMG1,[512,512]);
IMG2 = imresize(IMG2,[512,512]);
IMG3 = imresize(IMG3,[512,512]);


%%%step1:input the parameters of the calibrated camera
my_camera = cameraIntrinsics([3210,3204],[2016,1473],[512,512],'RadialDistortion',[0.0039,-0.00038,0],'TangentialDistortion',[-0.00041,0.00306],'Skew',0);
intrinsic = [3210,0,0;0,3204,0;2016,1473,1];
focal_length = [3210,3204];


%%%step3: Undistort the image and transform it to grey-scale image
% IMG1 = undistortImag(IMG1,intrinsic,focal_length);
% IMG2 = undistortImag(IMG2,intrinsic,focal_length);
% IMG3 = undistortImag(IMG3,intrinsic,focal_length);
IMG1 = undistortImage(IMG1,my_camera);
IMG2 = undistortImage(IMG2,my_camera);
IMG3 = undistortImage(IMG3,my_camera);


IMG1 = rgb2gray(IMG1);
IMG2 = rgb2gray(IMG2);
IMG3 = rgb2gray(IMG3);


%%%step4:match geatures using SURF algorithm

%%% extract features from the 2 images
IMG1_points= detectSURFFeatures(IMG1);
[IMG1_Features,IMG1_points_new] = extractFeatures(IMG1,IMG1_points);
IMG2_points= detectSURFFeatures(IMG2);
[IMG2_Features,IMG2_points_new] = extractFeatures(IMG2,IMG2_points);

%%%return index and distance of the matched features£¨ratio = 0.7£©
[index_12,dist12] = matchFeatures(IMG1_Features,IMG2_Features,'Maxratio',0.7); 
%[indexPairs_13,dist13] = matchFeatures(IMG1_Features,IMG3_Features,'Maxratio',0.7);

points_12_IMG1 = IMG1_points_new(index_12(:,1),:);
points_12_IMG2 = IMG2_points_new(index_12(:,2),:);
%matchedPts1_13 = IMG1_points_new(indexPairs_13(:,1),:);
%matchedPts3_13 = IMG3_points_new(indexPairs_13(:,2),:);


%%% show matched features after SURF
figure;
showMatchedFeatures(IMG1,IMG2,points_12_IMG1,points_12_IMG2,'montage');
title('camera1-camera2 matched features');




%%%%step5: obtain fundamental matrix using RANSAC
Num = 3000; 

[F,inliers] = estimateFundamentalMatrix(points_12_IMG1,points_12_IMG2,'Method','RANSAC','NumTrials',Num,'DistanceThreshold',1e-4);

% obtain inliers
inliers_1 = points_12_IMG1(inliers);
inliers_2 = points_12_IMG2(inliers);
% obtain essential matrix using inliers
[E,~] = estimateEssentialMatrix(inliers_1,inliers_2,my_camera);

figure;
showMatchedFeatures(IMG1,IMG2,inliers_1,inliers_2,'montage');
title('camera1-camera2 matched inliers');



%%%%step6:obtain rotaion matrix and transition matrix using essential
%%%%matrix

%%%%estimate relativeCameraPose(orientation + location)
[ori2_1,loc2_1] = relativeCameraPose(E,my_camera,inliers_1,inliers_2);

%[re_rot2_1,re_trans2_1] = extract_R_t(E);
[re_rot2_1,re_trans2_1] = cameraPoseToExtrinsics(ori2_1,loc2_1);

ori1_1 = [1,0,0;0,1,0;0,0,1];
loc1_1 = [0,0,0];
[re_rot1_1,re_trans1_1] = cameraPoseToExtrinsics(ori1_1,loc1_1);


%%%%%step7:3D reconstruction using triangulation
%%%%%first compute camera matrix
M_camera1 = cameraMatrix(my_camera,re_rot1_1,re_trans1_1);
M_camera2 = cameraMatrix(my_camera,re_rot2_1,re_trans2_1);

%%%%% compute 3D coordination using camera matrix
[points_3D,err] = triangulate(points_12_IMG1,points_12_IMG2,M_camera1,M_camera2);



%%%%%step8:3D-2D Registration

IMG3_points= detectSURFFeatures(IMG3);
[IMG3_Features,IMG3_points_new] = extractFeatures(IMG3,IMG3_points);
[index_13,dist13] = matchFeatures(IMG1_Features,IMG3_Features,'Maxratio',0.7);
points_13_IMG1 = IMG1_points_new(index_13(:,1),:);
points_13_IMG3 = IMG3_points_new(index_13(:,2),:);

[asd,index12_1,index13_1] = intersect(index_12(:,1),index_13(:,1));

points_3D_IMG3 = points_3D(index12_1,:);

points_2D_IMG3 = points_13_IMG3(index13_1,:).Location;


%%%%%step9:pose estimation using RANSAC or M-estimator

[ori3,loc3] = estimateWorldCameraPose(points_2D_IMG3,points_3D_IMG3,my_camera);


[re_rot3_1,re_trans3_1] = cameraPoseToExtrinsics(ori3,loc3)


[E_13,inliers] = estimateEssentialMatrix(points_13_IMG1,points_13_IMG3,my_camera);
inliers1 = points_13_IMG1(inliers);
inliers3 = points_13_IMG3(inliers);
[ori3_1,loc3_1] = relativeCameraPose(E_13,my_camera,inliers1,inliers3);
[re_rot3_1_dir,re_trans3_1_dir] = cameraPoseToExtrinsics(ori3_1,loc3_1);
figure;
plot3(points_3D(:,1),points_3D(:,2),points_3D(:,3),'.',re_trans3_1(1),re_trans3_1(2),re_trans3_1(3),'*',re_trans2_1(1),re_trans2_1(2),re_trans2_1(3),'*',re_trans1_1(1),re_trans1_1(2),re_trans1_1(3),'*');
grid on;










