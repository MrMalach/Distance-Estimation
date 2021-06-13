
%%---------------------------------------
%       Digital Image Processing
%    Vehicle Distance Estimation
%             Ran Malach ® 
%%---------------------------------------
%% Cam Calibration
clear all; close all;
addpath('CalibrationPics\L');
addpath('CalibrationPics\R');
names.L={'L1.jpg','L2.jpg','L3.jpg','L4.jpg','L5.jpg',...
    'L6.jpg','L7.jpg','L8.jpg','L9.jpg','L10.jpg',...
    'L11.jpg','L12.jpg','L13.jpg','L14.jpg','L15.jpg',...
    'L16.jpg','L17.jpg','L18.jpg','L19.jpg','L20.jpg'};
names.R={'R1.jpg','R2.jpg','R3.jpg','R4.jpg','R5.jpg',...
    'R6.jpg','R7.jpg','R8.jpg','R9.jpg','R10.jpg',...
    'R11.jpg','R12.jpg','R13.jpg','R14.jpg','R15.jpg',...
    'R16.jpg','R17.jpg','R18.jpg','R19.jpg','R20.jpg'};
% camList = webcamlist;
Cam.L=webcam(3);
Cam.R=webcam(2);
preview(Cam.L);
preview(Cam.R);
%% Take images loop
for n=1:length(names.L)
    pause;
    imwrite(snapshot(Cam.L),['CalibrationPics\L\' names.L{n}]);
end
for n=1:length(names.L)
    pause;
    imwrite(snapshot(Cam.R),['CalibrationPics\R\' names.R{n}]);
end
save('LcameraParams','LcameraParams');
save('RcameraParams','RcameraParams');
%% Program start
close all; clear all;
load('RcameraParams');
load('LcameraParams');
figure(1);
D = .2;   % The Distance between the two cams -> 20 cm
camList = webcamlist;
Cam.L=webcam(2);
Cam.R=webcam(1);%preview(Cam.L);preview(Cam.R);
%%
while(1)
    tic;%pause(3);
    img.L=snapshot(Cam.L);
    img.R=snapshot(Cam.R);
    % Create masks for red high brightness values
    mask.L.bw=img.L(:,:,1)>252 & img.L(:,:,2)<254 & img.L(:,:,3)<254 ;
    mask.R.bw=img.R(:,:,1)>252 & img.R(:,:,2)<254 & img.R(:,:,3)<254 ;
    % Check connectivity in order to choose only the largest area found.
    mask.L.bwfilt=bwareafilt(mask.L.bw,1);
    mask.R.bwfilt=bwareafilt(mask.R.bw,1);
    % find all points of value in masks.
    [R.Y.f,R.X.f]=find(mask.R.bwfilt);
    [L.Y.f,L.X.f]=find(mask.L.bwfilt);
    % Mean values for approx center.
    R.X.m=mean(R.X.f);
    L.X.m=mean(L.X.f);
    R.Y.m=mean(R.Y.f);
    L.Y.m=mean(L.Y.f);
    % Find the distance between the Pixel found and the center of the image.
    R.r1=sqrt((R.X.m-RcameraParams.ImageSize(2)/2)^2+...
            (R.Y.m-RcameraParams.ImageSize(1)/2)^2);
    L.r2=sqrt((L.X.m-LcameraParams.ImageSize(2)/2)^2+...
            (L.Y.m-LcameraParams.ImageSize(1)/2)^2);
    % a=sqrt(LcameraParams.FocalLength(1)^2+r1^2);
    % b=sqrt(RcameraParams.FocalLength(1)^2+r2^2);
    % Find the angles between the focal lengths and the distances r1,r2.
    % equation (5)
    R.alfa=atan(R.r1/RcameraParams.FocalLength(1));
    L.beta=atan(L.r2/LcameraParams.FocalLength(1));
    % The projection of r1,r2 on the major axis.
    R.p1=R.X.m-LcameraParams.ImageSize(2);
    L.p2=L.X.m-LcameraParams.ImageSize(2);
    % Get the vertical distance between the LED and the Image Sensor.
    % equation (4)
    h=D*RcameraParams.FocalLength(1)/(L.p2-R.p1);
    % Get the distances between each lens and the LED. equation (6)
    R.d1=h/cos(R.alfa);
    L.d2=h/cos(L.beta);
    % The distance to LED, the end result equation (7)
    dA=sqrt((2*(R.d1^2+L.d2^2)-D^2)/4);
    % ----------------------------------------------------------------
    subplot 121; imshow(img.L);title('Left');hold on
    plot(L.X.m,L.Y.m,'s','color','red','markersize',10,'LineWidth',1);
    hold off;
    subplot 122; imshow(img.R);title('Right');hold on
    plot(R.X.m,R.Y.m,'s','color','red','markersize',10,'LineWidth',1);
    hold off;
    c=suptitle(['LED Distance =',num2str(dA,3),'[m]']);
    set(c,'fontsize',40);
    toc
    pause(.75);
end



