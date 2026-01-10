 %upload image
ab = imread('y.jpg');
%converting to grayscale
abgray = im2gray(ab);
%brighntening image
newab= abgray + 20 ;
%gamma correcting (dark since gamma >1)
gamma = 1;
abadj= imadjust(abgray,[],[],gamma);
%inverting image
invert = fliplr(abgray);
%contrast stretching
%automatic contrast stretching
abs_auto = imadjust(ab,stretchlim(ab),[]);
%manual contrast stretching 
abs_manual = imadjust(ab,[0.1 0.9],[0 1]);
%negative transformation 
abneg = imcomplement(abgray);
%histogram equalisation by adjusting the no of bins
nbins = 5;
abhist= histeq(abadj, nbins);
abhistauto = histeq(abadj);
%plotting
figure; 
subplot(1,2,1);
imshow(abadj);
title('normal');
subplot(1,2,2); 
imshow(abgray);
title('automatic histogram eq');
%subplot(1,3,3); 
%imshow(abhist);
%title('manual histogram eq');

