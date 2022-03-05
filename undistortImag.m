function undistorted_img = undistortImag(img, K, D)
% Corrects an image for lens distortion.

 
[height, width] = size(img);
 
fx = K(1,1);
fy = K(2,2);
cx = K(1,3);
cy = K(2,3);
 
 
%Finish image undistorted function
undistorted_img = uint8(zeros(height, width));
 

for y = 1:height
    for x = 1:width
 
            
        x1 = (x-cx)/fx;  
        y1 = (y-cy)/fy;
        
        
        r2 = x1^2+y1^2;
        x2  = x1*(1+D(1)*r2+D(2)*r2^2); 
        y2 = y1*(1+D(1)*r2+D(2)*r2^2);
        
       
        u_distorted = fx*x2+cx;
        v_distorted = fy*y2+cy;
        
        v_distorted = round(v_distorted);
        u_distorted = round(u_distorted);
        if (u_distorted > 0 && v_distorted > 0 && u_distorted < width && v_distorted < height)
            undistorted_img(y, x) = img(v_distorted, u_distorted); 
        else
            undistorted_img(y, x) = 0;
        end
    end
end
 
end
