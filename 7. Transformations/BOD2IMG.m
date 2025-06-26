%==========================================================================
% Niel Theron
% 19-06-2025
%==========================================================================
% The purpose of this function is to use the pinhole model to change the
% feature vector to a feature point on the image plane
%==========================================================================


function v_M = BOD2IMG(v_B,fl,imgHeight,imgWidth)
        
        % Scale
        v_M = (-fl/v_B(3:1))*v_B;
        %---

        % Move Origin
        v_M(1,:) = v_M(1,:) + imgWidth/2;
        v_M(1,:) = v_M(2,:) + imgHeight/2;
        %---


end

