function [b]=Rot2Quat(R)
% Convert DCM2Quat

R = R +  (eye(3) - R*R') * 0.5 * R;

M1 = 1 + R(1,1) + R(2,2) + R(3,3);
M2 = 1 + R(1,1) - R(2,2) - R(3,3);
M3 = 1 - R(1,1) + R(2,2) - R(3,3);
M4 = 1 - R(1,1) - R(2,2) + R(3,3);
b = zeros(4,1);

if( (M1 > M2) && (M1 > M3)  &&  (M1 > M4))
    
    if(M1 > 0)
        b(1,1)    = (0.5*sqrt(M1));
        b(2,1)    = ( (R(3,2)-R(2,3))/(4*b(1)) );
        b(3,1)    = ( (R(1,3)-R(3,1))/(4*b(1)) );
        b(4,1)    = ( (R(2,1)-R(1,2))/(4*b(1)) );
        b         = (b/norm(b,'fro')); % renormalize
    else
        error('M1 won under 0');
    end
    
elseif( (M2 > M3) && (M2 > M4))
    
    if(M2 > 0)
        b(2,1) = 0.5*sqrt(M2);
        b(3,1) = (R(2,1) + R(1,2))/(4*b(2));
        b(4,1) = (R(3,1) + R(1,3))/(4*b(2));
        b(1,1) = (R(3,2) - R(2,3))/(4*b(2));
        b      = (b/norm(b,'fro')); % renormalize
    else
        error('M2 won under 0');
    end
    
elseif( M3 > M4 )
    
    
    if(M3 > 0)
        b(3,1) = 0.5*sqrt(M3);
        
        b(2,1) = (R(1,2) + R(2,1))/(4*b(3));
        b(4,1) = (R(3,2) + R(2,3))/(4*b(3));
        b(1,1) = (R(1,3) - R(3,1))/(4*b(3));
        b      = (b/norm(b,'fro')); % renormalize
    else
        error('M3 won under 0');
    end
    
else
    
    if(M4 > 0)
        b(4,1) = 0.5*sqrt(M4);               
        
        b(2,1) = (R(1,3) + R(3,1))/(4*b(4));
        b(3,1) = (R(2,3) + R(3,2))/(4*b(4));
        b(1,1) = (R(2,1) - R(1,2))/(4*b(4));
        b      = (b/norm(b,'fro')); % renormalize
    else
        error('M4 won under 0');
    end
    
end

end

