function overlap_region = robot_state(limb_A, limb_B, limb_C, limb_D, state_val)

% if state_val == 1
%     overlap_region = (limb_A == 0) & (limb_B == 0) & (limb_C == 0) & (limb_D == 0);
% end
%
% if state_val == 2
%     overlap_region = (limb_A == 0) & (limb_B == 0) & (limb_C == 0) & (limb_D == 1);
% end

switch state_val

    case 1
        overlap_region = (limb_A == 0) & (limb_B == 0) & (limb_C == 0) & (limb_D == 0);

    case 2
        overlap_region = (limb_A == 0) & (limb_B == 0) & (limb_C == 0) & (limb_D == 0);

    case 3
        overlap_region = (limb_A == 0) & (limb_B == 0) & (limb_C == 0) & (limb_D == 0);

    case 4
        overlap_region = (limb_A == 0) & (limb_B == 0) & (limb_C == 0) & (limb_D == 0);

    case 5
        overlap_region = (limb_A == 0) & (limb_B == 0) & (limb_C == 0) & (limb_D == 0);

    case 6
        overlap_region = (limb_A == 0) & (limb_B == 0) & (limb_C == 0) & (limb_D == 0);

    case 7
        overlap_region = (limb_A == 0) & (limb_B == 0) & (limb_C == 0) & (limb_D == 0);

    case 8
        overlap_region = (limb_A == 0) & (limb_B == 0) & (limb_C == 0) & (limb_D == 0);

    case 9
        overlap_region = (limb_A == 0) & (limb_B == 0) & (limb_C == 0) & (limb_D == 0);

    case 10
        overlap_region = (limb_A == 0) & (limb_B == 0) & (limb_C == 0) & (limb_D == 0);

    case 11
        overlap_region = (limb_A == 0) & (limb_B == 0) & (limb_C == 0) & (limb_D == 0);

    case 12
        overlap_region = (limb_A == 0) & (limb_B == 0) & (limb_C == 0) & (limb_D == 0);

    case 13
        overlap_region = (limb_A == 0) & (limb_B == 0) & (limb_C == 0) & (limb_D == 0);

    case 14
        overlap_region = (limb_A == 0) & (limb_B == 0) & (limb_C == 0) & (limb_D == 0);

    case 15
        overlap_region = (limb_A == 0) & (limb_B == 0) & (limb_C == 0) & (limb_D == 0);

    case 16
        overlap_region = (limb_A == 0) & (limb_B == 0) & (limb_C == 0) & (limb_D == 0);

    otherwise
        disp('Invalid robot state');
end

end