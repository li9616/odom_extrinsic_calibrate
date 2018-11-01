function full_path = dead_reckon_full_path(encoder_meas, parameter) 
pose.x = 0;
pose.y = 0;
pose.th = 0;
full_path = [pose.x, pose.y, pose.th];

diff_encoder_meas = diff(encoder_meas);

for i = 1: size(diff_encoder_meas,1)
    pose = dead_reckon(pose,diff_encoder_meas(i,:), parameter);
    full_path = [full_path;
        pose.x, pose.y, pose.th];
end

end