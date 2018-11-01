function pose_now = dead_reckon(pose_prev, delta_ticks, parameter)
baseline_ = 0.4456;
rad_per_tick_ = 2.0 * pi / 16384;
% Dead Reckoning
d1 = delta_ticks(1);
d2 = delta_ticks(2);

dth = (d2*parameter.left*rad_per_tick_ - d1*parameter.right*rad_per_tick_)/baseline_;
dist = (d1*parameter.left*rad_per_tick_ + d2*parameter.right*rad_per_tick_)/2;

pose_now.x = pose_prev.x + dist * cos(pose_prev.th + dth/2);
pose_now.y = pose_prev.y + dist * sin(pose_prev.th + dth/2);
pose_now.th = pose_prev.th + dth;

if (pose_now.th < -pi)
    pose_now.th = pose_now.th + 2 * pi;
end
if (pose_now.th > pi)
    pose_now.th = pose_now.th - 2 * pi;
end

end