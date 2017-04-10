# Unscented-Kalman-Filter-UKF

In Data 2, the starting lidar measurements for x, y are both zero, and this special case can create problems for both the EKF and UKF lidar update states. One way to catch for this is to observe when both px, py are zero and instead set them to some small floating value.

Your algorithm will be run against "sample-laser-radar-measurement-data-1.txt". We'll collect the positions that your algorithm outputs and compare them to ground truth data. Your px, py, vx, and vy RMSE should be less than or equal to the values [0.09, 0.09, 0.65, 0.65].

Your algorithm will be run against "sample-laser-radar-measurement-data-2.txt". We'll collect the positions that your algorithm outputs and compare them to ground truth data. Your px, py, vx, and vy RMSE should be less than or equal to the values [0.20, 0.20, 0.55, 0.55].

