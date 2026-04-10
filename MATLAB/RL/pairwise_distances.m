function dists = pairwise_distances(r1_xyz, r2_xyz, kuka_xyz)
%PAIRWISE_DISTANCES Compute all pairwise end-effector distances.
%
%   dists = pairwise_distances(r1_xyz, r2_xyz, kuka_xyz)
%
%   Inputs:
%     r1_xyz   - [1x3] RoArm 1 measured position [x, y, z] in mm
%     r2_xyz   - [1x3] RoArm 2 measured position [x, y, z] in mm
%     kuka_xyz - [1x3] KUKA TCP measured position [x, y, z] in mm
%
%   Output:
%     dists - struct with fields:
%       .r1_r2     - distance between RoArm 1 and RoArm 2
%       .r1_kuka   - distance between RoArm 1 and KUKA TCP
%       .r2_kuka   - distance between RoArm 2 and KUKA TCP
%       .min_dist  - minimum of all three distances
%       .r1_r2_vec - [1x3] direction vector from RoArm 1 to RoArm 2
%       .r1_k_vec  - [1x3] direction vector from RoArm 1 to KUKA
%       .r2_k_vec  - [1x3] direction vector from RoArm 2 to KUKA

    dists = struct();

    % Direction vectors (unnormalized, in mm)
    dists.r1_r2_vec = r2_xyz - r1_xyz;
    dists.r1_k_vec  = kuka_xyz - r1_xyz;
    dists.r2_k_vec  = kuka_xyz - r2_xyz;

    % Euclidean distances
    dists.r1_r2   = norm(dists.r1_r2_vec);
    dists.r1_kuka = norm(dists.r1_k_vec);
    dists.r2_kuka = norm(dists.r2_k_vec);

    dists.min_dist = min([dists.r1_r2, dists.r1_kuka, dists.r2_kuka]);
end