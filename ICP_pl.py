
#使用point to plane ICP对给定两个点云进行配准，无visualisation,适合命令行批量处理
#使用方法：
#python ICP_pl.py source.pcd target.pcd initialisation.txt result.txt
#其中source是在配准过程中被移动的点云
#initialisation和result都是4x4的transformation矩阵
#注意，输出的结果就是最终的变换矩阵，无需再和initialisation叠加，evaluation时直接去和groundtruth做比较

import open3d as o3d
import sys
import numpy as np
import copy
import time

if __name__ == "__main__":   
    source=o3d.io.read_point_cloud(sys.argv[1])
    target=o3d.io.read_point_cloud(sys.argv[2])
    trans_init = np.loadtxt(sys.argv[3], usecols=range(4))
    resultFile = open(sys.argv[4], "w")    
    start = time.time()
    #correspondent点之间的距离最大值,default 0.02
    threshold = 20.0
    source.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=threshold * 2, max_nn=30))
    target.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=threshold * 2, max_nn=30))
    crit = o3d.pipelines.registration.ICPConvergenceCriteria()
    crit.max_iteration = 50
    crit.relative_rmse = 1e-04
    crit.relative_fitness = 1e-06
    result_ICP =  o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(), crit)
    print("Point to Plane ICP registration took %.3f sec.\n" % (time.time() - start))
    print(result_ICP)
    np.savetxt(resultFile, result_ICP.transformation, fmt='%.6f')
