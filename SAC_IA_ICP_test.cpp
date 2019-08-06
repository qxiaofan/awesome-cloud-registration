#include <pcl/registration/ia_ransac.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <time.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

//点云可视化
void visualize_pcd(PointCloud::Ptr pcd_src,
                   PointCloud::Ptr pcd_tgt,
                   PointCloud::Ptr pcd_final)
{
    // Create a PCLVisualizer object
    pcl::visualization::PCLVisualizer viewer("registration Viewer");
    //viewer.createViewPort (0.0, 0, 0.5, 1.0, vp_1);
    // viewer.createViewPort (0.5, 0, 1.0, 1.0, vp_2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h (pcd_src, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h (pcd_tgt, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_h (pcd_final, 0, 0, 255);
    viewer.addPointCloud (pcd_src, src_h, "source cloud");
    viewer.addPointCloud (pcd_tgt, tgt_h, "tgt cloud");
    viewer.addPointCloud (pcd_final, final_h, "final cloud");
    //viewer.addCoordinateSystem(1.0);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(10000));
    }
}

//由旋转平移矩阵计算旋转角度
void Matrix2Angle (Eigen::Matrix4f &result_trans,Eigen::Vector3f &result_angle)
{
    double ax,ay,az;
    if (result_trans(2,0)==1 || result_trans(2,0)==-1)
    {
        az=0;
        double dlta;
        dlta=atan2(result_trans(0,1),result_trans(0,2));
        if (result_trans(2,0)==-1)
        {
            ay=M_PI/2;
            ax=az+dlta;
        }
        else
        {
            ay=-M_PI/2;
            ax=-az+dlta;
        }
    }
    else
    {
        ay=-asin(result_trans(2,0));
        ax=atan2(result_trans(2,1)/cos(ay),result_trans(2,2)/cos(ay));
        az=atan2(result_trans(1,0)/cos(ay),result_trans(0,0)/cos(ay));
    }
    result_angle<<ax,ay,az;
}

////src
//void doRegistration(std::string src_cloud_path,std::string tgt_cloud_path, Eigen::Matrix4f &icp_trans)
//{
//    //加载点云文件
//    PointCloud::Ptr cloud_src_origin (new PointCloud);//原点云，待配准
//    pcl::io::loadPCDFile (src_cloud_path,*cloud_src_origin);
//    PointCloud::Ptr cloud_tgt_origin (new PointCloud);//目标点云
//    pcl::io::loadPCDFile (tgt_cloud_path,*cloud_tgt_origin);
//
//    clock_t start=clock();
//    //去除NAN点
//    std::vector<int> indices_src; //保存去除的点的索引
//    pcl::removeNaNFromPointCloud(*cloud_src_origin,*cloud_src_origin, indices_src);
//    std::cout<<"remove *cloud_src_origin nan"<<endl;
//
//    //下采样滤波
//    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
//    voxel_grid.setLeafSize(0.014,0.014,0.014);
//    voxel_grid.setInputCloud(cloud_src_origin);
//    PointCloud::Ptr cloud_src (new PointCloud);
//    voxel_grid.filter(*cloud_src);
//    std::cout<<"down size *cloud_src_origin from "<<cloud_src_origin->size()<<"to"<<cloud_src->size()<<endl;
//    //pcl::io::savePCDFileASCII("monkey_src_down.pcd",*cloud_src);
//
//    //计算表面法线
//    pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne_src;
//    ne_src.setInputCloud(cloud_src);
//    pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_src(new pcl::search::KdTree< pcl::PointXYZ>());
//    ne_src.setSearchMethod(tree_src);
//    pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud< pcl::Normal>);
//    ne_src.setRadiusSearch(0.02);
//    ne_src.compute(*cloud_src_normals);
//
//    std::vector<int> indices_tgt;
//    pcl::removeNaNFromPointCloud(*cloud_tgt_origin,*cloud_tgt_origin, indices_tgt);
//    std::cout<<"remove *cloud_tgt_origin nan"<<endl;
//
//    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_2;
//    voxel_grid_2.setLeafSize(0.01,0.01,0.01);
//    voxel_grid_2.setInputCloud(cloud_tgt_origin);
//    PointCloud::Ptr cloud_tgt (new PointCloud);
//    voxel_grid_2.filter(*cloud_tgt);
//    std::cout<<"down size *cloud_tgt_origin.pcd from "<<cloud_tgt_origin->size()<<"to"<<cloud_tgt->size()<<endl;
//    //pcl::io::savePCDFileASCII("monkey_tgt_down.pcd",*cloud_tgt);
//
//    pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne_tgt;
//    ne_tgt.setInputCloud(cloud_tgt);
//    pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_tgt(new pcl::search::KdTree< pcl::PointXYZ>());
//    ne_tgt.setSearchMethod(tree_tgt);
//    pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud< pcl::Normal>);
//    //ne_tgt.setKSearch(20);
//    ne_tgt.setRadiusSearch(0.02);
//    ne_tgt.compute(*cloud_tgt_normals);
//
//    //计算FPFH
//    pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> fpfh_src;
//    fpfh_src.setInputCloud(cloud_src);
//    fpfh_src.setInputNormals(cloud_src_normals);
//    pcl::search::KdTree<PointT>::Ptr tree_src_fpfh (new pcl::search::KdTree<PointT>);
//    fpfh_src.setSearchMethod(tree_src_fpfh);
//    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>());
//    fpfh_src.setRadiusSearch(0.05);
//    fpfh_src.compute(*fpfhs_src);
//    std::cout<<"compute *cloud_src fpfh"<<endl;
//
//    pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> fpfh_tgt;
//    fpfh_tgt.setInputCloud(cloud_tgt);
//    fpfh_tgt.setInputNormals(cloud_tgt_normals);
//    pcl::search::KdTree<PointT>::Ptr tree_tgt_fpfh (new pcl::search::KdTree<PointT>);
//    fpfh_tgt.setSearchMethod(tree_tgt_fpfh);
//    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::FPFHSignature33>());
//    fpfh_tgt.setRadiusSearch(0.05);
//    fpfh_tgt.compute(*fpfhs_tgt);
//    std::cout<<"compute *cloud_tgt fpfh"<<endl;
//
//    //SAC配准
//    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> scia;
//    scia.setInputSource(cloud_src);
//    scia.setInputTarget(cloud_tgt);
//    scia.setSourceFeatures(fpfhs_src);
//    scia.setTargetFeatures(fpfhs_tgt);
//    //scia.setMinSampleDistance(1);
//    //scia.setNumberOfSamples(2);
//    //scia.setCorrespondenceRandomness(20);
//    PointCloud::Ptr sac_result (new PointCloud);
//    scia.align(*sac_result);
//    std::cout  <<"sac has converged:"<<scia.hasConverged()<<"  score: "<<scia.getFitnessScore()<<endl;
//    Eigen::Matrix4f sac_trans;
//    sac_trans=scia.getFinalTransformation();
//    std::cout<<"sac_trans== "<<sac_trans<<endl;
//    //pcl::io::savePCDFileASCII("monkey_transformed_sac.pcd",*sac_result);
//    clock_t sac_time=clock();
//
//    //icp配准
//    PointCloud::Ptr icp_result (new PointCloud);
//    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//    icp.setInputSource(cloud_src);
//    icp.setInputTarget(cloud_tgt_origin);
//    icp.setMaxCorrespondenceDistance (0.04);
//    // 最大迭代次数
//    icp.setMaximumIterations (15);
//    // 两次变化矩阵之间的差值
//    icp.setTransformationEpsilon (1e-10);
//    // 均方误差
//    icp.setEuclideanFitnessEpsilon (0.2);
//    icp.align(*icp_result,sac_trans);
//
//    clock_t end=clock();
//    cout<<"total time: "<<(double)(end-start)/(double)CLOCKS_PER_SEC<<" s"<<endl;
//
//    cout<<"sac time: "<<(double)(sac_time-start)/(double)CLOCKS_PER_SEC<<" s"<<endl;
//    cout<<"icp time: "<<(double)(end-sac_time)/(double)CLOCKS_PER_SEC<<" s"<<endl;
//
//    std::cout << "ICP has converged:" << icp.hasConverged()
//              << " score: " << icp.getFitnessScore() << std::endl;
//
//    icp_trans=icp.getFinalTransformation();
//    //cout<<"ransformationProbability"<<icp.getTransformationProbability()<<endl;
//    std::cout<<"icp_trans== "<<icp_trans<<endl;
//    //使用创建的变换对未过滤的输入点云进行变换
//    pcl::transformPointCloud(*cloud_src_origin, *icp_result, icp_trans);
//    //保存转换的输入点云
//    pcl::io::savePCDFileASCII("monkey_transformed_sac_ndt.pcd", *icp_result);
//
//    //计算误差
//    Eigen::Vector3f ANGLE_origin;
//    ANGLE_origin<<0,0,M_PI/5;
//    double error_x,error_y,error_z;
//    Eigen::Vector3f ANGLE_result;
//    Matrix2Angle(icp_trans,ANGLE_result);
//    error_x=fabs(ANGLE_result(0))-fabs(ANGLE_origin(0));
//    error_y=fabs(ANGLE_result(1))-fabs(ANGLE_origin(1));
//    error_z=fabs(ANGLE_result(2))-fabs(ANGLE_origin(2));
//    cout<<"original angle in x y z:\n"<<ANGLE_origin<<endl;
//    cout<<"error in aixs_x: "<<error_x<<"  error in aixs_y: "<<error_y<<"  error in aixs_z: "<<error_z<<endl;
//    //可视化
//    visualize_pcd(cloud_src_origin,cloud_tgt_origin,icp_result);
//}


//src
void doRegistration(std::string src_cloud_path,std::string tgt_cloud_path, Eigen::Matrix4f &icp_trans)
{
    //加载点云文件
    PointCloud::Ptr cloud_src_origin (new PointCloud);//原点云，待配准
    pcl::io::loadPCDFile (src_cloud_path,*cloud_src_origin);
    PointCloud::Ptr cloud_tgt_origin (new PointCloud);//目标点云
    pcl::io::loadPCDFile (tgt_cloud_path,*cloud_tgt_origin);

    clock_t start=clock();
    //去除NAN点
    std::vector<int> indices_src; //保存去除的点的索引
    pcl::removeNaNFromPointCloud(*cloud_src_origin,*cloud_src_origin, indices_src);
    std::cout<<"remove *cloud_src_origin nan"<<endl;

    //对源点云进行直通滤波
    pcl::PassThrough<pcl::PointXYZ> pass_src; //设置滤波器对象
    pass_src.setInputCloud(cloud_src_origin);
    pass_src.setFilterFieldName("z");
    pass_src.setFilterLimits(0.5,2.0); //设置在过滤字段上的范围
    pass_src.setFilterLimitsNegative(false); //决定是否保留范围内的
    pass_src.filter(*cloud_src_origin);


    //对目标点云进行直通滤波
    pcl::PassThrough<pcl::PointXYZ> pass_tgt; //设置滤波器对象
    pass_src.setInputCloud(cloud_tgt_origin);
    pass_src.setFilterFieldName("z");
    pass_src.setFilterLimits(0.5,2.0); //设置在过滤字段上的范围
    pass_src.setFilterLimitsNegative(false); //决定是否保留范围内的
    pass_src.filter(*cloud_tgt_origin);


    //下采样滤波
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setLeafSize(0.014,0.014,0.014);
    voxel_grid.setInputCloud(cloud_src_origin);
    PointCloud::Ptr cloud_src (new PointCloud);
    voxel_grid.filter(*cloud_src);
    std::cout<<"down size *cloud_src_origin from "<<cloud_src_origin->size()<<"to"<<cloud_src->size()<<endl;
    //pcl::io::savePCDFileASCII("monkey_src_down.pcd",*cloud_src);

    //计算表面法线
    pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne_src;
    ne_src.setInputCloud(cloud_src);
    pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_src(new pcl::search::KdTree< pcl::PointXYZ>());
    ne_src.setSearchMethod(tree_src);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud< pcl::Normal>);
    ne_src.setRadiusSearch(0.02);
    ne_src.compute(*cloud_src_normals);

    std::vector<int> indices_tgt;
    pcl::removeNaNFromPointCloud(*cloud_tgt_origin,*cloud_tgt_origin, indices_tgt);
    std::cout<<"remove *cloud_tgt_origin nan"<<endl;

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_2;
    voxel_grid_2.setLeafSize(0.01,0.01,0.01);
    voxel_grid_2.setInputCloud(cloud_tgt_origin);
    PointCloud::Ptr cloud_tgt (new PointCloud);
    voxel_grid_2.filter(*cloud_tgt);
    std::cout<<"down size *cloud_tgt_origin.pcd from "<<cloud_tgt_origin->size()<<"to"<<cloud_tgt->size()<<endl;
    //pcl::io::savePCDFileASCII("monkey_tgt_down.pcd",*cloud_tgt);

    pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne_tgt;
    ne_tgt.setInputCloud(cloud_tgt);
    pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_tgt(new pcl::search::KdTree< pcl::PointXYZ>());
    ne_tgt.setSearchMethod(tree_tgt);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud< pcl::Normal>);
    //ne_tgt.setKSearch(20);
    ne_tgt.setRadiusSearch(0.02);
    ne_tgt.compute(*cloud_tgt_normals);

    //计算FPFH
    pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> fpfh_src;
    fpfh_src.setInputCloud(cloud_src);
    fpfh_src.setInputNormals(cloud_src_normals);
    pcl::search::KdTree<PointT>::Ptr tree_src_fpfh (new pcl::search::KdTree<PointT>);
    fpfh_src.setSearchMethod(tree_src_fpfh);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>());
    fpfh_src.setRadiusSearch(0.05);
    fpfh_src.compute(*fpfhs_src);
    std::cout<<"compute *cloud_src fpfh"<<endl;

    pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> fpfh_tgt;
    fpfh_tgt.setInputCloud(cloud_tgt);
    fpfh_tgt.setInputNormals(cloud_tgt_normals);
    pcl::search::KdTree<PointT>::Ptr tree_tgt_fpfh (new pcl::search::KdTree<PointT>);
    fpfh_tgt.setSearchMethod(tree_tgt_fpfh);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::FPFHSignature33>());
    fpfh_tgt.setRadiusSearch(0.05);
    fpfh_tgt.compute(*fpfhs_tgt);
    std::cout<<"compute *cloud_tgt fpfh"<<endl;

    //SAC配准
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> scia;
    scia.setInputSource(cloud_src);
    scia.setInputTarget(cloud_tgt);
    scia.setSourceFeatures(fpfhs_src);
    scia.setTargetFeatures(fpfhs_tgt);
    //scia.setMinSampleDistance(1);
    //scia.setNumberOfSamples(2);
    //scia.setCorrespondenceRandomness(20);
    PointCloud::Ptr sac_result (new PointCloud);
    scia.align(*sac_result);
    std::cout  <<"sac has converged:"<<scia.hasConverged()<<"  score: "<<scia.getFitnessScore()<<endl;
    Eigen::Matrix4f sac_trans;
    sac_trans=scia.getFinalTransformation();
    std::cout<<"sac_trans== "<<sac_trans<<endl;
    //pcl::io::savePCDFileASCII("monkey_transformed_sac.pcd",*sac_result);
    clock_t sac_time=clock();

    //icp配准
    PointCloud::Ptr icp_result (new PointCloud);
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_src);
    icp.setInputTarget(cloud_tgt_origin);
    icp.setMaxCorrespondenceDistance (0.04);
    // 最大迭代次数
    icp.setMaximumIterations (5);
    // 两次变化矩阵之间的差值
    icp.setTransformationEpsilon (1e-10);
    // 均方误差
    icp.setEuclideanFitnessEpsilon (0.2);
    icp.align(*icp_result,sac_trans);

    clock_t end=clock();
    cout<<"total time: "<<(double)(end-start)/(double)CLOCKS_PER_SEC<<" s"<<endl;

    cout<<"sac time: "<<(double)(sac_time-start)/(double)CLOCKS_PER_SEC<<" s"<<endl;
    cout<<"icp time: "<<(double)(end-sac_time)/(double)CLOCKS_PER_SEC<<" s"<<endl;

    std::cout << "ICP has converged:" << icp.hasConverged()
              << " score: " << icp.getFitnessScore() << std::endl;

    icp_trans=icp.getFinalTransformation();
    //cout<<"ransformationProbability"<<icp.getTransformationProbability()<<endl;
    std::cout<<"icp_trans== "<<icp_trans<<endl;
    //使用创建的变换对未过滤的输入点云进行变换
    pcl::transformPointCloud(*cloud_src_origin, *icp_result, icp_trans);
    //保存转换的输入点云
    pcl::io::savePCDFileASCII("monkey_transformed_sac_ndt.pcd", *icp_result);

    //计算误差
    Eigen::Vector3f ANGLE_origin;
    ANGLE_origin<<0,0,M_PI/5;
    double error_x,error_y,error_z;
    Eigen::Vector3f ANGLE_result;
    Matrix2Angle(icp_trans,ANGLE_result);
    error_x=fabs(ANGLE_result(0))-fabs(ANGLE_origin(0));
    error_y=fabs(ANGLE_result(1))-fabs(ANGLE_origin(1));
    error_z=fabs(ANGLE_result(2))-fabs(ANGLE_origin(2));
    cout<<"original angle in x y z:\n"<<ANGLE_origin<<endl;
    cout<<"error in aixs_x: "<<error_x<<"  error in aixs_y: "<<error_y<<"  error in aixs_z: "<<error_z<<endl;
    //可视化
    visualize_pcd(cloud_src_origin,cloud_tgt_origin,icp_result);
}






//int
//main (int argc, char** argv)
//{
//    std::string src_cloud_path="../Depth_00052258.pcd";
//    std::string tgt_cloud_path="../Depth_00053716.pcd";
//
//    Eigen::Matrix4f icp_trans;
//
//    doRegistration(src_cloud_path,tgt_cloud_path,icp_trans);
//    return (0);
//}


//yong.qi reversed
//with
void doRegistration(std::string src_cloud_path,std::string tgt_cloud_path, Eigen::Matrix4f &sac_trans,Eigen::Matrix4f &icp_trans)
{
    //加载点云文件
    PointCloud::Ptr cloud_src_origin (new PointCloud);//原点云，待配准
    pcl::io::loadPCDFile (src_cloud_path,*cloud_src_origin);
    PointCloud::Ptr cloud_tgt_origin (new PointCloud);//目标点云
    pcl::io::loadPCDFile (tgt_cloud_path,*cloud_tgt_origin);

    clock_t start=clock();
    //去除源点云NAN点
    std::vector<int> indices_src; //保存去除的点的索引
    pcl::removeNaNFromPointCloud(*cloud_src_origin,*cloud_src_origin, indices_src);
    std::cout<<"remove *cloud_src_origin nan"<<endl;

    //去除目标点云NAN点
    std::vector<int> indices_tgt;
    pcl::removeNaNFromPointCloud(*cloud_tgt_origin,*cloud_tgt_origin,indices_tgt);
    std::cout<<"remove *cloud_tgt_orgin nan"<<endl;


    //对源点云进行直通滤波
    pcl::PassThrough<pcl::PointXYZ> pass_src; //设置滤波器对象
    pass_src.setInputCloud(cloud_src_origin);
    pass_src.setFilterFieldName("z");
    pass_src.setFilterLimits(0.5,2.0); //设置在过滤字段上的范围
    pass_src.setFilterLimitsNegative(false); //决定是否保留范围内的
    pass_src.filter(*cloud_src_origin);


    //对目标点云进行直通滤波
    pcl::PassThrough<pcl::PointXYZ> pass_tgt; //设置滤波器对象
    pass_src.setInputCloud(cloud_tgt_origin);
    pass_src.setFilterFieldName("z");
    pass_src.setFilterLimits(0.5,2.0); //设置在过滤字段上的范围
    pass_src.setFilterLimitsNegative(false); //决定是否保留范围内的
    pass_src.filter(*cloud_tgt_origin);



    //下采样滤波
    //pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
   // voxel_grid.setLeafSize(0.014,0.014,0.014);
   // voxel_grid.setInputCloud(cloud_src_origin);
   // PointCloud::Ptr cloud_src (new PointCloud);
   // voxel_grid.filter(*cloud_src);
    //std::cout<<"down size *cloud_src_origin from "<<cloud_src_origin->size()<<"to"<<cloud_src->size()<<endl;
    //pcl::io::savePCDFileASCII("monkey_src_down.pcd",*cloud_src);

    //计算表面法线
    //pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne_src;
    //ne_src.setInputCloud(cloud_src);
    //pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_src(new pcl::search::KdTree< pcl::PointXYZ>());
   // ne_src.setSearchMethod(tree_src);
  //  pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud< pcl::Normal>);
   // ne_src.setRadiusSearch(0.02);
  //  ne_src.compute(*cloud_src_normals);

    //std::vector<int> indices_tgt;
   // pcl::removeNaNFromPointCloud(*cloud_tgt_origin,*cloud_tgt_origin, indices_tgt);
   // std::cout<<"remove *cloud_tgt_origin nan"<<endl;

   // pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_2;
    //voxel_grid_2.setLeafSize(0.01,0.01,0.01);
   // voxel_grid_2.setInputCloud(cloud_tgt_origin);
   // PointCloud::Ptr cloud_tgt (new PointCloud);
   // voxel_grid_2.filter(*cloud_tgt);
    //std::cout<<"down size *cloud_tgt_origin.pcd from "<<cloud_tgt_origin->size()<<"to"<<cloud_tgt->size()<<endl;
    //pcl::io::savePCDFileASCII("monkey_tgt_down.pcd",*cloud_tgt);

   // pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne_tgt;
   // ne_tgt.setInputCloud(cloud_tgt);
   //  pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_tgt(new pcl::search::KdTree< pcl::PointXYZ>());
   //  ne_tgt.setSearchMethod(tree_tgt);
   // pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud< pcl::Normal>);
    //ne_tgt.setKSearch(20);
   // ne_tgt.setRadiusSearch(0.02);
   // ne_tgt.compute(*cloud_tgt_normals);

    //计算FPFH
    //pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> fpfh_src;
    // fpfh_src.setInputCloud(cloud_src);
   // fpfh_src.setInputNormals(cloud_src_normals);
   // pcl::search::KdTree<PointT>::Ptr tree_src_fpfh (new pcl::search::KdTree<PointT>);
    //fpfh_src.setSearchMethod(tree_src_fpfh);
   // pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>());
   // fpfh_src.setRadiusSearch(0.05);
    //fpfh_src.compute(*fpfhs_src);
    //std::cout<<"compute *cloud_src fpfh"<<endl;

   //  pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> fpfh_tgt;
   // fpfh_tgt.setInputCloud(cloud_tgt);
    //fpfh_tgt.setInputNormals(cloud_tgt_normals);
   // pcl::search::KdTree<PointT>::Ptr tree_tgt_fpfh (new pcl::search::KdTree<PointT>);
   // fpfh_tgt.setSearchMethod(tree_tgt_fpfh);
   // pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::FPFHSignature33>());
   // fpfh_tgt.setRadiusSearch(0.05);
   // fpfh_tgt.compute(*fpfhs_tgt);
   // std::cout<<"compute *cloud_tgt fpfh"<<endl;

    //SAC配准
    //  pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> scia;
    //scia.setInputSource(cloud_src);
    //scia.setInputTarget(cloud_tgt);
    // scia.setSourceFeatures(fpfhs_src);
    //scia.setTargetFeatures(fpfhs_tgt);
    //scia.setMinSampleDistance(1);
    //scia.setNumberOfSamples(2);
    //scia.setCorrespondenceRandomness(20);
   // PointCloud::Ptr sac_result (new PointCloud);
   // scia.align(*sac_result);
   // std::cout  <<"sac has converged:"<<scia.hasConverged()<<"  score: "<<scia.getFitnessScore()<<endl;
    // Eigen::Matrix4f sac_trans;
    // sac_trans=scia.getFinalTransformation();
    //std::cout<<"sac_trans== "<<sac_trans<<endl;
    //pcl::io::savePCDFileASCII("monkey_transformed_sac.pcd",*sac_result);
    clock_t sac_time=clock();

//    //icp配准
    PointCloud::Ptr icp_result (new PointCloud);
//    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//    icp.setInputSource(cloud_src_origin);
//    icp.setInputTarget(cloud_tgt_origin);
//    icp.setMaxCorrespondenceDistance (2);
//    // 最大迭代次数
//    icp.setMaximumIterations (5);
//    // 两次变化矩阵之间的差值
//    icp.setTransformationEpsilon (1e-10);
//    // 均方误差
//    icp.setEuclideanFitnessEpsilon (0.2);
//    icp.align(*icp_result,sac_trans);
//
//    clock_t end=clock();
//
//    cout<<"total time: "<<(double)(end-start)/(double)CLOCKS_PER_SEC<<" s"<<endl;
//
//    cout<<"sac time: "<<(double)(sac_time-start)/(double)CLOCKS_PER_SEC<<" s"<<endl;
//
//    cout<<"icp time: "<<(double)(end-sac_time)/(double)CLOCKS_PER_SEC<<" s"<<endl;
//
//    std::cout << "ICP has converged:" << icp.hasConverged()
//              << " score: " << icp.getFitnessScore() << std::endl;
//
//    icp_trans=icp.getFinalTransformation();
//    //cout<<"ransformationProbability"<<icp.getTransformationProbability()<<endl;
//    std::cout<<"icp_trans== "<<icp_trans<<endl;
//    //使用创建的变换对未过滤的输入点云进行变换
//    pcl::transformPointCloud(*cloud_src_origin, *icp_result, icp_trans);
//    //保存转换的输入点云
//    pcl::io::savePCDFileASCII("monkey_transformed_sac_ndt.pcd", *icp_result);
//
//    //计算误差
//    Eigen::Vector3f ANGLE_origin;
//    ANGLE_origin<<0,0,M_PI/5;
//    double error_x,error_y,error_z;
//    Eigen::Vector3f ANGLE_result;
//    Matrix2Angle(icp_trans,ANGLE_result);
//    error_x=fabs(ANGLE_result(0))-fabs(ANGLE_origin(0));
//    error_y=fabs(ANGLE_result(1))-fabs(ANGLE_origin(1));
//    error_z=fabs(ANGLE_result(2))-fabs(ANGLE_origin(2));
//    cout<<"original angle in x y z:\n"<<ANGLE_origin<<endl;
//    cout<<"error in aixs_x: "<<error_x<<"  error in aixs_y: "<<error_y<<"  error in aixs_z: "<<error_z<<endl;
    //可视化

    std::cout

    visualize_pcd(cloud_src_origin,cloud_tgt_origin,icp_result);
}

//yong.qi reversed


int
main (int argc, char** argv)
{
    std::string src_cloud_path="../Depth_00052258.pcd";
    std::string tgt_cloud_path="../Depth_00053716.pcd";
    float rotation_degree=-0.035214+0.040624;
    Eigen::AngleAxisd rotation_vector(rotation_degree,Eigen::Vector3d(0,0,1));
    Eigen::Vector3d v((-131.884842+579.803345),(-320.641327+302.853088),0);
    Eigen::Isometry3d T=Eigen::Isometry3d::Identity();  //虽然称为3d，实质上是4x4的矩阵

    T.rotate(rotation_vector); //按照rotation_vector进行旋转
    T.pretranslate(v);

    Eigen::Matrix4f sac_trans=Eigen::Matrix4f::Identity();

    sac_trans<<T(0,0),T(0,1),T(0,2),T(0,3),
            T(1,0),T(1,1),T(1,2),T(1,3),
            T(2,0),T(2,1),T(2,2),T(2,3),
            T(3,0),T(3,1),T(3,2),T(3,3);

    cout<<"Isometry3d== "<< T.matrix() <<endl;
    cout<<"Matrix4f== "<<sac_trans<<endl;

    Eigen::Matrix4f icp_trans;

    doRegistration(src_cloud_path,tgt_cloud_path,sac_trans,icp_trans);
    return (0);
}




