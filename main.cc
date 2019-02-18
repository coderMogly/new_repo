//Author Anirudh Yadav


#include <iostream>
#include <fstream>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
//#include <MapPoint.h>
//#include <MapFrame.h>
#include <VoxelData.h>
#include <Eigen/Eigen>

#include <unsupported/Eigen/NonLinearOptimization>


#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
// #include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/types/icp/types_icp.h>
#include <g2o/core/robust_kernel_impl.h>



void load_global_map(pcl::PointCloud<pcl::PointXYZ>::Ptr gl_ptr){
	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("global_map.pcd", *cloud) == -1)
  	{
    	PCL_ERROR ("Couldn't read file global_map.pcd \n");
	}
}

cv::Mat read_transformation(std::ifstream file, int j){

}



int main(){
	//important variables
	int N_min = 100;
	int multiple_cov = 4;


	//opening transformation file
	char str1[];
	std::cout<<"Enter the file name for transformations"<<endl;
	std::cin>>str1;
	std::ifstream keyframe_trans(str1); 


	
	//creation of global map and setting up kdtree
	pcl::PointCloud<pcl::PointXYZ>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZ>);
	load_global_map(global_map);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(global_map);
	int K = 2;
	std::vector<int> pointIdxNKNSearch(K);
  	std::vector<float> pointNKNSquaredDistance(K);
  	//example
  	

  	//kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
  	pcl::PointCloud<pcl::PointXYZ>::Ptr keyframe_pcd(new pcl::PointCloud<pcl::PointXYZ>());
  	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_keyframe_pcd(new pcl::PointCloud<pcl::PointXYZ>());
  	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_global_map(new pcl::PointCloud<pcl::PointXYZ>());
  	

  	

  	//voxelization of global map
  	pcl::VoxelGridCovariance<pcl::PointXYZ> voxel_grid_filter;
  	voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  	voxel_grid_filter.setInputCloud(global_map);
  	voxel_grid_filter.filter(*filtered_global_map);
  	const std::map< size_t, pcl::VoxelGridCovariance::Leaf> vox_leaf = voxel_grid_filter.getLeaves(); 

  	//do stuff to create local point distribution
  	std::map< size_t, pcl::VoxelGridCovariance::Leaf>::iterator itr;
  	std::std::vector<MONO_SLAM::VoxelData> vox_data;
  	vox_data.resize(vox_leaf.size());	
  	for(itr=vox_leaf.begin(); itr!=vox_leaf.end(); ++itr){
  		MONO_SLAM::VoxelData temp;
  		Eigen::Vector3d mean = itr->second.getMean();
  		//this needs to be modified 
  		Eigen::Matrix3d evecs = itr->second.getEvacs();
  		Eigen::Matrix3d temp_cov = itr->second.getCov();
  		Eigen::Matrix4d cov;
  		Eigen::Matrix4d A;
  		temp.TR(0,0) = evecs(0,0);
  		temp.TR(0,1) = evecs(1,0);
  		temp.TR(0,2) = evecs(2,0);
  		temp.TR(1,0) = evecs(0,1);
  		temp.TR(1,1) = evecs(1,1);
  		temp.TR(1,2) = evecs(2,1);
  		temp.TR(2,0) = evecs(0,2);
  		temp.TR(2,1) = evecs(1,2);
  		temp.TR(2,2) = evecs(2,2);
  		temp.TR(0,3) = 0;
  		temp.TR(1,3) = 0;
  		temp.TR(2,3) = 0;
  		temp.TR(3,0) = mean(0);
  		temp.TR(3,1) = mean(1);
  		temp.TR(3,2) = mean(2);
  		temp.TR(3,3) = 1;
  		cov(0,0)=temp_cov(0,0);
  		cov(0,1)=temp_cov(0,1);
  		cov(0,2)=temp_cov(0,2);
  		cov(1,0)=temp_cov(1,0);
  		cov(1,1)=temp_cov(1,1);
  		cov(1,2)=temp_cov(1,2);
  		cov(2,0)=temp_cov(2,0);
  		cov(2,1)=temp_cov(2,1);
  		cov(2,2)=temp_cov(2,2);
  		cov(3,0)=0;
  		cov(3,1)=0;
  		cov(3,2)=0;
  		cov(0,3)=0;
  		cov(1,3)=0;
  		cov(2,3)=0;
  		cov(3,3)=1;
  		A = (temp.TR)*cov*(Eigen::Transpose(temp.TR));
  		temp.AR(0) = sqrt(A(0,0));
  		temp.AR(1) = sqrt(A(1,1));
  		temp.AR(2) = sqrt(A(2,2));
  		temp.N = itr->second.getPointCount();
  	}





  	//cv::Mat keyframe_trans;
	char str[];
  	bool stop = true;
  	int i =1;
  	float threshold;
  	int K_itr =10;
  	std::vector <std::pair<int, int>> corres_set;
	while(stop == true){

		//loading of a local map
		std::cout<<"Enter the file name of first key frame: "<<endl;
		std::cin>>str;
		if(str == "stop"){
			stop = true;
			continue;
		}
		if (pcl::io::loadPCDFile<pcl::PointXYZ> (str, *keyframe_pcd) == -1)
  		{
    		PCL_ERROR ("Couldn't read file str.pcd \n");
		}
		keyframe_trans = read_transformation(keyframe_trans, i);
		corres_set.clear();
		threshold = 0.01;
		for(int j = 0; j<K_itr; j++){
			pcl::transformPointCloud (*keyframe_pcd, *transformed_keyframe_pcd, keyframe_trans);
			for( int idx = 0; idx < transformed_keyframe_pcd->size(); idx++ )
			{
   				pcl::PointXYZ v = transformed_keyframe_pcd->points[idx];
   				kdtree.nearestKSearch(v, K, pointIdxNKNSearch, pointNKNSquaredDistance);
   				if(pointNKNSquaredDistance[0]<threshold){
   					corres_set.push_back(std::make_pair(idx,pointIdxNKNSearch[0]));
   				}
			}
			if(corres_set.empty()){
				std::cout<<"corres_set is empty"<<std::endl;
				break;
			}
		}
		std::vector<std::pair<int, int>>::iterator it;
		for(it = corres_set.begin(); it != corres_set.end(); ++it){
			int flag = 0;
			pcl::PointXYZ v = transformed_keyframe_pcd->points[*it.first];  // multiply with transformation
			std::vector<pcl::VoxelGridCovariance::LeafConstPtr> near_Leaf;
			std::vector<float> near_leaf_dist;
			int tttt = voxel_grid_filter.nearestKSearch(v, 2, near_Leaf, near_leaf_dist);
			std::vector<pcl::VoxelGridCovariance::LeafConstPtr>::iterator it1;
			for(it1 = near_Leaf.begin(); it1!=near_Leaf.end(); ++it1){
				//pcl::VoxelGridCovariance::LeafConstPtr temp_ptr = near_Leaf[0];
				if(near_Leaf[0].getPointCount() > N_min){
					flag = 1;
				}
			}
			if(flag == 0){
				corres_set.erase(it1);
			}
		}


		//Alignment-------------------------------------
		g2o::SparseOptimizer optimizer;
  		optimizer.setVerbose(false);

  		// variable-size block solver
  		g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
    	g2o::make_unique<BlockSolverX>(g2o::make_unique<LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>()));

  		optimizer.setAlgorithm(solver);

  		VertexSE3 *vp0 = new VertexSE3(); 
    	vp0->setId(0);
    	VertexSE3 *vp1 = new VertexSE3(); 
   	 	vp1->setId(1);

    	vp0->setFixed(true);

    	// add to optimizer
    	optimizer.addVertex(vp0);
    	optimizer.addVertex(vp1);


    	//new datatype needed in g2o






		i=i+1;
	}
}
