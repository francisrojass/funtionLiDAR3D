#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <geometry_msgs/TransformStamped.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>

#include <iomanip>

class PointCloudAccumulator
{
public:
    PointCloudAccumulator() : max_point_age_(1000000.0), tfListener(tfBuffer) // Máxima antigüedad en segundos
    {
        // Suscribirse al tópico de la nube de puntos
        sub_ = nh_.subscribe("/limovelo/pcl", 1000, &PointCloudAccumulator::pointCloudCallback, this);

        // Publicar la nube de puntos acumulada
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/slam_perception", 1);

        // Inicializar la nube de puntos acumulada
        accumulated_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &input)
    {
        sensor_msgs::PointCloud2 transformed_cloud_msg;

        geometry_msgs::TransformStamped transformStamped;
        try
        {
            transformStamped = tfBuffer.lookupTransform("map", "body", input->header.stamp);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            return;
        }
        // Aplicar la transformación a la nube de puntos
        try
        {
            tf2::doTransform(*input, transformed_cloud_msg, transformStamped);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("Failed to transform point cloud: %s", ex.what());
            return;
        }

        // Convertir el mensaje de nube de puntos a una nube de puntos PCL
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*input, *transformed_cloud);

        double current_time = ros::Time::now().toSec();
        // Agregar la nube de puntos actual a la cola de nubes
        for (auto &point : transformed_cloud->points)
        {
            point_cloud_queue_.push_back(std::make_pair(point, current_time));
        }

        // Eliminar puntos antiguos
        while (!point_cloud_queue_.empty() && (current_time - point_cloud_queue_.front().second > max_point_age_))
        {
            point_cloud_queue_.pop_front();
        }

        // Reconstruir la nube de puntos acumulada
        accumulated_cloud_->clear();
        for (auto &pair : point_cloud_queue_)
        {
            accumulated_cloud_->points.push_back(pair.first);
        }

        /*
        Eigen::Affine3f transformI = Eigen::Affine3f::Identity();
        transformI(1, 1) = -transformI(1, 1); // Invertir el eje Y
        transformI(2, 2) = -transformI(2, 2); // Invertir el eje Z

        // Aplicar la transformación a la nube de puntos
        pcl::transformPointCloud(*accumulated_cloud_, *accumulated_cloud_, transformI);
        */


        // Publicar la nube de puntos acumulada
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*accumulated_cloud_, output);
        output.header.frame_id = input->header.frame_id;
        output.header.stamp = ros::Time::now();
        pub_.publish(output);
    }

private:

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_cloud_;
    std::deque<std::pair<pcl::PointXYZI, double>> point_cloud_queue_;
    double max_point_age_; // Máxima antigüedad de los puntos en segundos
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_accumulator_node");
    PointCloudAccumulator pca;
    ros::spin();
    return 0;
}
