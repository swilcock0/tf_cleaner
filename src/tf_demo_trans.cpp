#include <tf_cleaner.h>
#include <ros/ros.h>
#include <random>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

int main(int argc, char **argv)
{
    std::random_device rd;
    std::default_random_engine generator(rd()); // rd() provides a random seed
    std::uniform_real_distribution<double> distribution(-2.0,2.0);

    std::uniform_real_distribution<double> distribution2(-5, 5);


    ros::init(argc, argv, "tf_test");

    ros::NodeHandle nh("/tf_test");
    
    ros::Publisher pub = nh.advertise<PointCloud> ("points2", 10);
    ros::Publisher markpub = nh.advertise<visualization_msgs::MarkerArray> ("/visualization_marker_array", 10);
    tfCleaner::tfCleaner cleaner(50);
    

    std::vector<tf::Transform> tf_store;

    visualization_msgs::Marker markMsg;
    markMsg.header.frame_id = "map";
    markMsg.type = markMsg.TEXT_VIEW_FACING;
    markMsg.pose.orientation.w = 1.0;
    markMsg.scale.x = markMsg.scale.y = markMsg.scale.z = 1.0;
    markMsg.color.r = markMsg.color.g = markMsg.color.b = markMsg.color.a = 1.0;
    visualization_msgs::MarkerArray marks;

    // Build a collection of 10 points with 1 outlier
    for (int i = 0; i < 50; i++){
        tf::Transform trans;
        trans.setIdentity();
        float x, y, z;
        x = distribution(generator);
        y = distribution(generator);
        z = distribution(generator);
        markMsg.text = "i";
        

        if (i %5 == 0){
            x = distribution2(generator);
            y = distribution2(generator);
            z = distribution2(generator);
            markMsg.text = "o";
        }
        
        markMsg.pose.position.x = x;
        markMsg.pose.position.y = y;
        markMsg.pose.position.z = z;
        markMsg.id = i;        

        trans.setOrigin(tf::Vector3(x,y,z));

        tf_store.push_back(trans);
        cleaner.addTF(trans);
        marks.markers.push_back(markMsg);
    }

    std::vector<int> inliers = cleaner.getStatisticalInlierIndicesTrans(1.0);


    // Publish cloud. Highlight outliers
    PointCloud::Ptr msg (new PointCloud);
    msg->header.frame_id = "map";
    
    //msg->height = msg->width = 10;


    for (int i = 0; i< 50; i++)
    {
        pcl::PointXYZRGB pt;
        
        if(std::find(inliers.begin(), inliers.end(), i) != inliers.end()) {
            // inlier, add green
            pt = pcl::PointXYZRGB(0, 255, 0);
        } else {
            // outlier, add red
            pt = pcl::PointXYZRGB(255, 0, 0);
            
        }
        
        tf::Vector3 vec = tf_store[i].getOrigin();
        pt.x = vec.getX();
        pt.y = vec.getY();
        pt.z = vec.getZ();

        msg->points.push_back (pt);
    }
    
    ROS_INFO("%d", (int)msg->points.size());

    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);

    pub.publish (*msg);

    ros::Rate loop_rate(0.5);
    while (nh.ok())
    {
        pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
        pub.publish (*msg);
        markpub.publish(marks);
        ros::spinOnce ();
        loop_rate.sleep ();
    }


    return 0;
}
