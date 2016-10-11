#include <signal.h>
#include <string.h>
#include <sys/stat.h>

#include <boost/date_time/local_time/local_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include <ros/console.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/PointCloud2.h>

namespace po = boost::program_options;

bool storageLock;
std::list<pcl::PointCloud<pcl::PointXYZ> > storage;

std::string pcdprefix;
std::string pcdpath;
std::string topic;
bool pcdbinary;
int recvNum=0;
void registedLaserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
    if (storageLock)
        return;
    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
    storage.push_back(laserCloudIn);
    ROS_INFO_STREAM("Recv "<<recvNum++);
}

void exportMergedMap()
{
    ROS_INFO("Start merging...");
    pcl::PointCloud<pcl::PointXYZ> cloudOut;
    std::list<pcl::PointCloud<pcl::PointXYZ> >::iterator it = storage.begin(), end = storage.end();
    for (; it != end; ++it) {
        cloudOut += *it;
        it->clear();
    }
    ROS_INFO_STREAM("Merged: " << cloudOut.size());
    ROS_INFO("Start saving, please wait...");
    pcl::io::savePCDFile(pcdpath, cloudOut, pcdbinary);
    ROS_INFO("File save success, Good bye.");
}

void exitNode(int sig)
{
    storageLock = true;
    ROS_INFO_STREAM("SIG " << sig << " RECEIVED.");
    exportMergedMap();
    ros::shutdown();
}
template <class T>
std::string timeToStr(T ros_t)
{
    std::stringstream msg;
    const boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
    boost::posix_time::time_facet* const f = new boost::posix_time::time_facet("%Y%m%d%H%M%S");
    msg.imbue(std::locale(msg.getloc(), f));
    msg << now;
    return msg.str();
}

void updateFilenames()
{
    std::vector<std::string> parts;

    std::string prefix = pcdprefix;
    uint32_t ind = prefix.rfind(".pcd");

    if (ind == prefix.size() - 4) {
        prefix.erase(ind);
        ind = prefix.rfind(".pcd");
    }

    if (prefix.length() > 0)
        parts.push_back(prefix);
    parts.push_back(timeToStr(ros::WallTime::now()));

    pcdpath = parts[0];
    for (unsigned int i = 1; i < parts.size(); i++)
        pcdpath += std::string("_") + parts[i];
    pcdpath += std::string(".pcd");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "loam_gatherMap");
    ros::NodeHandle nh;
    po::options_description desc("Allowed options");
    desc.add_options()("help,h", "print help message")("output,o", po::value<std::string>(), "explictly output file name")("binary,b", po::value<bool>()->default_value(true), "write binary PCD")("prefix,p", po::value<std::string>(), "output prefix")("topic", "topic which would be subscribed");
    po::positional_options_description p;
    p.add("topic", -1);
    po::variables_map vm;

    try {
        po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
        po::notify(vm);
    } catch (std::exception& e) {
        throw ros::Exception(e.what());
    }
    if (vm.count("help")) {
        std::cout << desc << std::endl;
        exit(0);
    }
    if (vm.count("binary")) {
        pcdbinary = vm["binary"].as<bool>();
    }
    if (vm.count("prefix")) {
        pcdprefix = vm["prefix"].as<std::string>();
    }
    if (vm.count("output")) {
        pcdpath = vm["output"].as<std::string>();
    } else {
        updateFilenames();
    }
    if (vm.count("topic")) {
        topic = vm["topic"].as<std::string>();
    } else {
        std::cout << desc << std::endl;
        exit(0);
    }
    signal(SIGABRT, &exitNode);
    signal(SIGTERM, &exitNode);
    signal(SIGINT, &exitNode);
    storageLock = false;
    ros::Subscriber subPointCloudMsg = nh.subscribe<sensor_msgs::PointCloud2>(topic, 2, registedLaserCloudHandler);

    ROS_INFO_STREAM("=Prefix: " << pcdprefix);
    ROS_INFO_STREAM("=Output: " << pcdpath);
    ROS_INFO_STREAM("=Use Binary: " << pcdbinary);
    ROS_INFO_STREAM("=Topic: " << topic);
    ros::spin();

    return 0;
}
