#include <iostream>
#include <signal.h>
#include <sstream>
#include <string>
#include <sys/stat.h>

#include <boost/date_time/local_time/local_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include <ros/console.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/PointCloud2.h>

boost::thread_group grp;

namespace po = boost::program_options;

bool storageLock;
std::list<pcl::PointCloud<pcl::PointXYZ> > storage;

std::string pcdprefix;
std::string pcdpath;
std::string topic;
bool pcdbinary;

int recvNum = 0; // recv frame num
int saveStartPoint = 0; // start pointer to be saving
int partitionSize = 0; // control the frame number of partition

int partNum = 0;

void mergeAndExport(std::string path, bool toEnd);

//! Work thread
struct OutputOp {
    OutputOp(std::string path)
    {
        this->path = path;
    }
    void operator()()
    {
        mergeAndExport(path, false);
    }

    std::string path;
};

//! pcl topic callback
void pclMsgCallback(const sensor_msgs::PointCloud2ConstPtr& pclMsg)
{
    if (storageLock)
        return;
    pcl::PointCloud<pcl::PointXYZ> pclCloud;
    pcl::fromROSMsg(*pclMsg, pclCloud);
    storage.push_back(pclCloud);
    ROS_INFO_STREAM("Recv " << recvNum++);
    if (partitionSize != 0) {
        if (recvNum % partitionSize == 0) {
            partNum++;
            ROS_INFO_STREAM("PARTNUM " << partNum);
            std::string path = pcdpath;
            std::ostringstream oss;
            oss << "p" << partNum;
            path.insert(path.size() - 4, oss.str());
            boost::thread* t = grp.create_thread(OutputOp(path));
        }
    }
}

//! merge and export frames to pcd file
void mergeAndExport(std::string path, bool toEnd)
{
    ROS_INFO("Start merging...");
    pcl::PointCloud<pcl::PointXYZ> cloudOut;

    std::list<pcl::PointCloud<pcl::PointXYZ> >::iterator it = storage.begin();
    std::advance(it, saveStartPoint);
    std::list<pcl::PointCloud<pcl::PointXYZ> >::iterator end = it;
    if (toEnd) {
        end = storage.end();
    } else {
        std::advance(end, partitionSize);
        saveStartPoint += partitionSize;
    }

    for (; it != end; ++it) {
        cloudOut += *it;
        it->clear();
    }
    ROS_INFO_STREAM("Merged " << cloudOut.size());
    if (cloudOut.empty()) {
        ROS_INFO("Empty, exit.");
        return;
    }
    ROS_INFO_STREAM("Start saving, please wait...\n"
        << path);
    pcl::io::savePCDFile(path, cloudOut, pcdbinary);
    ROS_INFO("File save success, Good bye.");
}

//! capturing terminal sig
void exitNode(int sig)
{
    storageLock = true;
    grp.join_all();
    std::string path = pcdpath;
    if (partNum != 0) {
        partNum++;
        std::ostringstream oss;
        oss << "p" << partNum;
        path.insert(path.size() - 4, oss.str());
    }
    mergeAndExport(path, true);
    ros::shutdown();
}

//! convert current time to string
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

//! generate file name
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
    ros::init(argc, argv, "gatherpclmsg");
    ros::NodeHandle nh;
    po::options_description desc("Allowed options");
    desc.add_options()("help,h", "print help message")("output,o", po::value<std::string>(), "explictly output file name")("binary,b", po::value<bool>()->default_value(true), "write binary PCD")("prefix,p", po::value<std::string>(), "output prefix")("partition,t", po::value<int>()->default_value(0), "frame num of a partition ( 0 for no parititon)")("topic", "topic which would be subscribed");
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
    if (vm.count("partition")) {
        partitionSize = vm["partition"].as<int>();
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
    ros::Subscriber subPointCloudMsg = nh.subscribe<sensor_msgs::PointCloud2>(topic, 2, pclMsgCallback);

    ROS_INFO_STREAM("|Prefix:\t| " << pcdprefix);
    ROS_INFO_STREAM("|Output:\t| " << pcdpath);
    ROS_INFO_STREAM("|BinaryPCD:\t| " << pcdbinary);
    ROS_INFO_STREAM("|Paritition:\t| " << partitionSize);
    ROS_INFO_STREAM("|Topic:\t\t| " << topic);
    ros::spin();

    return 0;
}
