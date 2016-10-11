# gatherpclmsg
The ROS package, used to gather PCL point cloud topic and save to designated path. This package listen to topic when it started and terminates when user input CTRL-C.
If the cloud data would surpass your memory capacity. Partition feature is ready for this. For more detail, please read in Usage section.
## Usage
You can read help info by
```bash
rosrun gatherpclmsg gatherpclmsg  -h
Allowed options:
  -h [ --help ]               print help message
  -o [ --output ] arg         explictly output file name
  -b [ --binary ] arg (=1)    write binary PCD
  -p [ --prefix ] arg         output prefix
  -t [ --partition ] arg (=0) frame num of a partition ( 0 for no parititon)
  --topic                     topic which would be subscribed
```
In generally, the minium format of the command line like this:
```bash
rosrun gatherpclmsg gatherpclmsg /velodyne_points
```
Listen to /velodyne_points and save to $PWD directory with the timestamp relative filename.

Advanced, you can use '-p' or '-o' to assign other folder to save.
'-t' argument decide frame count interval to divide into seperate partition file. This would be processed in other thread, don't worry about blocking main thread.
