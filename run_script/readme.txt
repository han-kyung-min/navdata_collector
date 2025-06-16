
0.  run "conda activate navdata"
    then, run "source ~/catkin_ws/install/setup.bash"  <--- need this to make packages understand the custom RGB-D message  

1. To collect data

Do not run "run_ae_data_collection.py". It is not recommended to use this auto-data collection function b/c SLAM goes unstable after a certain amount of time (5 ~ 10mins).

I would run "run_manual_data_collection.py" to use joy-stick controller to move the robot around while collecting the dataset.
Make sure to edit "navdata_collector.yaml" file located under "param/" to modify the parameters in the config file.
Double check with the "bagfile_root_path". 5mins long bag files gonna be saved under the specified bagfile_root_path

There should be a custom rgb-d message (exact time) synced saved into the bag files. Make sure to follow the instruction below to extract out the time-synced RGB and depth images.

2. To decode bag files and associate metadata. 

run "script_extract_bags.py ../../param/navdata_collector.yaml"

Eliminate small sized bag files (less than 200M) as there is no rgb-d data saved.. I wouldn't care about the file size less than 1 GB.. Just remove those small files.

The script should be capable of fixing *.active.bag files. Nevertheless, You can manually fix the file inactive file by 
$ rosbag reindex *.bag.active
$ rosbag fix *.bag.active repaired.bag

In addition, you need to make sure to check if you have collected essential dataset. Is there any data missing ?
Besides, this process is supposed to extract synced RGB and depth file. 


3. to visualize them

run "script_display_associated_metadata.py" 

Here, you need to make sure to keep low time differences btwn the associated metadata.


