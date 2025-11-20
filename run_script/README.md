
## To collect the dataset

#### Init scripts
```
    conda activate navdata
    source ~/catkin_ws/install/setup.bash
```
> * The 2nd command is necessary to make system understand the custom RGB-D message  

#### Manual data collection

```
    cd /home/$USER/catkin_ws/src/navdata_collector
    python manual_data_collection.py
```

> * Use joy-stick controller to move the robot around while collecting the dataset. <br />
> * Make sure to edit "navdata_collector.yaml" file located under "param/" to modify the parameters in the config file. Double check the "bagfile_root_path" in the config file. 

The command above starts generating 5 mins long bag files. The files are gonna be saved under the specified bagfile_root_path

#### Autonomous data collection (is not ready yet)

Do not run "run_ae_data_collection.py". It is not recommended to use this auto-data collection function b/c SLAM goes unstable after a certain amount of time (5 ~ 10mins).

## To extract bag files (and to sync them)

```
    python script_extract_bags.py ../../param/navdata_collector.yaml"
```


> * I would eliminate small sized bag files (less than 200M) as there is no rgb-d data saved.. I wouldn't care about the file size less than 1 GB.. Just remove those small files.

> * The script should be capable of fixing *.active.bag files. Nevertheless, You can manually fix the file inactive file by 

```
    $ rosbag reindex *.bag.active
    $ rosbag fix *.bag.active repaired.bag
```

In addition, you need to make sure to check if you have collected essential dataset. Is there any data missing ?
Besides, this process is supposed to extract synced RGB and depth file. 


## To visualize them

> * run "script_gen_sync_movie.m" in MATLAB 


# To collect collision dataset

> ### 1. Follow the nav dataset collector above to generate a bag file to create a TOPOMAP

> ### 2. Extract the bag file by following to extract the bag file. 
> ### 3. Move the contents under bagfile to TOPOMAP dir 
> * e.g., "/home/hankm/python_ws/viznav/depth-nav/deployment/topomaps/roundxx"

> ### 4. Have Former robot ready to collect the collision events
> * Make sure to modify "depth_nav.yaml" config file under depth-nav/config dir
> * *topomap_dir* must be modified appropriately to make the robot follow SGs
> * Then, place the robot where the topomap begins

> ### 5. Execute the running command
```
    $ cd /home/$USER/python_ws/viznav/depth-nav/deployment/src
    $ sh navigate_w_bagging_data.sh
```
> * You need to press L1 + joy control when the collision is about to happen. 
> * Pressing L1, L2, R1, and R2 together will end the process
 
> ### 6. Extract the stored colldata bag file
> * Open navdata_collector.yaml which is located under ~/catkin_ws/src/navdata_collector/param/ 
> * Set navdata_collector['navdata_extractor']['inpath'] and navdata_collector['navdata_extractor']['outpath']. The former should points to the input bagfile folder and the latter should points to the output path to extract the data.
> * The, execute the following commands
 ```
    $ cd /home/$USER/catkin_ws/src/navdata_collector/run_script/data_extractor
    $ python script_extract_bags.py ../../param/navdata_collector.yaml
```
> ### 7. Semi-automatic data collection
> * (1) Open MATLAB
> * (2) cd to /home/hankm/catkin_ws/src/navdata_collector/run_script/data_extractor/potmap2d
> * (3) Run  script_colldata_collection.m


