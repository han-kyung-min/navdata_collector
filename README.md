
# Navdata_collector


## Overview

This package supports dataset collection, extraction, and topological map generation for real-world robot deployment of the [DevGRU](#) project. 

Unless a user intends to create their own dataset, the use of this package is generally limited.

## 1 How to Setup

```bash
cd ~/catkin_ws/src
git clone https://github.com/han-kyung-min/navdata_collector.git

conda env create -f environment.yml
conda activate navdata

cd ~/catkin_ws
catkin_make install
```

---

## 2 How to Extract Topological Map from Bagfile

Modify `navdata_collector.yaml` (located in ~/catkin_ws/src/navdata_collector/param)  to set the correct paths for extracting bag file data.  
Specifically, update the following fields:

navdata_collector['navdata_extractor']['inpath']  
navdata_collector['navdata_extractor']['outpath']  

- `$INPATH`: Directory generated from the previous data collection step (typically named in the format `YYYY-MM-DD-HH-MM`).    
  This directory usually contains subfolders such as `bag_YYYY-MM-DD-HH-MM-SS`.  
- `$OUTPATH`: Directory where the extracted data will be saved.  
  This must be a **different directory** from `$INPATH`.

After configuring the paths, execute the following command to generate the topological map from the recorded bagfile:
<!-- Make sure that ROS_MASTER_URI is set to http://127.0.0.1:11311 before running the following steps-->

```
cd ~/catkin_ws/src/navdata_collector/run_script/data_extractor
python generate_topomap_from_bag.py
```

The procedure extracts and synchronizes the metadata from the bag files recorded in the previous step.  
The extracted data is then copied to `$TOPOMAP_DIR`, where `slam_poses.txt` and the final topological map are generated.
    

## 3 How to Collect Collision-free Dataset

### (1) Record Bag Files During Teleoperation

Modify `navdata_collector.yaml` (located in ~/catkin_ws/src/navdata_collector/param) to set the correct directory for saving recorded bag files.  
Specifically, update the following fields:

navdata_collector['navdata_collector']['bagfile_root_path']  

Run the following commands and tele-operate the robot to begin data collection:

```
cd ~/catkin_ws/src/navdata_collector/run_script/data_collector
python run_manual_data_collection.py
```

This procedure records multiple bag files, each with a maximum duration defined by:  
>navdata_collector['navdata_collector']['max_nav_time']   

Recording continues until one of the following conditions is met:  
>- navdata_collector['navdata_collector']['max_nav_time'] is reached for a bag file  
>- navdata_collector['navdata_collector']['max_num_bagfiles'] is saved.

### (2) Extract Bag Files and Generate Metadata

Similar to the step described for extracting topological map, modify `navdata_collector.yaml` (located in ~/catkin_ws/src/navdata_collector/param) to set the correct paths for extracting bag file data.
Specifically, update the following fields:

navdata_collector['navdata_extractor']['inpath']  
navdata_collector['navdata_extractor']['outpath']  

- `$INPATH`: Directory generated from the previous data collection step (typically named in the format `YYYY-MM-DD-HH-MM`).    
  This directory usually contains subfolders such as `bag_YYYY-MM-DD-HH-MM-SS`.  
- `$OUTPATH`: Directory where the extracted data will be saved.  
  This must be a **different directory** from `$INPATH`.

After configuring the paths, execute the following command to generate the topological map from the recorded bagfile:

```
cd ~/catkin_ws/src/navdata_collector/run_script/data_extractor
python script_extract_bags.py
```

### (3) Generate Collision-Free Dataset from Extracted Metadata

This step requires running the following MATLAB script:  
```
~/catkin_ws/src/navdata_collector/matlab_scripts/script_gen_noncoll_dataset.m
```

Before execution, configure the following parameters in the script:  
>- data_root_path: Path to the directory containing the extracted metadata
>- out_folder: Directory where the generated dataset will be saved

The script generates multiple folders. Each folder contains:
> - Depth image context
> - Odometry context
> - Ground-truth subgoal (SG) pose
> - Ground-truth waypoint sequences
> - Additional metadata (if applicable)

# 4 How to Collect Collision Dataset
Collecting a collision dataset requires autonomous navigation using the DevGRU-base model, which is trained on a collision-free dataset. Therefore, [DevGRU](#) package must be properly installed prior to executing the steps below.

## (1) Recording Collision Event Data

> (i) Locate the robot at the starting node of a prebuilt topological map  
> (ii) Then, execute the following cmd:
> ```
> cd $DevGRU_PROJECT_DIR/deployment/src
> ./navigate_w_colldata_bagging.sh 
> ```   
> (iii) During autonomous navigation, use the deadman switch button (L1) to stop the robot when a collision is imminent. Then, correct its pose and allow it to resume navigation. By repeating this procedure, multiple collision events can be recorded in a bag file.  
> (iv) Press L1 + R1 + L2 to stop the process once a sufficient number of events has been collected.

## (2) Extract Metadata Including Collision Events from Bag File

>Modify navdata_collector.yaml (located in ~/catkin_ws/src/navdata_collector/param) to set correct $INPATH and $OUTPATH to extract the bagfile data.
>Specifically, modify navdata_collector['navdata_extractor']['inpath'] and navdata_collector['navdata_extractor']['outpath']
>$INPATH could be the folder generated from the previous data collection step whose name has YYYY-MM-DD-HH-MM with the file indicates if the bag file is for navdata or collision data. For example, coll_data file indicates this bag file contains collision data. The '$INPATH normally contains child bagfile folders such as bag_YYYY-MM-DD-HH-MM-SS. $OUTPATH must be a physically different folder from $OUTPATH.

>execute the following command to generate the topomap where $EXTRACTED_DATA_DIR is must be identical to $OUTPATH specified above. 

```
 cd $BASE_DIR/navdata_collector/run_script/data_extractor
 ./generate_colldata_from_bag.sh 
```
>$BASE_DIR is navdata_collector pkg in your catkin_ws folder. e.g) catkin_ws/src/navdata_collector
>This process extracts and syncs the metadata stored in the bag file recorded in the previous step.

## (3) Generate Collision Data from the extracted metadata

This process requires the MATLAB GUI script located at:  
$BASE_DIR/navdata_collector/run_script/data_extractor/matlab_script/script_colldata_collection.m  

Before proceeding, ensure the following parameters are correctly configured:  

>- proc_dir: Directory containing the processed metadata
>- out_base_dir: Output directory for the generated dataset
>- topomap_root_dir: Path to the pre-built topological maps

Procedure:
>(i)  Run `script_colldata_collection.m` to generate the collision dataset  
>(ii) Run `script_gen_colldata_from_collected.m` to sample and copy a subset of the data into the training data directory

