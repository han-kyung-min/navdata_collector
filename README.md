
# How extract the collected bagfiles 


## 1. To extract collision data bagfile

>Modify navdata_collector.yaml to set correct $INPATH and $OUTPATH to extract the bagfile data.
>Specifically, modify navdata_collector['navdata_extractor']['inpath'] and navdata_collector['navdata_extractor']['outpath']
>$INPATH could be the folder generated from the previous data collection step whose name has YYYY-MM-DD-HH-MM with the file indicates if the bag file is for navdata or collision data. For example, coll_data file indicates this bag file contains collision data. The '$INPATH normally contains child bagfile folders such as bag_YYYY-MM-DD-HH-MM-SS. $OUTPATH must be a physically different folder from $OUTPATH.

>execute the following command to generate the topomap where $EXTRACTED_DATA_DIR is must be identical to $OUTPATH specified above. 

```
cd $BASE_DIR/navdata_collector/run_script/data_extractor
./generate_colldata_from_bag.sh 
```
>$BASE_DIR is navdata_collector pkg in your catkin_ws folder. e.g) catkin_ws/src/navdata_collector
>This process extracts and syncs the metadata stored in the bag file recorded in the previous step.

## 2. To collect collision data using the matlab GUI run_script




