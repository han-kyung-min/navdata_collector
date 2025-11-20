
# How extract the collected bagfiles 

## 2. Extract and copy the extracted data to TOPOMAP_DIR

>Similar to the previous step, modify navdata_collector.yaml to set correct $INPATH and $OUTPATH to extract the bagfile data.
>Specifically, modify navdata_collector['navdata_extractor']['inpath'] and navdata_collector['navdata_extractor']['outpath']
>$INPATH could be the folder generated from the previous data collection step whose name has YYYY-MM-DD-HH-MM. The '$INPATH normally contains child bagfile folders such as bag_YYYY-MM-DD-HH-MM-SS. $OUTPATH must be a physically different folder from $OUTPATH.

>execute the following command to generate the topomap where $EXTRACTED_DATA_DIR is must be identical to $OUTPATH specified above. $TOPOMAP_DIR the final destination of topology map.

```
cd $PROJECT_DIR/deployment/src
./generate_topomap.sh <EXTRACTED_DATA_DIR> <TOPOMAP_DIR>
```

>This process extracts and syncs the metadata stored in the bag file recorded in the previous step.
>Then, copies them to $TOPOMAP_DIR, followed by generating slam_poses.txt and the final topopmap
