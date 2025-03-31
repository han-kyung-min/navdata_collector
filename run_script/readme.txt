
0. conda activate navdata

1. To collect data

run "run_ae_data_collection.py"

rgb-d should be synced...

2. To decode bag files

run "script_decode_bags.py"

Eliminate small sized bag files (less than 200M) as there is no rgb-d data saved.. I wouldn't care about the file size less than 1 GB..  
In addition, you need to make sure to check if you have collected essential dataset. Is there any data missing ?

3. To associate dataset 

run "script_sync_metadata"

4 to visualize them

run "script_display_associated_metadata.py" 

Here, you need to make sure to keep low time differences btwn the associated metadata.


