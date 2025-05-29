
0. conda activate navdata

1. To collect data

run "run_ae_data_collection.py"

there should be a custom rgb-d message (exact time) synced saved into the bag files

2. To decode bag files and associate metadata 

run "script_extract_bags.py ../../param/navdata_collector.yaml"

Eliminate small sized bag files (less than 200M) as there is no rgb-d data saved.. I wouldn't care about the file size less than 1 GB..  
In addition, you need to make sure to check if you have collected essential dataset. Is there any data missing ?

Besides, this process is supposed to extract synced RGB and depth file. 

3. to visualize them

run "script_display_associated_metadata.py" 

Here, you need to make sure to keep low time differences btwn the associated metadata.


