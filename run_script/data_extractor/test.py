#! /usr/bin/env python
import sys
import os
import pathlib
import yaml

def main(argv):
    curr_dir = os.path.dirname(os.path.abspath(__file__))
    proj_dir = os.path.abspath(os.path.join(os.getcwd(), os.pardir))
    config_file = '%s/param/navdata_collector.yaml'%proj_dir
    print(config_file)
    
    with open(config_file, "r") as f:
        config = yaml.safe_load(f)
    
    print(config)
    
if __name__ == '__main__':
   main(sys.argv)
