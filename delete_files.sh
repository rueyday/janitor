#!/bin/bash

# Get the parent directory of the script
DIR="$(dirname "$(realpath "$0")")/.."

# Find and delete all non-hidden files and directories in the parent directory
find "$DIR" -maxdepth 1 ! -name ".*" ! -path "$DIR" -exec rm -rf {} \;

echo "Deleted all non-hidden files and directories in $DIR"
