#!/bin/bash

# Get the parent directory of the script
DIR="$(dirname "$(realpath "$0")")/.."

# Find and delete files that don't start with a dot in the parent directory
find "$DIR" -maxdepth 1 -type f ! -name ".*" -exec rm -f {} \;

echo "Deleted all non-hidden files in $DIR"
