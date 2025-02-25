#!/bin/bash

# Define the target directory (or use the current directory)
DIR="${1:-.}"

# Find and delete files that don't start with a dot
find "$DIR" -maxdepth 1 -type f ! -name ".*" -exec rm -f {} \;

echo "Deleted all non-hidden files in $DIR"
