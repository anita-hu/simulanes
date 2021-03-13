#!/bin/bash

parent=$(dirname $(realpath -s $0))
version=$(cat $parent/"image_version.txt")

docker build --rm --tag simulanes/dev:$version $parent
