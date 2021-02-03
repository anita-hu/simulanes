#!/bin/bash

version=$(cat $(dirname $(realpath -s $0))/"image_version.txt")

cd server && docker build --rm --tag simulanes/carla:$version .
