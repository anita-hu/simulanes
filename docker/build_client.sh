#!/bin/bash

version=$(cat $(dirname $(realpath -s $0))/"image_version.txt")

cd client && docker build --rm --tag simulanes/dev:$version .
