#!/bin/bash

git add .
read -p "enter commit message = " msg
git commit -am "$msg"
git push origin master
