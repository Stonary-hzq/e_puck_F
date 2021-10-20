#!/bin/bash

#do not change the lines numbers without changing them in the Create Eclipse_e-puck2.sh script
#because it is used to actualise the line with arm-none-eabi

dir=$PWD

gvfs-set-attribute -t string $dir/Eclipse_e-puck2 metadata::custom-icon file:///$dir/eclipse/icon.xpm

#below is the line changed by the script
export PATH=$dir/Tools/gcc-arm-none-eabi-7-2017-q4-major/bin:$PATH

export PATH=/usr/lib/jvm/java-8-openjdk/bin:$PATH
export PATH=/usr/lib/jvm/java-8-openjdk-amd64/bin:$PATH

eclipse/eclipse -data Workspace
