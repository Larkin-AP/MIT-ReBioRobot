#!/bin/bash
set -e
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"


cd ${DIR}/../build/
rm -rf robot-software
mkdir robot-software
mkdir robot-software/build
#cp common/test-common robot-software/build
cp $1 robot-software/build
find . -name \*.so* -exec cp {} ./robot-software/build \;
cp ../scripts/run_mc* ./robot-software/build
cp ../scripts/setup_network_mc.py ./robot-software/build
cp ../scripts/run_mc_debug.sh ./robot-software/build
cp ../scripts/config_network_lcm.sh ./robot-software
cp -r ../robot robot-software
cp -r ../config robot-software

DATE=$(date +"%Y%m%d%H%M")
#scp -r robot-software user@10.0.0.34:~/robot-software-$DATE/

#need change the address
if [ -z "$2" ]
then
  echo "No mini-cheetah number specified, using old mini-cheetah address"
  scp -r robot-software robot@10.16.102.144:~/test2/
else
  scp -r robot-software robot@10.16.102.144$2:~/test2/
fi


