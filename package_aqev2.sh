#!/bin/bash

ARGC=$#
version=$1

echo "Number of args: $ARGC"

if [ $ARGC -ne 1 ]
then
  echo "version string required."
  exit
fi

rm -rf WildFire-Arduino-Core
rm -rf WildFire
rm -rf WildFire_CC3000_Library
rm -rf WildFire_SPIFlash
rm -rf TinyWatchdog
rm -rf AQEV2FW
rm -rf AQEV2FW_SO2O3
rm -rf AQEV2FW_PM
rm -rf LMP91000
rm -rf SHT25
rm -rf MCP342x
rm -rf pubsubclient
rm -rf CapacitiveSensor
rm -rf Time
rm -rf RTClib
rm -rf SdFat
rm -rf TinyGPS
rm package_airqualityegg-$version\_index.json

git clone https://github.com/WickedDevice/WildFire-Arduino-Core.git
git clone https://github.com/WickedDevice/WildFire.git
git clone https://github.com/WickedDevice/WildFire_CC3000_Library.git
git clone https://github.com/WickedDevice/WildFire_SPIFlash.git
git clone https://github.com/WickedDevice/TinyWatchdog.git
git clone https://github.com/WickedDevice/AQEV2FW.git
git clone https://github.com/WickedDevice/AQEV2FW_SO2O3.git
git clone https://github.com/WickedDevice/AQEV2FW_PM.git
git clone https://github.com/WickedDevice/LMP91000
git clone https://github.com/WickedDevice/SHT25
git clone https://github.com/stevemarple/MCP342x
git clone https://github.com/vicatcu/pubsubclient
git clone https://github.com/PaulStoffregen/CapacitiveSensor
git clone https://github.com/PaulStoffregen/Time
git clone https://github.com/mizraith/RTClib
git clone https://github.com/greiman/SdFat
git clone https://github.com/mikalhart/TinyGPS

rm -rf AirQualityEgg
rm -rf libraries

mkdir AirQualityEgg
mkdir AirQualityEgg/libraries
mkdir tmp 

cd WildFire-Arduino-Core
git archive master | tar -x -C ../tmp
cd ../tmp/WickedDevice
sed -i -e '$a\' platform.txt
echo "name=Air Quality Egg" >> platform.txt
mv * ../../AirQualityEgg/
cd ../  

cd ../WildFire
mkdir ../AirQualityEgg/libraries/WildFire
git archive master | tar -x -C ../AirQualityEgg/libraries/WildFire

cd ../WildFire_CC3000_Library
mkdir ../AirQualityEgg/libraries/WildFire_CC3000_Library
git archive master | tar -x -C ../AirQualityEgg/libraries/WildFire_CC3000_Library

cd ../WildFire_SPIFlash
mkdir ../AirQualityEgg/libraries/WildFire_SPIFlash
git archive master | tar -x -C ../AirQualityEgg/libraries/WildFire_SPIFlash

cd ../TinyWatchdog
mkdir ../AirQualityEgg/libraries/TinyWatchdog
git archive master | tar -x -C ../AirQualityEgg/libraries/TinyWatchdog

cd ../AQEV2FW
git checkout tags/$version
githash=`git rev-parse HEAD`
mkdir ../AirQualityEgg/libraries/AQEV2FW
mkdir ../AirQualityEgg/libraries/AQEV2FW/examples
mkdir ../AirQualityEgg/libraries/AQEV2FW/examples/AQEV2FW
git archive master | tar -x -C ../AirQualityEgg/libraries/AQEV2FW/examples/AQEV2FW

cd ../AQEV2FW_SO2O3
git checkout tags/$version
mkdir ../AirQualityEgg/libraries/AQEV2FW_SO2O3
mkdir ../AirQualityEgg/libraries/AQEV2FW_SO2O3/examples
mkdir ../AirQualityEgg/libraries/AQEV2FW_SO2O3/examples/AQEV2FW_SO2O3
git archive master | tar -x -C ../AirQualityEgg/libraries/AQEV2FW_SO2O3/examples/AQEV2FW_SO2O3

cd ../AQEV2FW_PM
git checkout tags/$version
mkdir ../AirQualityEgg/libraries/AQEV2FW_PM
mkdir ../AirQualityEgg/libraries/AQEV2FW_PM/examples
mkdir ../AirQualityEgg/libraries/AQEV2FW_PM/examples/AQEV2FW_PM
git archive master | tar -x -C ../AirQualityEgg/libraries/AQEV2FW_PM/examples/AQEV2FW_PM


cd ../LMP91000
mkdir ../AirQualityEgg/libraries/LMP91000
git archive master | tar -x -C ../AirQualityEgg/libraries/LMP91000

cd ../SHT25
mkdir ../AirQualityEgg/libraries/SHT25
git archive master | tar -x -C ../AirQualityEgg/libraries/SHT25

cd ../MCP342x
mkdir ../AirQualityEgg/libraries/MCP342x
git archive master | tar -x -C ../AirQualityEgg/libraries/MCP342x

cd ../CapacitiveSensor
mkdir ../AirQualityEgg/libraries/CapacitiveSensor
git archive master | tar -x -C ../AirQualityEgg/libraries/CapacitiveSensor

cd ../Time
mkdir ../AirQualityEgg/libraries/Time
git archive master | tar -x -C ../AirQualityEgg/libraries/Time

cd ../RTClib
mkdir ../AirQualityEgg/libraries/RTClib
git archive master | tar -x -C ../AirQualityEgg/libraries/RTClib

cd ../TinyGPS
mkdir ../AirQualityEgg/libraries/TinyGPS
git archive master | tar -x -C ../AirQualityEgg/libraries/TinyGPS

cd ../tmp
rm -rf *
cd ../SdFat
mkdir ../AirQualityEgg/libraries/SdFat
git archive master | tar -x -C ../tmp
cd ../tmp/SdFat
cp -rf * ../../AirQualityEgg/libraries/SdFat
cd ../

cd ../tmp
rm -rf *
cd ../pubsubclient
mkdir ../AirQualityEgg/libraries/PubSubClient
git archive master | tar -x -C ../tmp
cd ../tmp/PubSubClient
sed -i 's/#define MQTT_MAX_PACKET_SIZE 128/#define MQTT_MAX_PACKET_SIZE 512/g' PubSubClient.h
cp -rf * ../../AirQualityEgg/libraries/PubSubClient
cd ../
  
cd ../

rm AQEV2-$version-Arduino-$githash.zip
zip -r AQEV2-$version-Arduino-$githash.zip AirQualityEgg

filesize=`stat --printf="%s" AQEV2-$version-Arduino-$githash.zip`
sha256=`sha256sum AQEV2-$version-Arduino-$githash.zip | awk '{print $1;}'`

cp package_template.json package_airqualityegg-$version\_index.json
replace_version="s/%version%/$version/g"
replace_githash="s/%githash%/$githash/g"
replace_sha256="s/%sha256%/$sha256/g"
replace_filesize="s/%filesize%/$filesize/g"
sed -i $replace_version package_airqualityegg-$version\_index.json
sed -i $replace_githash package_airqualityegg-$version\_index.json
sed -i $replace_sha256 package_airqualityegg-$version\_index.json
sed -i $replace_filesize package_airqualityegg-$version\_index.json

rm -rf tmp
rm -rf AirQualityEgg
rm -rf WildFire-Arduino-Core
rm -rf WildFire
rm -rf WildFire_CC3000_Library
rm -rf WildFire_SPIFlash
rm -rf TinyWatchdog
rm -rf AQEV2FW
rm -rf AQEV2FW_SO2O3
rm -rf AQEV2FW_PM
rm -rf LMP91000
rm -rf SHT25
rm -rf MCP342x
rm -rf pubsubclient
rm -rf CapacitiveSensor
rm -rf Time
rm -rf RTClib
rm -rf TinyGPS
rm -rf SdFat
