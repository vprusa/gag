#!/bin/bash

PROCESSING_IDE_PATH=~/IDE/processing-3.4-linux64/processing-3.4/processing
ECLIPSE_IDE_PATH=~/workspace/devstudio/latestDevstudio
ECLIPSE_GAG_WORKSPACE=~/ws
THIS_DIR_PATH=$(dirname `realpath "$0"`)
echo "BASH_SOURCE[0]: ${BASH_SOURCE[0]}"
PROCESSING_PROJECT_PATH=${THIS_DIR_PATH}/proto/processing/MPUTeapot/MPUTeapot.pde
echo "PROCESSING_PROJECT_PATH: ${PROCESSING_PROJECT_PATH}"
echo "PROCESSING_IDE_PATH: ${PROCESSING_IDE_PATH}"
#${PROCESSING_IDE_PATH} ${PROCESSING_PROJECT_PATH}

CODE_PROJECT_PATH=${THIS_DIR_PATH}/proto/MPU6050_DMP6-3
echo "CODE_PROJECT_PATH: ${CODE_PROJECT_PATH}"
code ${CODE_PROJECT_PATH}

${ECLIPSE_IDE_PATH} -data ${ECLIPSE_GAG_WORKSPACE}
