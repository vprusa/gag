#!/bin/bash

PROCESSING_IDE_PATH=~/IDE/processing-3.4-linux64/processing-3.4/processing
ECLIPSE_IDE_PATH=~/workspace/devstudio/latestDevstudio
ECLIPSE_GAG_WORKSPACE=~/ws
PROJECT_PATH=~/workspace/p/notes/work/projects/arduino/gag/gag-web
THIS_DIR_PATH=$(dirname `realpath "$0"`)
IDEA_IDE_PATH=~/IDE/idea/idea-IC-192.5728.98/bin/idea.sh
IDEA_WEB_IDE_PATH=~/IDE/WebStorm-193.6494.34/bin/webstorm.sh
ANDROID_STUDIO_IDE_PATH=~/IDE/android-studio/bin/studio.sh


echo "Opening temrinator, goto"
echo "~/workspace/p/notes/work/projects/arduino/gag/gag-web"
cd ${PROJECT_PATH} && terminator -e "echo 'mvn wildfly:start wildfly:undeploy install wildfly:deploy -DskipTests=true -Dcheckstyle.skip | tee app.log ' && echo 'mvn clean wildfly:undeploy install wildfly:deploy -DskipTests=true -Dcheckstyle.skip | tee app.log' && echo './config/startKC.sh' && echo './wildfly-home/bin/standalone.sh --debug' && echo 'cd services && mvn clean install test -Dtest=RecognitionTest -Dcheckstyle.skip | tee test.log' && zsh " & disown

echo "BASH_SOURCE[0]: ${BASH_SOURCE[0]}"
PROCESSING_PROJECT_PATH=${THIS_DIR_PATH}/proto/processing/MPUTeapot/MPUTeapot.pde
echo "PROCESSING_PROJECT_PATH: ${PROCESSING_PROJECT_PATH}"
echo "PROCESSING_IDE_PATH: ${PROCESSING_IDE_PATH}"
#${PROCESSING_IDE_PATH} ${PROCESSING_PROJECT_PATH}

CODE_PROJECT_PATH=${THIS_DIR_PATH}/proto/gag
echo "CODE_PROJECT_PATH: ${CODE_PROJECT_PATH}"
code ${CODE_PROJECT_PATH}

#${ECLIPSE_IDE_PATH} -data ${ECLIPSE_GAG_WORKSPACE}
${IDEA_IDE_PATH} & disown
${IDEA_WEB_IDE_PATH} & disown
${ANDROID_STUDIO_IDE_PATH} & disown 
