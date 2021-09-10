#!/bin/bash
export AMSDK=/c/version-control/SparkFun_Apollo3_AmbiqSuite_BSPs

BSP_PATH=/c/version-control/EDD-LCP_Control/Firmware/LCP_Control/src/bsp
BOARDS_FILE=/c/version-control/EDD-LCP_Control/Firmware/tools/configuration/boards.sh

$AMSDK/common/tools_sfe/scripts/regen_bsp_pins.sh -r $BSP_PATH -b $BOARDS_FILE
