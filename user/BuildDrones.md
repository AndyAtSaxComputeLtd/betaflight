## Drones to build for 

# version
# Betaflight / STM32F411 (S411) 4.5.1 Dec 23 2024 / 12:51:51 (77d01ba3b) MSP API: 1.46
# board_name TAKERF411
make --no-print-directory fwo CONFIG=TAKERF411

# GEP Vapour D5
# Betaflight / STM32F7X2 (S7X2) 4.5.1 Sep 25 2024 / 06:04:24 (77d01ba3b) MSP API: 1.46
# board_name GEPRCF722

make --no-print-directory fwo CONFIG=GEPRCF722

make --no-print-directory fwo CONFIG=GEPRCF722 OPTIONS="USE_SERIALRX_CRSF USE_TELEMETRY_CRSF USE_OSD USE_OSD_HD USE_MSP_DISPLAYPORT USE_DSHOT USE_GPS USE_LED_STRIP USE_ACRO_TRAINER USE_PINIO USE_RANGEFINDER USE_SOFTSERIAL USE_VTX USE_OPTICALFLOW_MT"

# Convert config
pwsh -NoLogo -NoProfile -File user/Convert-BetaflightCliTo2025.ps1 -InputPath user/BTFL_cli_VAPOR-D5_20260703_134740_GEPRCF722_Dump_All.txt -OutputPath user/BTFL_cli_VAPOR-D5_20260703_134740_GEPRCF722_Dump_All_2025.txt