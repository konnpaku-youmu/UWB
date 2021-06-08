#ifndef UWB_REGMAP_H
#define UWB_REGMAP_H

namespace uwb
{

#define REG_BYTE_SIZE           0x0002

#define REG_BAUD_RATE           0x0000
#define REG_MODBUS_ID           0x0001
#define REG_RANGING_MODE        0x0002
#define REG_DEVICE_MODE         0x0003
#define REG_DEVICE_ID           0x0004
#define REG_CHANNEL_SPEED       0x0005
#define REG_KALMAN_Q            0x0006
#define REG_KALMAN_R            0x0007
#define REG_ANT_DELAY           0x0008

#define REG_STA_A_POS_X         0x0009
#define REG_STA_A_POS_Y         0x000A
#define REG_STA_A_POS_Z         0x000B
#define REG_STA_B_EN            0x000C
#define REG_STA_B_POS_X         0x000D
#define REG_STA_B_POS_Y         0x000e
#define REG_STA_B_POS_Z         0x000f
#define REG_STA_C_EN            0x0010
#define REG_STA_C_POS_X         0x0011
#define REG_STA_C_POS_Y         0x0012
#define REG_STA_C_POS_Z         0x0013
#define REG_STA_D_EN            0x0014
#define REG_STA_D_POS_X         0x0015
#define REG_STA_D_POS_Y         0x0016
#define REG_STA_D_POS_Z         0x0017
#define REG_STA_E_EN            0x0018
#define REG_STA_E_POS_X         0x0019
#define REG_STA_E_POS_Y         0x001a
#define REG_STA_E_POS_Z         0x001b
#define REG_STA_F_EN            0x001c
#define REG_STA_F_POS_X         0x001d
#define REG_STA_F_POS_Y         0x001e
#define REG_STA_F_POS_Z         0x001f
#define REG_STA_G_EN            0x0020
#define REG_STA_G_POS_X         0x0021
#define REG_STA_G_POS_Y         0x0022
#define REG_STA_G_POS_Z         0x0023
#define REG_STA_H_EN            0x0024
#define REG_STA_H_POS_X         0x0025
#define REG_STA_H_POS_Y         0x0026
#define REG_STA_H_POS_Z         0x0027

#define REG_RANGING_EN          0x0028
#define REG_TAG_NUM             0x0029
#define REG_ACTIVE_TAG_NO       0x002A

#define REG_RD_RANGING_FLAG     0x002B
#define REG_RD_TAG_POS_X        0x002C    
#define REG_RD_TAG_POS_Y        0x002D
#define REG_RD_TAG_POS_Z        0x002E

#define REG_RD_RANGING_A_DIST   0x002F
#define REG_RD_RANGING_B_DIST   0x0030
#define REG_RD_RANGING_C_DIST   0x0031
#define REG_RD_RANGING_D_DIST   0x0032
#define REG_RD_RANGING_E_DIST   0x0033
#define REG_RD_RANGING_F_DIST   0x0034
#define REG_RD_RANGING_G_DIST   0x0035
#define REG_RD_RANGING_H_DIST   0x0036

#define REG_TAG_ID_LIST_HEAD    0x0037
#define REG_TAG_ID_LIST_SIZE    0x0032      // 50

#define REG_RD_FIRMWARE_VER     0x0069

}

#endif