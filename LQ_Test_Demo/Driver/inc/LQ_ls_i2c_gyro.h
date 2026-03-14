#pragma once

extern signed short aac_x, aac_y, aac_z;    // 加速度传感器原始数据
extern signed short gyro_x, gyro_y, gyro_z; // 陀螺仪原始数据
extern uint8_t IIC_ICM20602;
extern uint8_t IIC_ICM20689 ;
extern uint8_t IIC_MPU9250 ;
extern uint8_t IIC_ICM42605 ;
extern uint8_t IIC_MPU6050 ;
extern uint8_t IIC_errorid ;

// AK8963的内部寄存器 9250
#define MPU_MAG_WIA          0X00 // AK8963的器件ID寄存器地址
#define AK8963_ADDR          0x0C // AK8963的I2C地址
#define AK8963_ID            0x48 // AK8963的器件ID

#define MPU_MAG_XOUTL_REG    0X03 // 磁力计值,X轴低8位寄存器
#define MPU_MAG_XOUTH_REG    0X04 // 磁力计值,X轴高8位寄存器
#define MPU_MAG_YOUTL_REG    0X05 // 磁力计值,Y轴低8位寄存器
#define MPU_MAG_YOUTH_REG    0X06 // 磁力计值,Y轴高8位寄存器
#define MPU_MAG_ZOUTL_REG    0X07 // 磁力计值,Z轴低8位寄存器
#define MPU_MAG_ZOUTH_REG    0X08 // 磁力计值,Z轴高8位寄存器
#define MPU_MAG_ST1_REG      0X02 // 磁力计读取状态
#define MPU_MAG_ST2_REG      0X09 // 磁力计读取状态
#define MPU_MAG_CNTL1_REG    0X0A // 磁力计模式控制
#define MPU_MAG_CNTL2_REG    0X0B // 磁力计模式控制

//****************************************
// 定义MPU6050内部地址
//****************************************
// MPU6500的内部寄存器
#define MPU_SELF_TESTX_REG   0X0D // 自检寄存器X
#define MPU_SELF_TESTY_REG   0X0E // 自检寄存器Y
#define MPU_SELF_TESTZ_REG   0X0F // 自检寄存器z
#define MPU_SELF_TESTA_REG   0X10 // 自检寄存器A
#define MPU_SAMPLE_RATE_REG  0X19 // 采样频率分频器
#define MPU_CFG_REG          0X1A // 配置寄存器
#define MPU_GYRO_CFG_REG     0X1B // 陀螺仪配置寄存器
#define MPU_ACCEL_CFG_REG    0X1C // 加速度计配置寄存器
#define MPU_MOTION_DET_REG   0X1F // 运动检测阀值设置寄存器
#define MPU_FIFO_EN_REG      0X23 // FIFO使能寄存器

#define MPU_I2CMST_STA_REG   0X36 // IIC主机状态寄存器
#define MPU_INTBP_CFG_REG    0X37 // 中断/旁路设置寄存器
#define MPU_INT_EN_REG       0X38 // 中断使能寄存器
#define MPU_INT_STA_REG      0X3A // 中断状态寄存器

#define MPU_ACCEL_XOUTH_REG  0X3B // 加速度值,X轴高8位寄存器
#define MPU_ACCEL_XOUTL_REG  0X3C // 加速度值,X轴低8位寄存器
#define MPU_ACCEL_YOUTH_REG  0X3D // 加速度值,Y轴高8位寄存器
#define MPU_ACCEL_YOUTL_REG  0X3E // 加速度值,Y轴低8位寄存器
#define MPU_ACCEL_ZOUTH_REG  0X3F // 加速度值,Z轴高8位寄存器
#define MPU_ACCEL_ZOUTL_REG  0X40 // 加速度值,Z轴低8位寄存器

#define MPU_TEMP_OUTH_REG    0X41 // 温度值高八位寄存器
#define MPU_TEMP_OUTL_REG    0X42 // 温度值低8位寄存器

#define MPU_GYRO_XOUTH_REG   0X43 // 陀螺仪值,X轴高8位寄存器
#define MPU_GYRO_XOUTL_REG   0X44 // 陀螺仪值,X轴低8位寄存器
#define MPU_GYRO_YOUTH_REG   0X45 // 陀螺仪值,Y轴高8位寄存器
#define MPU_GYRO_YOUTL_REG   0X46 // 陀螺仪值,Y轴低8位寄存器
#define MPU_GYRO_ZOUTH_REG   0X47 // 陀螺仪值,Z轴高8位寄存器
#define MPU_GYRO_ZOUTL_REG   0X48 // 陀螺仪值,Z轴低8位寄存器

#define MPU_I2CSLV0_DO_REG   0X63 // IIC从机0数据寄存器
#define MPU_I2CSLV1_DO_REG   0X64 // IIC从机1数据寄存器
#define MPU_I2CSLV2_DO_REG   0X65 // IIC从机2数据寄存器
#define MPU_I2CSLV3_DO_REG   0X66 // IIC从机3数据寄存器

#define MPU_I2CMST_DELAY_REG 0X67 // IIC主机延时管理寄存器
#define MPU_SIGPATH_RST_REG  0X68 // 信号通道复位寄存器
#define MPU_MDETECT_CTRL_REG 0X69 // 运动检测控制寄存器
#define MPU_USER_CTRL_REG    0X6A // 用户控制寄存器
#define MPU_PWR_MGMT1_REG    0X6B // 电源管理寄存器1
#define MPU_PWR_MGMT2_REG    0X6C // 电源管理寄存器2
#define MPU_FIFO_CNTH_REG    0X72 // FIFO计数寄存器高八位
#define MPU_FIFO_CNTL_REG    0X73 // FIFO计数寄存器低八位
#define MPU_FIFO_RW_REG      0X74 // FIFO读写寄存器
#define WHO_AM_I             0X75 // 器件ID寄存器

//****************************************
// 定义ICM42605内部地址
//****************************************
// ICM42605的内部寄存器
#define ICM42605_TEMP_OUTH_REG   0X1D // 温度值高八位寄存器
#define ICM42605_TEMP_OUTL_REG   0X1E // 温度值低8位寄存器

#define ICM42605_ACCEL_XOUTH_REG 0X1F // 加速度值,X轴高8位寄存器
#define ICM42605_ACCEL_XOUTL_REG 0X20 // 加速度值,X轴低8位寄存器
#define ICM42605_ACCEL_YOUTH_REG 0X21 // 加速度值,Y轴高8位寄存器
#define ICM42605_ACCEL_YOUTL_REG 0X22 // 加速度值,Y轴低8位寄存器
#define ICM42605_ACCEL_ZOUTH_REG 0X23 // 加速度值,Z轴高8位寄存器
#define ICM42605_ACCEL_ZOUTL_REG 0X24 // 加速度值,Z轴低8位寄存器

#define ICM42605_GYRO_XOUTH_REG  0X25 // 陀螺仪值,X轴高8位寄存器
#define ICM42605_GYRO_XOUTL_REG  0X26 // 陀螺仪值,X轴低8位寄存器
#define ICM42605_GYRO_YOUTH_REG  0X27 // 陀螺仪值,Y轴高8位寄存器
#define ICM42605_GYRO_YOUTL_REG  0X28 // 陀螺仪值,Y轴低8位寄存器
#define ICM42605_GYRO_ZOUTH_REG  0X29 // 陀螺仪值,Z轴高8位寄存器
#define ICM42605_GYRO_ZOUTL_REG  0X2A // 陀螺仪值,Z轴低8位寄存器

#define reg_bank_sel               0x76
#define device_config_reg          0x11
#define bit_spi_mode               0x10
#define bit_soft_reset_chip_config 0x01

#define intf_config4  0x7A
#define pwr_mgmt0_reg 0x4E
// #define bit_temp_dis               0x20
#define bit_idle      0x10
#define bit_gyro_mode_mask    ((0x03) << 2)
#define bit_gyro_mode_off     ((0x00) << 2)
#define bit_gyro_mode_standby ((0x01) << 2)
// #define bit_gyro_mode_lp           ((0x02)<<2)
#define bit_gyro_mode_ln    ((0x03) << 2)
#define bit_accel_mode_mask ((0x03) << 0)
#define bit_accel_mode_off    0x00
#define bit_accel_mode_lp     0x02
#define bit_accel_mode_ln     0x03

#define gyro_config0_reg             0x4F
#define bit_gyro_ui_fs_sel_shift     5
#define bit_gyro_ui_fs_sel_2000dps ((0x00) << bit_gyro_ui_fs_sel_shift)
#define bit_gyro_ui_fs_sel_1000dps ((0x01) << bit_gyro_ui_fs_sel_shift)
#define bit_gyro_ui_fs_sel_mask    ((0x07) << bit_gyro_ui_fs_sel_shift)
#define bit_gyro_odr_100hz         ((0x08) << 0)
#define bit_gyro_odr_50hz          ((0x09) << 0)
#define bit_gyro_odr_nonflame_mask ((0x0F) << 0)

#define accel_config0_reg             0x50
#define bit_accel_ui_fs_sel_shift     5
#define bit_accel_ui_fs_sel_8g      ((0x01) << bit_accel_ui_fs_sel_shift)
#define bit_accel_ui_fs_sel_mask    ((0x07) << bit_accel_ui_fs_sel_shift)
#define bit_accel_odr_100hz         ((0x08) << 0)
#define bit_accel_odr_50hz          ((0x09) << 0)
#define bit_accel_odr_nonflame_mask ((0x0F) << 0)

#define int_source0_reg            0x65
#define bit_int_ui_fsync_int1_en   0x40
#define bit_int_pll_rdy_int1_en    0x20
#define bit_int_reset_done_int1_en 0x10
#define bit_int_ui_drdy_int1_en    0x08
#define bit_int_fifo_ths_int1_en   0x04 // FIFO threshold interrupt
#define bit_int_fifo_full_int1_en  0x02
#define bit_int_ui_agc_rdy_int1_en 0x01

#define sensor_selftest_reg 0x6B
#define bit_accel_st_result 0x08
#define bit_gyro_st_result  0x04
#define bit_accel_st_status 0x02
#define bit_gyro_st_status  0x01

#define int_config_reg         0x14
#define bit_int2_mode          0x20
#define bit_int2_drive_circuit 0x10
#define bit_int2_polarity      0x08
#define bit_int1_mode          0x04
#define bit_int1_drive_circuit 0x02
#define bit_int1_polarity      0x01

#define fifo_config_reg               0x16
#define bit_fifo_mode_ctrl_mask     ((0x03) << 6)
#define bit_fifo_mode_ctrl_bypass   ((0x00) << 6)
#define bit_fifo_mode_ctrl_stream   ((0x01) << 6)
#define bit_fifo_mode_ctrl_snapshot ((0x02) << 6)

#define tmst_config_reg      0x54
#define bit_fifo_ram_iso_ena 0x40
#define bit_en_dreg_fifo_d2a 0x20
#define bit_tmst_to_regs_en  0x10
#define bit_tmst_resol       0x08
#define bit_tmst_delta_en    0x04
#define bit_tmst_fsync_en    0x02
#define bit_tmst_en          0x01

#define fifo_config2_reg 0x60
#define fifo_config3_reg 0x61

#define fsync_config_reg            0x62
#define bit_fsync_ui_sel_mask     ((0x07) << 4)
#define bit_fsync_ui_sel_tag_temp ((0x01) << 4)
#define bit_fsync_ui_flag_clear_sel 0x02

#define fifo_config1_reg        0x5F
#define bit_fifo_resume_partial 0x40
#define bit_fifo_wm_gt_th       0x20
#define bit_fifo_hires_en       0x10
#define bit_fifo_tmst_fsync_en  0x08
#define bit_fifo_temp_en        0x04
#define bit_fifo_gyro_en        0x02
#define bit_fifo_accel_en       0x01

#define int_config0_reg         0x63
#define int_config1_reg         0x64
#define bit_int_asy_rst_disable 0x10

#define fifo_byte_count_h_res 0x2E
#define fifo_byte_count_l_res 0x2F

#define fifo_accel_en 0x40
#define fifo_gyro_en  0x20

#define fifo_data_port 0x30

//****************42605寄存器**********************************************************************************
