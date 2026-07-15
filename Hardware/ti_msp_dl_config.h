/*
 * 修改原因：Keil 工程把 Hardware 目录加入了头文件搜索路径，而 SysConfig 生成文件在工程根目录；
 * 通过这个转发头文件兼容两种包含路径，避免原来的 ti_msp_dl_config.h 找不到错误。
 */
#ifndef HARDWARE_TI_MSP_DL_CONFIG_FORWARD_H
#define HARDWARE_TI_MSP_DL_CONFIG_FORWARD_H

#include "../ti_msp_dl_config.h"

#endif
