#ifndef BSP_TRACK_H
#define BSP_TRACK_H

#include <stdint.h>

typedef struct {
	uint8_t raw[8]; /* 模块原始值 x1~x8                        */
	uint8_t black[8]; /* 已按左右排好：[0] 最左，[7] 最右，1 = 黑 */
	uint8_t bits; /* black[] 打包成位，bit0 = 最左，方便观察  */
	uint8_t black_cnt; /* 压黑线的探头数                          */
	uint8_t cluster_cnt; /* 黑色连通块个数（2 个以上说明有分叉）    */
	uint8_t lost; /* 1 = 一个黑点都没有                      */
	float pos_mm; /* 线相对车中心的位置，右为正；丢线时保持  */
	uint32_t frame_cnt; /* 收到的有效帧数                          */
	uint32_t last_frame_tick; /* 最近一帧的 HAL_GetTick()                */
} TrackData_t;

void Track_Init(void);
void Track_Process(uint32_t now_ms); /* 主循环里调用：解析帧、掉线重发配置 */
const TrackData_t* Track_Get(void);
uint8_t Track_IsAlive(uint32_t now_ms);
void Track_SetRef(float pos_mm); /* 设定分叉时优先跟踪的位置 */
void Track_Calibrate(void); /* 发送校准指令（也可按模块 KEY 键） */

/* 最近一次读到的状态字节（"ST=0xXX"），调试时观察 */
extern char g_track_last_frame[96];

/* 调试探针 */
extern char     g_track_raw[128];
extern uint16_t g_track_raw_len;
extern uint32_t g_track_err;   /* I2C 读取失败次数 */
extern uint32_t g_track_err_stage[4];  /* 失败阶段统计：[1]=写地址无应答 [2]=写寄存器无应答 [3]=读地址无应答 */
extern uint8_t  g_i2c_found[8];      /* I2C 地址扫描结果：有应答的 7 位地址 */
extern uint8_t  g_i2c_found_cnt;     /* 扫到的地址个数 */

#endif
