/**
 * bsp_track.c  Hiwonder LineFollower_8CH 八路巡线模块驱动（软件 I2C，读寄存器 5）
 *
 * 为什么用 I2C：该模块（Hiwonder LineFollower_8CH v1.0）的 UART 固件行为
 * 和手头实测对不上（上电只发 8 字节就沉默），而官方文档的 I2C 接口
 * 完全透明：I2C 地址 0x5D，寄存器 5 = 8 通道数字状态（1 字节），
 * 无需任何配置命令，直接读即可，最稳。
 *
 * 接线：模块 SCL -> PA2，模块 SDA -> PA3（软件 I2C，开漏），
 *       模块 5V/GND 照常。上电后建议先按 KEY 做一次一键校准。
 */
#include "bsp_track.h"
#include "app_config.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

#define TRACK_POLL_MS  10   /* 读一次全通道状态的周期（与控制周期对齐，感知延迟≈0~10ms） */

/* 软件 I2C 引脚：SCL = PA2，SDA = PA3 */
#define TRK_SCL_PIN   GPIO_PIN_2
#define TRK_SDA_PIN   GPIO_PIN_3
#define TRK_PORT      GPIOA

#define TRK_I2C_ADDR_W  (0x5D << 1)        /* 0xBA */
#define TRK_I2C_ADDR_R  ((0x5D << 1) | 1u) /* 0xBB */
#define TRK_REG_ANALOG  6u                 /* 通道 1 模拟值寄存器（共 8 通道 × 2 字节） */
#define TRK_ANALOG_BLACK_TH  1200u         /* 模拟值 ≥ 此阈值判为黑线（实测：白底~286，黑线~2300+） */

char g_track_last_frame[96];

/* 模拟量：8 个探头的原始红外测值（白底高、黑线低，具体以实测为准） */
uint16_t g_track_analog[8];

/* 调试探针 */
char     g_track_raw[128];
uint16_t g_track_raw_len;
uint32_t g_track_err;
uint32_t g_track_err_stage[4]; /* I2C 失败阶段统计：[1]=写地址无应答 [2]=写寄存器无应答 [3]=读地址无应答 */

/* I2C 地址扫描结果：上电后自动把 1~127 全部喊一遍，谁应答记谁 */
uint8_t  g_i2c_found[8];
uint8_t  g_i2c_found_cnt;

static TrackData_t s_data;
static float       s_ref;
static uint32_t    s_last_poll;
static uint8_t     s_i2c_addr  = 0x5D;   /* 7 位地址，扫描后自动更新 */
static uint8_t     s_scan_done;

/* ---------------- 软件 I2C（开漏，读引脚电平用 IDR） ---------------- */

static void trk_dly(void)
{
    volatile uint32_t n = 40;          /* 72MHz 下约 2~3us */
    while (n--) { __NOP(); }
}

static void scl_h(void) { HAL_GPIO_WritePin(TRK_PORT, TRK_SCL_PIN, GPIO_PIN_SET); }
static void scl_l(void) { HAL_GPIO_WritePin(TRK_PORT, TRK_SCL_PIN, GPIO_PIN_RESET); }
static void sda_h(void) { HAL_GPIO_WritePin(TRK_PORT, TRK_SDA_PIN, GPIO_PIN_SET); }
static void sda_l(void) { HAL_GPIO_WritePin(TRK_PORT, TRK_SDA_PIN, GPIO_PIN_RESET); }
static uint8_t sda_read(void)
{
    return (HAL_GPIO_ReadPin(TRK_PORT, TRK_SDA_PIN) == GPIO_PIN_SET) ? 1u : 0u;
}

static void track_pins_as_i2c(void)
{
    GPIO_InitTypeDef gp = {0};

    HAL_GPIO_DeInit(TRK_PORT, TRK_SCL_PIN | TRK_SDA_PIN);  /* 摆脱 USART2 复用 */
    gp.Pin   = TRK_SCL_PIN | TRK_SDA_PIN;
    gp.Mode  = GPIO_MODE_OUTPUT_OD;
    gp.Pull  = GPIO_PULLUP;
    gp.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(TRK_PORT, &gp);

    scl_h();
    sda_h();
}

static void i2c_start(void)
{
    sda_h(); scl_h(); trk_dly();
    sda_l(); trk_dly();
    scl_l(); trk_dly();
}

static void i2c_stop(void)
{
    sda_l(); trk_dly();
    scl_h(); trk_dly();
    sda_h(); trk_dly();
}

/* 写 1 字节，返回 ACK：0 = 收到应答 */
static uint8_t i2c_wr_byte(uint8_t b)
{
    uint8_t i, nack;

    for (i = 0; i < 8; i++)
    {
        if (b & 0x80u) sda_h(); else sda_l();
        b <<= 1;
        trk_dly();
        scl_h(); trk_dly();
        scl_l();
    }
    sda_h();                            /* 释放 SDA 读 ACK */
    trk_dly();
    scl_h(); trk_dly();
    nack = sda_read();
    scl_l();
    return nack;
}

/* 读 1 字节；ack = 1 回应答，0 回非应答 */
static uint8_t i2c_rd_byte(uint8_t ack)
{
    uint8_t i, b = 0;

    sda_h();
    for (i = 0; i < 8; i++)
    {
        trk_dly();
        scl_h(); trk_dly();
        b = (uint8_t)((b << 1) | sda_read());
        scl_l();
    }
    if (ack) sda_l(); else sda_h();
    trk_dly();
    scl_h(); trk_dly();
    scl_l();
    sda_h();
    return b;
}

/* I2C 地址扫描：把 1~127 全部喊一遍，记录所有应答的地址 */
static void track_i2c_scan(void)
{
    uint8_t a;

    g_i2c_found_cnt = 0;
    for (a = 1; a < 128; a++)
    {
        i2c_start();
        if (i2c_wr_byte((uint8_t)(a << 1)) == 0u)
        {
            if (g_i2c_found_cnt < sizeof(g_i2c_found))
                g_i2c_found[g_i2c_found_cnt++] = a;
        }
        i2c_stop();
    }

    if (g_i2c_found_cnt > 0u)
        s_i2c_addr = g_i2c_found[0];    /* 自动采用扫到的第一个地址 */
}

/* 读 8 通道模拟量（寄存器 6 起，16 字节，低字节在前），成功返回 1。
   写和读之间用 STOP 隔开（部分固件不支持 repeated start） */
static uint8_t track_i2c_read_analog(uint16_t out[8])
{
    uint8_t buf[16];
    uint8_t i;

    i2c_start();
    if (i2c_wr_byte((uint8_t)(s_i2c_addr << 1))) { i2c_stop(); g_track_err_stage[1]++; return 0; }
    if (i2c_wr_byte(TRK_REG_ANALOG))             { i2c_stop(); g_track_err_stage[2]++; return 0; }
    i2c_stop();

    i2c_start();
    if (i2c_wr_byte((uint8_t)((s_i2c_addr << 1) | 1u))) { i2c_stop(); g_track_err_stage[3]++; return 0; }
    for (i = 0; i < 15; i++) buf[i] = i2c_rd_byte(1);   /* 前 15 字节回 ACK */
    buf[15] = i2c_rd_byte(0);                           /* 末字节回 NACK */
    i2c_stop();

    for (i = 0; i < 8; i++)
        out[i] = (uint16_t)(buf[i * 2u] | ((uint16_t)buf[i * 2u + 1u] << 8));
    return 1;
}

/* ---------------- 对外接口 ---------------- */

void Track_Init(void)
{
    memset(&s_data, 0, sizeof(s_data));
    s_data.lost   = 1;
    s_ref         = 0.0f;
    s_last_poll   = 0;
    track_pins_as_i2c();
}

void Track_Calibrate(void)
{
    /* 校准通过模块板上 KEY 键完成 */
}

void Track_SetRef(float pos_mm)
{
    s_ref = pos_mm;
}

const TrackData_t *Track_Get(void)
{
    return &s_data;
}

uint8_t Track_IsAlive(uint32_t now_ms)
{
    return (s_data.frame_cnt > 0u) &&
           ((uint32_t)(now_ms - s_data.last_frame_tick) < CFG_TRACK_TIMEOUT_MS);
}

/* 根据 black[] 计算线位置：连续黑点分块，取离 s_ref 最近那块的中心 */
static void track_locate(void)
{
    float   pitch  = g_param.track_pitch_mm;
    float   best_c = 0.0f;
    float   best_d = 1e9f;
    uint8_t cnt = 0, ncl = 0;
    int     i = 0;

    while (i < 8)
    {
        int     j;
        float   sum = 0.0f;
        uint8_t n = 0;

        if (!s_data.black[i]) { i++; continue; }
        for (j = i; j < 8 && s_data.black[j]; j++)
        {
            sum += ((float)j - 3.5f) * pitch;
            n++;
        }
        {
            float c = sum / (float)n;
            float d = fabsf(c - s_ref);
            if (d < best_d) { best_d = d; best_c = c; }
        }
        ncl++;
        cnt = (uint8_t)(cnt + n);
        i = j;
    }

    s_data.bits = 0;
    for (i = 0; i < 8; i++)
        s_data.bits |= (uint8_t)(s_data.black[i] << i);

    s_data.black_cnt   = cnt;
    s_data.cluster_cnt = ncl;
    if (cnt == 0u)
    {
        s_data.lost = 1;           /* pos_mm 保留上次值，供找线判断方向 */
    }
    else
    {
        s_data.lost   = 0;
        s_data.pos_mm = best_c;
        s_ref         = best_c;
    }
}

void Track_Process(uint32_t now_ms)
{
    /* 上电等模块启动完成后，先做一次 I2C 地址扫描并锁定地址 */
    if (!s_scan_done)
    {
        if (now_ms >= CFG_TRACK_BOOT_MS)
        {
            track_i2c_scan();
            s_scan_done = 1;
        }
        return;
    }

    if ((uint32_t)(now_ms - s_last_poll) < TRACK_POLL_MS)
        return;
    s_last_poll = now_ms;

    /* 读模拟量（原始红外测值，随黑白真实变化），失败则跳过本周期 */
    if (!track_i2c_read_analog(g_track_analog))
    {
        g_track_err++;              /* I2C 无应答等错误 */
        return;
    }

    /* 由模拟量推导各通道黑白：实测黑线=高值（白底~286，黑线~2300+），
       阈值 TRK_ANALOG_BLACK_TH，不再依赖模块的数字状态寄存器和 KEY 校准 */
    {
        uint8_t cnt = 0;
        uint8_t ncl = 0;
        int     i;

        for (i = 0; i < 8; i++)
        {
            uint8_t b = (g_track_analog[i] >= TRK_ANALOG_BLACK_TH) ? 1u : 0u;

            s_data.raw[i] = (uint8_t)(g_track_analog[i] >> 4);   /* 压缩到 0~255 便于观察 */
#if CFG_TRACK_X1_IS_LEFT
            s_data.black[i] = b;
#else
            s_data.black[7u - i] = b;
#endif
            cnt = (uint8_t)(cnt + b);
        }
        for (i = 0; i < 8; i++)
            if (s_data.black[i] && (i == 0 || !s_data.black[i - 1u]))
                ncl++;

        s_data.black_cnt   = cnt;
        s_data.cluster_cnt = ncl;

        track_locate();

        s_data.frame_cnt++;
        s_data.last_frame_tick = HAL_GetTick();
    }

    /* 调试显示：8 个模拟值 */
    sprintf(g_track_last_frame, "A%u,%u,%u,%u,%u,%u,%u,%u",
            g_track_analog[0], g_track_analog[1], g_track_analog[2], g_track_analog[3],
            g_track_analog[4], g_track_analog[5], g_track_analog[6], g_track_analog[7]);
}
