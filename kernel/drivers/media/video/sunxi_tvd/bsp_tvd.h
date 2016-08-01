#ifndef __BSP_TVD_H__
#define __BSP_TVD_H__

#include <linux/kernel.h>

//typedef int 	     s32;
//typedef unsigned int	u32;

typedef enum
{
    TVD_PL_YUV422,
    TVD_PL_YUV420,
    TVD_MB_YUV420, 
}tvd_fmt_t;

typedef enum
{
    TVD_NTSC,
    TVD_PAL,
}tvd_mode_t;

typedef enum
{
    TVD_FRAME_DONE,
    TVD_LOCK,
    TVD_UNLOCK,
}tvd_irq_t;

void TVD_init(u32 addr);

//s32 TVD_set_mode(tvd_mode_t mode);

//void TVD_det_enable();
//void TVD_det_disable();
//u32 TVD_det_finish();
//tvd_mode_t TVD_det_mode();

void TVD_irq_enable(u32 id,tvd_irq_t irq);
void TVD_irq_disable(u32 id,tvd_irq_t irq);
u32  TVD_irq_status_get(u32 id,tvd_irq_t irq);
void TVD_irq_status_clear(u32 id,tvd_irq_t irq);

void TVD_capture_on(u32 id);
void TVD_capture_off(u32 id);

void TVD_set_addr_y(u32 id,u32 addr);
void TVD_set_addr_c(u32 id,u32 addr);

void TVD_set_width(u32 id,u32 w);
void TVD_set_width_jump(u32 id,u32 j);
void TVD_set_height(u32 id,u32 h);

void TVD_set_fmt(u32 id,tvd_fmt_t fmt);
void TVD_config(u32 interface, u32 system);
u32 TVD_get_status(u32 id);

#endif
