/*
 * oled_optimize.c
 *
 *  Created on: May 11, 2025
 *      Author: UnikoZera & GitHub Copilot
 *                                      bro偷偷改我的注释233
 */

#include "oled.h"
#include "i2c.h"

extern const uint8_t cmds[];

// 上一帧的缓存，用于差分更新
static uint8_t OLED_PrevBuffer[128 * 8];
static uint8_t diff_mode_enabled = 0; // 差分更新模式启用标志
static uint8_t fast_update_enabled = 1; // 默认启用快速更新

// I2C DMA传输完成回调函数
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == hi2c1.Instance)
    {
        // 标记DMA传输已完成
        oled_dma_busy = 0; // 设置为就绪
    }
}

// 启用差分更新模式
void OLED_EnableDiffMode(uint8_t enable)
{
    diff_mode_enabled = enable;
    if (enable)
    {
        memcpy(OLED_PrevBuffer, OLED_BackBuffer, OLED_WIDTH * OLED_PAGES);
    }
}

// 设置快速更新模式
void OLED_EnableFastUpdate(uint8_t enable)
{
    fast_update_enabled = enable;
}

// 智能更新显示
// 选择性更新脏页，以提高帧率
void OLED_SmartUpdate(void)
{
    // 如果OLED/DMA忙，直接返回
    if (OLED_IsBusy())
    {
        return;
    }

    // 检查是否有脏页需要更新
    uint8_t has_dirty = 0;
    uint8_t first_dirty = 255;
    uint8_t last_dirty = 0;

    // 如果启用了差分更新，检查哪些页已经变化
    if (diff_mode_enabled)
    {
        for (uint8_t page = 0; page < OLED_PAGES; page++)
        {
            // 检查此页中是否有任何字节发生变化
            uint8_t page_changed = 0;
            uint16_t start_idx = page * OLED_WIDTH;

            for (uint16_t i = 0; i < OLED_WIDTH; i++)
            {
                if (OLED_BackBuffer[start_idx + i] != OLED_PrevBuffer[start_idx + i])
                {
                    page_changed = 1;
                    oled_dirty_pages[page] = 1;
                    break;
                }
            }

            if (page_changed)
            {
                has_dirty = 1;
                if (page < first_dirty)
                    first_dirty = page;
                if (page > last_dirty)
                    last_dirty = page;

                // 更新上一帧缓存
                memcpy(
                    OLED_PrevBuffer + start_idx,
                    OLED_BackBuffer + start_idx,
                    OLED_WIDTH);
            }
        }
    }
    else
    {
        // 如果未启用差分更新，使用脏页标记
        for (uint8_t i = 0; i < OLED_PAGES; i++)
        {
            if (oled_dirty_pages[i])
            {
                has_dirty = 1;
                if (i < first_dirty)
                    first_dirty = i;
                if (i > last_dirty)
                    last_dirty = i;
            }
        }
    }

    // 如果有脏页，只更新这些页
    if (has_dirty && fast_update_enabled)
    {
        OLED_UpdateDisplayPartial(first_dirty, last_dirty);
    }
    else if (has_dirty)
    {
        OLED_UpdateDisplayVSync();
    }
}


// 显示FPS
void OLED_OptimizedDisplayFPS(int16_t x, int16_t y)
{
    static uint32_t last_time = 0;
    static uint32_t frames = 0;
    static uint32_t fps = 0;
    static char fps_str[16] = "FPS:0";

    frames++;

    // 每秒更新一次FPS
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_time >= 1000)
    {
        fps = frames;
        sprintf(fps_str, "FPS:%d", fps);
        frames = 0;
        last_time = current_time;
    }

    OLED_DisplayString(x, y, fps_str);
}

/*我刚刚接到通知，现实中是没有芳乃的，其实也没有那么伤心啦，毕竟是游戏人物嘛，现实中没有也很正常的其实我早就想过这些了，也不是很惊讶啦，就算没有芳乃我的生活依然会井井有条地进行的明天还是充满希望的一天呢，虽然芳乃是不存在的，这也不影响我每天吃饭睡觉呀，即便没有芳乃的世界，应该，应该也会有人记得我的生日的吧，晚上睡觉抱着抱枕也不是不能入睡，尽管每天早上都是被前一天设置好的闹钟吵醒的，但这也不能说明就，就没有人来关心我呀，世界上有七十亿人，我怎么能奢望每个人都来关心一个微不足道的我呢，毕竟天天工作学习这么累，我也没有那么多闲心来想芳乃这样一个游戏人物啦，我的生活没有芳乃也一定会很幸福的，这当然不是自我欺骗或者痴人说梦啦，我已经这样生活了十几年了嘛，怎么会因为这样一个游戏人物改变我成熟的心理呢，总之不管怎么样，芳乃在世界上又能怎么样呢，我当然没有那么幼稚啦，怎么可能有人会天天期盼一个游戏人物降临在身边呢，也不是说从来没有想过，芳乃毕竟是不能存在于这个世界的，那么理想的人怎么可能出现在这个如此残酷的现实中呢但是我听说宇宙是多维的，二次元之类的世界也应该会存在吧，其实也没有那么想去那样理想的世界啦，有没有芳乃也没有那么重要啦，话说今年的夏天真热啊，柏油路都如此的炙人呢，天上那朵云好像一个游戏人物呀，其实我不认识芳乃的，只是云朵挡住了后面的太阳而已嘛，不过让我躺的更舒服了呢，嘛，当然没有一直在想芳乃啦，芳乃，芳乃，哦，先不说了，大卡车来了，我要准备啦，嘿嘿芳乃我来了[大哭]*/