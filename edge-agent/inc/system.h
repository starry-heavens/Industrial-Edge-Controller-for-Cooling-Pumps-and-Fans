#ifndef SYSTEM_H
#define SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 初始化：创建 /run/edge 目录，记录启动时间，打印 "app started"。
 * 成功返回 0，失败返回 -1。
 */
int  system_init(void);

/**
 * 更新一次状态：写 /run/edge/status.json（原子替换），打印 "alive uptime=Ns"。
 * 需先调用 system_init()。
 */
void system_update(void);

/**
 * 阻塞循环：每隔 1 秒调用一次 system_update()，永不返回。
 * 适合放在独立线程或直接作为主循环体。
 */
void system_run(void);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_H */