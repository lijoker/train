# train
首先装python和程序开头的库
下载chrome浏览器
下载对应的chromedriver
self.executable_path='D:/chrom/chromedriver' 这一句中更改chromedriver的存在路径
在更改starts = u"%u5E7F%u5DDE%2CGZQ" ends = u"%u897F%u5B89%2CXAY" 在chrome中查看cookie
更改时间
更改班次
运行

## FSM 中断响应测试 demo

新增了 `fsm_interrupt_demo.c`，用于根据状态机做中断响应验证。

运行方式：

```bash
gcc -std=c11 -Wall -Wextra -O2 fsm_interrupt_demo.c -o fsm_interrupt_demo
./fsm_interrupt_demo
```

程序包含 3 个场景：

1. `HW trigger flow`：硬件触发路径，`WAIT_ACK_HW -> IDLE` 时产生中断
2. `SW flow (no flow control)`：软件触发无流控路径，`CFG_END_SW -> IDLE` 时产生中断
3. `SW flow (flow control, multi cfg)`：软件触发带流控多轮配置路径，仅最终完成时产生中断

可复用中断模拟函数：

```c
bool simulate_interrupt_and_handle(
    InterruptFSM *fsm,
    InterruptServiceStats *stats,
    const char *caller_name,
    int work_steps
);
```

这个函数可被其他业务函数调用，用于模拟：
- 进入中断上下文
- 执行一定步数的中断处理（每步包含一次 `memcpy` 数据搬运）
- 清除中断标志

新增 C 测试代码：`fsm_interrupt_api_test.c`

编译并运行测试：

```bash
gcc -std=c11 -Wall -Wextra -O2 -DFSM_DEMO_NO_MAIN fsm_interrupt_demo.c fsm_interrupt_api_test.c -o fsm_interrupt_api_test
./fsm_interrupt_api_test
```

## 中断吞吐能力测试（3000+ 次/秒）

新增 `interrupt_capacity_test.c`，用于验证 CPU 在不同中断到达模式下的响应能力。

到达模式：
- `periodic`：固定周期到达
- `burst`：突发到达（默认每次 8 个中断）
- `poisson`：泊松随机到达

默认测试参数：
- 目标速率：`3500 irq/s`
- 每种模式运行时长：`2s`
- ISR 处理负载：`work_steps=200`

编译并运行：

```bash
gcc -std=c11 -Wall -Wextra -O2 -DFSM_DEMO_NO_MAIN fsm_interrupt_demo.c interrupt_capacity_test.c -lm -o interrupt_capacity_test
./interrupt_capacity_test
```

输出中会给出：
- `handled_rate`：实际处理中断速率
- `avg_isr`/`max_isr`：ISR 平均/最大运行时长（微秒）
- `CPU_3000_PLUS=PASS/FAIL`：是否达到 3000+ 中断/秒
