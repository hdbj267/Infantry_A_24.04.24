没限功率版（纯净版）

步兵后续规划（2025开始）：

1.更换新的限功率的方案

2.自瞄调pid

3.完成功率控制板与a板的can通信

4.解决ui界面不推送的问题

5.之后麦轮代码更换为全向轮代码

重点关注下面的task和app(重要度排序)： chassis_task/

connect_task/

can1_app/

can2_app/

judge/裁判系统移植，每年都要去官网看看有没有更新串口协议，工作就是替换帧头和结构体内容

start_task/任务添加，堆栈大小设置

初此之外，还要关注

stm32f4xx_it.c/

其他的基本上用不上/不重点关注
