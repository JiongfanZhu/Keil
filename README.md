# README

这个文档是用于电工基地控制组2022年秋季培训部分红绿灯代码部分的解释说明

## 用户端操作说明

首先通过手机蓝牙连接模块(默认已经配置好蓝牙模块的相关设置)。

注意，你所发送的所有信息应以**UTF-8**编码发生，且在**末尾必须加上空格**(用于区别两条信息并以'\0'作为第二条信息的启动信号)。

若想要改变红绿灯的工作模式，发送**"SwitchL "**，将会返回“light mode %d”，0对应自动模式，1对应手动模式

若想要改变指示牌的工作模式，发送**"SwitchS "**，将会返回“sign mode %d”，0对应自动模式，1对应手动模式

若想要在红绿灯的手动模式下指定工作灯，发送**"L0 ",“L1 ”,"L2 "**，对应红黄绿

若想要在指示牌的手动模式下指定反转状态，发送**"S0 ",“S1 ”**，对应A面B面