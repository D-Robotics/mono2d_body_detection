# Changelog for package mono2d_body_detection

v1.0.1 (2022-06-23)
------------------
1. 发布的AI msg中增加perf信息输出，用于推理性能分析。
2. 更新订阅到的输入和发布的推理输出帧率的统计方式，直接使用dnn node输出的统计数据。
3. 更新不同日志输出的log级别，WARN级别只输出订阅和发布的消息帧率，INFO级别输出消息内容。
