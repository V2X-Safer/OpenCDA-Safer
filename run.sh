#!/bin/bash

# 终止已存在的进程，如果存在的话
pkill -f "python src/main.py" || true

rm -f src/log/param/current_time.txt
rm -rf data_dumping/*

# 正确激活conda环境
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate opencood

# 在后台运行程序并将输出重定向到nohup.out
nohup python src/main.py > nohup.out 2>&1 &

# 获取最后一个后台作业的PID并分离
bg_pid=$!
echo "程序已在后台启动，PID: $bg_pid"
disown $bg_pid

