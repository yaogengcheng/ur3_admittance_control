#!/usr/bin/env python3
#-*- coding:utf-8 -*-
import socket 

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# 连接服务端
s.connect(('127.0.0.1', 8999))

# 请求 | 发送数据到服务端
# s.sendall(b'hello')

# 响应 | 接受服务端返回到数据
data = s.recv(1024)
res =list(map(float, data.strip().split()))
print("11")
print(res) # hello

# 关闭 socket
s.close()