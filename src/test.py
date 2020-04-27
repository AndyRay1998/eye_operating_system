#!/usr/bin/env python3
# coding: utf-8
import os
import subprocess
import time
import signal
'''
sub = subprocess.Popen('tmux new -s nameit', shell = True ,bufsize = -1, stdin=subprocess.PIPE,stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

print('started')
# os.system('tmux ls')
subprocess.call('tmux ls', shell=True)
time.sleep(3)
print('killing')

# os.system("gnome-terminal -x bash -c 'tmux kill-session nameit'&");
# os.kill(sub.pid, signal.SIGINT)
# sub = subprocess.call("exit 1", shell=True)
'''
p_cmd=subprocess.Popen('tmux new-session -s username -d && tmux attach -t username', shell=True, universal_newlines=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
# os.system('tmux attach -t username && echo "1"')
time.sleep(2)
os.system('echo "1"')
print('2')
print(p_cmd)
# subprocess.call('tmux ls')
# p_cmd.stdin.write('echo "1"')
