#!/bin/bash

/sbin/sshd -D &

gosu rosuser /bin/zsh
