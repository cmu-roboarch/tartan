#! /usr/bin/env bash

set -x
set -e

echo 0 > /proc/sys/kernel/randomize_va_space

./setup.sh --force

export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

./replicate.py

exec "$@"
