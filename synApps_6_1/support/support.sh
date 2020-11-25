#!/bin/bash

python support_mod.py 1
make release
python support_mod.py 2
make release

