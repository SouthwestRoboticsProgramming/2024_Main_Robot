#!/bin/bash

rm log.log
./messenger-server 2>&1 | tee log.log

