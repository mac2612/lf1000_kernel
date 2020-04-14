#!/bin/bash

set +x

./install.sh
if [ "$?" != "0" ]; then
	exit $?
fi

../scripts/make_cbf.py
cp kernel.cbf /home/lfu

pushd $ROOTFS_PATH
tar -cvf modules.tar ./lib/modules
cp modules.tar /home/lfu
popd
