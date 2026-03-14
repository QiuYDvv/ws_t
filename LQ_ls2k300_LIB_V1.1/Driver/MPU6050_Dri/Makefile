# 本文件中只需要修改 KDIR 的路径，指定一下内核根目录的路径
KDIR:=/home/lqkj/99_pai/ls2k300-linux-4.19/
ARCH=loongarch
CROSS_COMPILE=loongarch64-linux-gnu-
PWD?=$(shell pwd)

BUILD_DIR=build
INCLUDE=inc
KO_DIR=ko

EXTRA_CFLAGS += -I$(PWD)/$(INCLUDE)/

obj-m += LQ_MPU6050_dev.o
obj-m += LQ_MPU6050_dri.o

all:
	make -C $(KDIR) M=$(PWD) modules
	@find . -path ./$(BUILD_DIR) -prune                 \
	        -o -type f ! -name '*.c' ! -name '*.h'      \
		    ! -name 'Makefile' -exec mv {} $(BUILD_DIR) \;
	@find . -path ./$(BUILD_DIR) -prune -o -type f -name '*.mod.c' -exec mv {} $(BUILD_DIR) \;
	@find . -path ./$(KO_DIR) -prune -o -type f -name '*.ko' -exec mv {} $(KO_DIR) \;
clean:
	rm $(BUILD_DIR)/* $(BUILD_DIR)/.LQ* $(KO_DIR)/* .tmp* -rf
