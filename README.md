# Marvell FC HBA 驱动源代码 

## 介绍
为openEuler 社区提供Marvell QLogic FC HBA 卡驱动

## 驱动信息
Marvell QLogic FC HBA卡驱动源代码.

源代码上游来自Marvell 官方驱动下载网站 [Marvell/QLogic webisite](https://www.marvell.com/support/downloads.html)




### 驱动版本
> qla2xxx-x.y.z.







## 安装方法

安装.tar.gz格式的源码包步骤
a.   执行“tar –zxvf qla2xxx-src--<ver>.tar.gz命令解压源码包。
b.   执行“cd qla2xxx-<ver>/”命令进入源码包目录。
c.   在源代码解压后的根目录执行“./extras/build.sh initrd”命令进行编译安装
d.   驱动安装完成后，重启系统或依次执行“modprobe -r qla2xxx”命令卸载旧驱动和“modprobe -v qla2xxx”命令加载新驱动模块，让新驱动生效。
e.   执行“modinfo qla2xxx”命令或"cat /sys/module/qla2xxx/version"命令，查看驱动版本，确认新的驱动已生效。


### QLogic FC HBA driver

1. This repository is for provide driver source code.
2. this qla2xxx driver supports openEuler 20.03 LTS SP3 and openEuler 22.03 LTS version.
3. this qla2xxx driver supports ARM64 and X64 platform.
4. this qla2xxx driver supports Kunpeng and Phytium CPU platform.

