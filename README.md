![longqiu](./MD_Image/longqiu.png)

# 龙邱科技 龙芯`2k300`久久派开源例程库

### 1- 简介
`LQ-ls2k300_ELIB-V1.1`库，针对龙芯`2k0300`久久派常用外设资源和龙邱产品模块移植驱动例程，以方便参加智能车竞赛和使用我们产品的人入门学习使用。

工程默认使用开发环境为`Linux`系统下 ，可以使用`Ubuntu`或`WSL`，软件可使用`VScode`，请自行修改工程路径相关配置。

### 2- 开发环境

- 硬件平台：龙芯`2k0300`久久派/龙邱久久派拓展板

<img src="./MD_Image/image-龙芯2k0300久久派.png" alt="image-20250303113807517" style="zoom:67%;" />

如需了解龙芯久久派拓展版相关信息，可点击链接：[龙芯久久派拓展板](https://item.taobao.com/item.htm?id=878340053033)了解详情。

<img src="./MD_Image/image-龙芯久久派拓展板.png" alt="image-20250303114029223" style="zoom:80%;" />

- 开发及编辑环境 `Ubuntu`+`VScode 1.93` 以上

### 3- 使用说明

1. 安装`Ubuntu`或`WSL`并安装`VScode`环境

2. 下载或克隆库，本链接或购买产品的附赠资料中

3. 打开软件导入工程。

4. 项目介绍：
   - <font color="red">`LQ_ls2k300_LIB_V1.1`</font>是更新后的不带有测试例程的项目工程
   - <font color="red">`LQ_Test_Demo`</font>是更新后的带有测试例程的项目工程
   - <font color="red">`ls2k300-linux-4.19.zip`</font>是龙芯的内核文件
   - <font color="red">`SPI1设备文件配置方法.pdf`</font>是设备树配置spi1的教程，如果使用软件SPI或内核屏幕驱动来驱动TFT屏幕，则不用查看该文件
   - <font color="red">`龙邱-龙芯2k0300开源库使用介绍.pdf`</font>是该开源库的简单使用介绍
   - <font color="red">`根据龙芯久久派拓展板的引脚资源分配.xlsx`</font>是根据拓展板写的引脚资源分配表格
   - <font color="red">`MD_Image`</font>是存放的`README.md`中的图片
   - 其他详细教程内容可在B站查看相关教程：[龙邱科技的个人空间-龙邱科技个人主页-哔哩哔哩视频](https://space.bilibili.com/95313236)




### 4- 更新日志

   1. 更新内容 详见工程目录下的**历史版本更新记录.txt**文件


​         

###  5-其他核心板类开源库

   龙邱-核心板类开源库百度网盘链接：[https://pan.baidu.com/s/1exDJTBU4HdRVE5ne6-5LCA](https://gitee.com/link?target=https%3A%2F%2Fpan.baidu.com%2Fs%2F1exDJTBU4HdRVE5ne6-5LCA) 提取码：7sa3

   其他开源库，陆续整理中。。。后续也会同步gitee

### 6-关于资讯

   其他关于龙邱科技，智能车相关资讯，敬请关注龙邱官方微信公众号：

   ![image-20250218135059980](./MD_Image/%E5%BE%AE%E4%BF%A1%E5%85%AC%E4%BC%97%E5%8F%B7%E4%BA%8C%E7%BB%B4%E7%A0%81.png)

   更多智能车和公司动态信息、文章会在此发布！




----




#### 特技

1.  使用 Readme\_XXX.md 来支持不同的语言，例如 Readme\_en.md, Readme\_zh.md
2.  Gitee 官方博客 [blog.gitee.com](https://blog.gitee.com)
3.  你可以 [https://gitee.com/explore](https://gitee.com/explore) 这个地址来了解 Gitee 上的优秀开源项目
4.  [GVP](https://gitee.com/gvp) 全称是 Gitee 最有价值开源项目，是综合评定出的优秀开源项目
5.  Gitee 官方提供的使用手册 [https://gitee.com/help](https://gitee.com/help)
6.  Gitee 封面人物是一档用来展示 Gitee 会员风采的栏目 [https://gitee.com/gitee-stars/](https://gitee.com/gitee-stars/)
