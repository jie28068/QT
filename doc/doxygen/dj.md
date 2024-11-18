### 介绍

**Doxygen是一个开源的文档生成工具，主要用于从源代码文件中提取注释并生成文档。它支持多种编程语言，包括C++, C, Java, Python, PHP等。Doxygen能够识别特定的注释格式，并根据这些注释生成漂亮的文档，文档可以输出为HTML, LaTeX, RTF, XML等多种格式。**
Doxygen通常被开发人员用于编写和维护项目文档，特别是开源项目，以帮助其他开发人员理解和使用代码库。
Doxygen本身是一个独立的项目，但有一些工具可以与其配合使用，例如：

- **Graphviz**：Doxygen可以使用Graphviz来生成更为复杂的图形，如调用图和类继承图。
- **HTML Help WorkShop**：Doxygen 软件默认生成HTML文件或Latex文件， 我们要通过HTML生成CHM文档， 需要先安装HTML Help WorkShop 软件，并在Doxygen中进行关联

---

### 安装

- 下面为自己用的版本
<https://cloud.189.cn/web/share?code=ZfQjeme2iIZj（访问码：8s2y）>Doxygen
<https://cloud.189.cn/web/share?code=UBrmEzfma6Rz（访问码：of7e）>Graphviz
<https://cloud.189.cn/web/share?code=JZFFBfemiIV3（访问码：tjv9）>HTML
- 也可以去官网下
- 软件安装都选择默认方式，点击下一步直至安装完成

### 配置

1. Wizard->Project 最重要的是**工作目录，源代码目录，生成参考文件目录**三处的设定 ，其它项目名称、项目简介、版本和标识可以依照实际情况选填。工作目录是新建的一个目录，在配置完成之后可以把配置文件存在这个目录里，每次从这个目录中导入配置文件（.cfg），然后进行说明文档生成。

2. Wizard- >Mode 选择编程语言对应的最优化结果，按照编程语言选择。

3. Wizard- >Output  选择 输出格式，选**HTML下的（.chm）项**，为最后生成chm做准备。由于不需要LaTeX结果，不选此项。

4. Wizard- >Diagrams 选择dot tool项，通过==GraphViz==来作图（前面要求安装的，不安装将不会有）。

5. Expert-> Project 选择输出目录，选着输出语言。如果代码中采用了中文注释，此处选择为中文。

向下拉滑条，看见有`JAVADOC_AUTOBRIEF` 和`QT_AUTOBRIEF`两个框，如果勾选了，在这两种风格下默认第一行为简单说明，以第一个句号为分隔；如果不选，则需要按照Doxygen的指令@brief来进行标准注释。

6. Expert-> Input 将输入编码方式改为**GBK**方式，确保输出中不会由于UTF-8方式导致乱码。

最后也是经常遇到的问题就是DoxyGen生成的CHM文件的左边树目录的中文变成了乱码 。这个 只需要将**chm索引的编码类型修改为GB2312**即可。 在HTML的`CHM_INDEX_ENCODING`中输入GB2312即可。

7. Expert-> HTML 勾选生成`HTMLHELP`项，输入生成CHM名称，在`HHC_LOCATION`中填入`HTMLHELP WORKSHOP`安装目录中**hhc.exe**的路径，将chm编码方式改为**GBK**方式，与第（6）步中的输入编码方式一致。

8. Expert->Dot 在`Dot_PATH`中填写**GraphViz**的安装路径。

需要在build中配置`EXTRACT_ALL和LOCAL_METHODS`才能生成所有的变量和函数。

9. 存储配置信息。 到上一步Doxygen已经完全配置好，可以在Run中点击运行了，但为了保存以上配置信息，可以将配置好的文件存一个.cfg文件，之后再运行Doxygen时只需要将该文件 用Doxygen 打开 ，改变第（1）步中的输入、输出目录及工程的信息再运行。
File->Save as, 取一个名，默认为Doxyfile，加.cfg,点击保存。如果需要改变配置文件，改动之后再Save替换之前的配置文件即可。

10. Run->Run Doxygen 即可运行Doxygen，运行完成后在输出目录中的html文件夹中找到index.chm文件即为输入代码的文档说明。
