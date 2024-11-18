Qt中的处理图像的类介绍以及QPixmap、QImage、QSvgRenderer间转换
[TOC]

### 常见图片类

- 在Qt框架中，QPixmap、QImage和QSvgRenderer是用于处理图像的三个不同类，它们各自有不同的用途和特点。

1. **QPixmap**：
   - QPixmap是用于处理像素图的类，它专门为在屏幕上显示图像而优化。QPixmap可以高效地处理大图像，并且与Qt的窗口系统紧密集成。
   - 它通常用于显示**图标**、位图和图像，特别是在用户界面中。QPixmap提供了快速的像素访问和绘制操作，可以直接被绘制到任何`QPaintDevice`派生的对象上，如`QWidget`和`QGraphicsItem`。
   - QPixmap针对屏幕渲染进行了优化，因此当需要**频繁在屏幕上绘制图像时**，使用QPixmap是最佳选择。
2. **QImage**：
   - QImage是用于处理图像数据的类，它提供了对图像像素的**直接访问**。QImage可以用于加载、保存和操作图像文件，支持常见的图像格式，如PNG、JPEG和BMP。
   - QImage适用于图像处理和图像编辑，因为它允许对单个像素进行操作。它还支持图像的转换、缩放和镜像等操作。
   - 与QPixmap不同，QImage不依赖于屏幕分辨率，因此在处理图像时，QImage可以提供更精确的像素控制。但是，**当需要在屏幕上显示QImage时，需要将其转换为QPixmap**。
3. **QSvgRenderer**：
   - QSvgRenderer是用于渲染SVG（可缩放矢量图形）图像的类。SVG是一种基于**XML**的图像格式，它提供了高质量的图像，并且可以无损地缩放到任何大小。
   - QSvgRenderer可以将SVG图像渲染到任何QPaintDevice派生的对象上，如`QWidget`和`QGraphicsItem`。它支持缩放、旋转和平移等变换。
   - 由于SVG图像是矢量图形，因此它们可以**无限放大而不失真**。这使得QSvgRenderer特别适合于需要高质量图像和灵活缩放的应用程序。
4. **QBitmap**：
   - QBitmap是QPixmap的一个**子类**，它主要用于创建单色（1位深度）图像，通常用于创建**光标、笔刷和掩码**。QBitmap通常与`QCursor`和`QBrush`一起使用。
5. **QIcon**：
   - QIcon是一个用于处理图标的类，它可以包含多个不同大小的图像，以便在应用程序中根据需要自动选择最合适的大小。QIcon可以接受QPixmap作为参数，并且可以在**工具栏、菜单和对话框**等地方使用。

- ==总的来说，QPixmap适合于屏幕显示和用户界面中的图像，QImage适用于图像处理和编辑，而QSvgRenderer则适用于渲染高质量的矢量图形==。

***

### 互转

在Qt中，QPixmap、QImage和QSvgRenderer之间可以相互转换，以便在不同的场合使用。以下是如何在这三种图像类型之间进行转换的方法：

1. **QPixmap转换为QImage**：
   要将QPixmap转换为QImage，有如下俩种方式：

   ```cpp
   QPixmap pixmap = ...;
   QImage image = QImage(pixmap);
   QImage image2 = pixmap.toImage();
   ```

2. **QImage转换为QPixmap**：
   要将QImage转换为QPixmap，可以使用QPixmap的构造函数，该构造函数接受QImage作为参数。例如：

   ```cpp
   QImage image = ...;
   QPixmap pixmap = QPixmap::fromImage(image);
   ```

3. **QSvgRenderer转换为QPixmap**：
   QSvgRenderer本身并不直接存储图像数据，它是一个用于渲染SVG图像的类。要将SVG图像渲染为QPixmap，可以使用QPainter来绘制QSvgRenderer的内容到一个空的QPixmap上。例如：

   ```cpp
   QSvgRenderer renderer("path/to/your/svgfile.svg");
   QPixmap pixmap(300, 300); // 设置适当的大小
   pixmap.fill(Qt::transparent); // 确保背景透明
   QPainter painter(&pixmap);
   renderer.render(&painter);
   ```

4. **QSvgRenderer转换为QImage**：
    按上述方式先转为QPixmap，再转为QImage。

- 因为SVG本质上就是XML，所以`QSvgRenderer renderer(data);`,里面的data就可以是一个`QVariant`转为`QByteArray`的数据了，它也是有这个构造函数的。
- 转换过程中可能需要考虑图像的分辨率和颜色格式。例如，当从QImage转换为QPixmap时，如果QImage是32位颜色深度的，那么转换后的QPixmap也将保持32位颜色深度。同样，在将QSvgRenderer渲染到QPixmap或QImage时，需要确保目标图像的大小和背景颜色符合预期。

***

### 其他图片类

- 在Qt中，除了QPixmap、QImage和QSvgRenderer之外，还有一些其他的类和模块可以用于处理图片：

1. **QPicture**：
   - QPicture是一个可以记录和重放QPainter命令的类。它将QPainter的绘制操作保存到一个文件中，这个文件可以在以后被读取并重新绘制到任何QPaintDevice上。QPicture是平台无关的，它保存的是绘图命令，而不是像素数据，因此可以无损地缩放。
2. **QImageReader/QImageWriter**：
   - QImageReader和QImageWriter是用于读取和写入图像文件的类。它们支持多种图像格式，如PNG、JPEG和BMP。QImageReader提供了对图像进行解码的功能，而QImageWriter则用于将QImage保存到文件中。
3. **QOpenGLWidget/QOpenGLFunctions**：
   - 如果你的应用程序使用OpenGL进行图形渲染，那么可以使用QOpenGLWidget来绘制和处理图像。QOpenGLFunctions提供了OpenGL核心或兼容版本的函数指针，以便在Qt应用程序中使用OpenGL。
