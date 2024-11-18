[TOC]
### 效果图
### 功能点
1. **加载指定路径下的所有图片并显示**
2. **滑动滑动条查看指定图片，也滚轮切换图片**
3. **滑动条缩略图加入动画效果**
4. **图片可以进行缩放移动查看**
### 代码解析
- ==整体来说相对，显示图片的是一个自定义的`QLabel`,缩略图是很多`QLabel`加入到一个`QScrollArea`滑动条中，使用`QPropertyAnimation`即可加入的动画效果。==
#### 图片切换显示与动画效果
- 展示指定图片时，将图片名传入，`imageLabel`重新加载指定图片。计算出图片在滑动条的中间位置，并使用`QPropertyAnimation`进行属性的设置从而达到一种动画效果。
- `QPropertyAnimation`怎么用自行百度，简单来说就是可以给QT对象任意属性设置平滑的动画。
```c++
void ImageBrowser::showImage(const QString &filename)
{
    QPixmap pixmap(dirname + filename);
    imageLabel->resert();
    imageLabel->setPixmap(pixmap.scaled(400, 400, Qt::KeepAspectRatio));

    // 动画效果，使选中的缩略图居中
    QScrollBar *scrollBar = scrollArea->horizontalScrollBar();
    int targetValue = labels[currentImageIndex]->geometry().center().x() - (scrollArea->width() / 2);
    // 创建动画并执行
    QPropertyAnimation *animation = new QPropertyAnimation(scrollBar, "value");
    animation->setDuration(1000);
    animation->setStartValue(scrollBar->value());
    animation->setEndValue(targetValue);
    animation->start();

    // 移除之前选中的缩略图的边框
    for (auto label : labels)
    {
        label->setStyleSheet("");
    }
    // 为当前选中的缩略图添加边框
    auto currentThumbnail = labels[currentImageIndex];
    currentThumbnail->setStyleSheet("border: 4px solid lightskyblue;");
}
```

#### 图片缩放
- 略，拉取源码查看

### 总结
- 知识理应共享,[源码](https://gitee.com/shan-jie6/my-case/tree/master/QT/pictureBrowser)在此。
- 这个案例相对简单，切换图片使用的是滚轮，当然也可以使用按钮。后续也可以加入预览gif图片的功能，路径是写死的这也很好改。。。