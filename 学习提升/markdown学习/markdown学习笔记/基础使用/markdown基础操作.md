<center><span style="font-size:2rem;font-weight:bold;">MarkDown基础操作命令</span></center>

| 版本   | 日期       | 作者   | 备注             |
| ------ | ---------- | ------ | ---------------- |
| V1.0.0 | 2022.09.05 | 庞明慧 | markdown基础操作 |

<center><span style="font-size:2rem;font-weight:bold;">MarkDown基础操作命令</span></center>

[toc]

<div style="page-break-after: always;"></div>

# 快捷键

## 多级标题

```
ctrl+num(1-6)
```

## 换行不换段

```
shift + enter
```

## 换段

```
ctrl + enter
```

## 加粗

```
ctrl + b
```

## 插入连接

```
ctrl + k
```

## 插入代码

* 插入代码 
  ```
  `一行代码`
  ```

* 插入代码块
  ````html
  ```
  代码块
  ```
  ````

## 斜体

单个`*`号或者单个下划线`_`,例子：
```
*斜体文字*
_斜体文字_
```

## 粗体

两个`*`号
```
**粗体文字**
```

# 文档标题

```html
<center><span style="font-size:2rem;font-weight:bold;">MarkDown基础操作命令</span></center>
```

# 换页符

```html
<center><span style="font-size:2rem;font-weight:bold;">MarkDown基础操作命令</span></center>
```

# 目录

```html
[toc]
```

# 表格

<span id="插入表格">插入表格</span>
输入|first Header |Second Header |并按下回车操作将创建一个包含两列的表。
创建表后，焦点在该表上将弹出一个表格工具栏，可以在其中调整表格，对其或者删除表格，还可以使用上下文菜单来复制和添加删除列行。

## markdown表格源代码

```
|First Header| Second Header|
|------------|--------------|
|content cell|content cell  |
|contenc cell|content cell  |
```

也可以通过在标题行中包含冒号`:`,将文本定义为左对齐，右对齐或者居中对齐。
```
| Left-Aligned  | Center Aligned  | Right Aligned |
| :------------ |:---------------:| -----:|
| col 3 is      | some wordy text | $1600 |
| col 2 is      | centered        |   $12 |
| zebra stripes | are neat        |    $1 |
```

##  Typora生成表格

| First Header | Second Header |
| ------------ | ------------- |
|              |               |

## 表格内部换行

在表格的单元格里换行，需要使用`<br />`或者`<br>`，快捷键为 `Shift+Enter`。
例子：

```
| Header1 | Header2                            |
| ------- | ---------------------------------- |
| item1   | 1. one <br />2. two <br />3. three |

```

# 标题

除了`Ctrl+数字`的方式创建标题外，markdown源码使用`#`的方式创建标题
```
# 一级标题
## 二级标题
### 三级标题
#### 四级标题
##### 五级标题
###### 六级标题
```

**注意：每个`#`后面都需要空一格，且markdown最多支持6级标题**

# 列表

## 无序列表

输入`*`、`+`、`-`可以创建一个无序列表

## 有序列表

输入`1. `、`2. `、`...`可以创建有序列表

## 任务列表

任务列表是标记为`[]`或者`[x]`（完成或者未完成）的项目的列表，例如：
```
-[] 这是一个任务列表项
-[x] 该任务完成 
```

**注意：`-`号后需加一个空格，且方括号内需加一个空格，且方括号与任务描述需加一个空格**

# 数学公式

输入`$$`后输入latex渲染的公式符号，例子：
$$
{\LARGE
\color{blue}
\begin{align}
x &= 输入蓝色超大字体公式，加入公式序号 \\
y &= 输入红色超大字体公式，加入公式序号 \\
z &= 需要对齐等号
\end{align}
}
$$


## 显示公式字体的颜色

```
{\color{Blue}   }      //蓝色字体
{\color{Red}    }      //红色字体
{\color{Green}  }      //绿色字体
{\color{Purple} }      //紫色字体
{\color[RGB]{0,0,0} }  //自定义颜色  
```

## 插入分数

```
\frac{}{}
```

## 插入微分

```
\dot{}
\ddot{}
\dddot{}
```

## 插入次方

```
^{}
```

## 插入矩阵

3*3加中括号矩阵

```
{
\begin{bmatrix}
 &  & \\ 
 &  & \\ 
 &  &
\end{bmatrix}
}
```

## 公式换行+对齐

```
 \\    换行
 &     对齐
```

## 开放根

```
\sqrt[2]{x}
\sqrt{x}
```

# 图片

插入图片之前需要添加额外的`!`字符，语法如下所示：
```
![替代文字](/path/to/img.jpg)

![替代文字](/path/to/img.jpg "可选标题")
```

# 文字

## 粗体文字

快捷键：`ctrl + k`
或者使用：`**加粗文字**`

## 下划线

快捷键：`ctrl + u`
或者使用：`<u>下划线文字</u>`

## 斜体文字

快捷键：`ctrl + I`
或者使用：`*斜体文字*`或者`_斜体文字_`

## 删除线文字

快捷键：`Shift+Alt+5`
或者使用：`~~删除文字~~`

## 标签样式文字

`<test> 文字示例 </test>`

## 水平分割线

一行中使用三个或以上的`*`、`-`即可添加分隔线，独占一行才能生效，前后不能有其他字符。

## HTML文字样式

​	推荐使用样式`style="font-family:黑体; color:red; font-size:16px; background: green"`来设置字体、颜色、大小和背景：

```html
<p style="font-family:黑体; color:red; font-size:16px">我是黑体</p>
<p style="font-family:微软雅黑; color:green; font-size:24px">我是微软雅黑</p>
<p style="font-family:华文彩云; color:#6666ff; font-size:32px; background: #66ff66">我是华文彩云</p>
```

# 超链接

## 直接插入网址

```
百度网页：<https://www.baidu.com>
```

## 链接

* 快捷键`ctrl + k`

* `[链接名字](网址)`
  例子：

  ```
  [百度网址](https://www.baidu.com)
  ```

* 本地文件超链接
  同上：`[链接名称](相对地址)`

## 文章内部跳转

* 目录跳转
  使用`[toc]`生成目录
* 标题跳转
  跳转到某个标题位置：`[名称](#标题名称)`，例子：
  [查看快捷键](#快捷键)

* 任意跳转：[名称](#标题名称)
  想要跳转到任意位置需要设置锚点，即跳转点：`<span id="table_id">名称</span>`
  **注意：其中id是必填项，名称可填可不填，内容随意**
  例子：[如何插入表格](#插入表格)
