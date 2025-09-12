依赖:
sudo apt install -y pybind11-dev python3-pybind11
pip install pybind11

### 构建y1_sdk pyhton sdk
1. 编译和安装
mkdir build && cd build
cmake ..
make
make install

2.设置Python路径
有几种方式可以导入模块：
方法一：修改PYTHONPATH（临时）
export PYTHONPATH="${PYTHONPATH}:/home/ubuntu/IMETA_LAB/y1_sdk_python/api"
python3 -c "import y1_sdk; print('Success!')"

方法二：创建setup.py进行正式打包
创建一个 setup.py 文件：

安装打包工具：
pip install build twine
<!-- python3 -m pip install --upgrade setuptools wheel build twine -->

然后在项目根目录运行：
python3 -m build

python3 -m build --sdist

上传分发文件到 PyPI:
python3 -m twine upload dist/*
在执行过程中会提示输入 PyPI 的用户名／密码／token

用户安装测试:
pip install y1_sdk