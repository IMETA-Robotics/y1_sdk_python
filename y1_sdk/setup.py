from setuptools import setup, Extension, find_packages
import os
import pybind11
import platform

current_dir = os.path.dirname(os.path.abspath(__file__))

# 库路径
sdk_lib = os.path.join(current_dir, "y1_sdk", "lib")

# 根据架构选择
arch = platform.machine()
if arch in ['x86_64', 'AMD64']:
    lib_path = os.path.join(sdk_lib, "x64")
    lib_name = "y1_sdk_x64"
elif arch in ['aarch64', 'arm64']:
    lib_path = os.path.join(sdk_lib, "arm64")
    lib_name = "y1_sdk_arm64"
else:
    raise RuntimeError(f"Unsupported architecture: {arch}")

ext_modules = [
    Extension(
        "y1_sdk.y1_sdk",   # Python 导入的模块名
        ["y1_sdk_interface.cpp"],
        include_dirs=[
            pybind11.get_include(),
            current_dir,
        ],
        library_dirs=[lib_path],
        libraries=[lib_name, 'glog'],  # 不带前缀 lib 和后缀 .so
        language="c++",
        extra_compile_args=["-O3", "-std=c++14"],
        runtime_library_dirs=["$ORIGIN/lib/x64", "$ORIGIN/lib/arm64"],
    )
]

setup(
    name="y1_sdk",
    version="0.1.0",
    author="xiaofanZhang",
    description="Python SDK wrapper for Y1SDKInterface",
    packages=find_packages(),
    ext_modules=ext_modules,
    zip_safe=False,
    install_requires=[
        "pybind11>=2.10.0",
    ],
    include_package_data=True,
    package_data={
        "y1_sdk": ["lib/x64/*.so", "lib/arm64/*.so"],
    },
    python_requires=">=3.7",
)
