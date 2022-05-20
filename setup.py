from skbuild import setup


setup(
    name='yolo3d',
    version='0.0.1',
    author='Ryan P. Daly',
    author_email='your@email.com',
    description='Add description here',
    long_description='',
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],
    zip_safe=False,
    packages=["yolo3d"],
    cmake_args=[
        "-DBUILD_TESTING=OFF",
        "-DBUILD_DOCS=OFF",
    ],
    package_dir={"": "python"},
    cmake_install_dir="python/yolo3d",
)
