from setuptools import setup, find_packages

package_name = 'auto_nav'  # ✅ Define package_name at the top

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),  
    # ↑ Since we're *not* using src/, 
    #   just find_packages() will discover the auto_nav folder.
    
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),  # ✅ Uses package_name variable
        ('share/' + package_name, ['package.xml']),  # ✅ Uses package_name variable
    ],
    include_package_data=True,  # Ensures non-code files are included if needed
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nus',
    maintainer_email='nus@todo.todo',
    description='Autonomous Navigation Robot Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'r2mover = auto_nav.r2mover:main',
            'r2moverotate = auto_nav.r2moverotate:main',
            'r2scanner = auto_nav.r2scanner:main',
            'r2occupancy = auto_nav.r2occupancy:main',
            'r2occupancy2 = auto_nav.r2occupancy2:main',
            'r2auto_nav = auto_nav.r2auto_nav:main',
            'explore_and_shoot = auto_nav.explore_and_shoot:main',
            'no_nav2 = auto_nav.no_nav2:main',
        ],
    },
)
