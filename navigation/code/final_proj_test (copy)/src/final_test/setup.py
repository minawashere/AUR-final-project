from setuptools import setup

package_name = 'final_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name,
        package_name + '.filters',  # Include filters package
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'paho-mqtt','msgpack'],
    zip_safe=True,
    maintainer='abood',
    maintainer_email='your_email@example.com',
    description='Odometry node that integrates MQTT for telemetry data.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test=final_test.test:main",  
            "test_mqtt=final_test.test_mqtt:main",
            "sub_mqtt=final_test.sub_mqtt:main",
            "esp_sub=final_test.esp_subscriber_node:main",
            "esp_calc=final_test.esp_calc_node:main",
            "esp_pub=final_test.esp_publisher_node:main",
        ],
    },
)
