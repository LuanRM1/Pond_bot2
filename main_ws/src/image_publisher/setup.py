from setuptools import setup

package_name = 'image_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seu_nome',
    maintainer_email='seu_email@example.com',
    description='Descrição do pacote image_publisher',
    license='Licença',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = image_publisher.publisher:main'
        ],
    },
)

